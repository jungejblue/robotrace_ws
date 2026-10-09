import importlib.util
from pathlib import Path
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'summarize_metrics.py'
SPEC = importlib.util.spec_from_file_location('summarize_metrics', SCRIPT)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


class MetricsTests(unittest.TestCase):
    def summarize_text(self, text):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'metrics.csv'
            path.write_text(text)
            return module.summarize(path)

    def test_known_quantiles_and_empty(self):
        self.assertEqual(module.quantile([40, 10, 30, 20], 0.5), 25)
        self.assertAlmostEqual(module.quantile([10, 20, 30, 40], 0.95), 38.5)
        self.assertAlmostEqual(module.quantile([10, 20, 30, 40], 0.99), 39.7)
        self.assertIsNone(module.quantile([], 0.5))

    def test_first_row_separate_tracking_fraction_and_result_age(self):
        report = self.summarize_text(
            'stamp,state,preprocess_ms,registration_ms,validation_ms,total_ms,scan_age_s,result_age_s,dropped_scans,reason\n'
            '1,GNSS_SEEDED,100,20,10,130,0.01,0.14,0,first\n'
            '2,TRACKING,5,10,2,17,0.01,0.03,3,accepted\n'
            '3,TRACKING,5,12,2,19,0.02,0.04,5,accepted\n'
            '4,DEGRADED,5,14,2,21,0.02,0.05,8,rejected\n')
        self.assertEqual(report['row_count'], 4)
        self.assertEqual(report['state_counts']['TRACKING'], 2)
        self.assertEqual(report['tracking_fraction'], 0.5)
        self.assertEqual(report['dropped_scans_last'], 8)
        self.assertEqual(report['first_logged_row']['metrics']['total_ms'], 130)
        self.assertEqual(report['steady_rows_excluding_first']['total_ms']['p50'], 19)
        self.assertEqual(report['all_rows']['result_age_s']['count'], 4)
        self.assertEqual(report['metric_units']['result_age_s'], 'seconds')

    def test_empty_csv_no_crash(self):
        report = self.summarize_text('')
        self.assertEqual(report['row_count'], 0)
        self.assertIsNone(report['tracking_fraction'])
        self.assertIsNone(report['first_logged_row'])
        self.assertIsNone(report['dropped_scans_last'])
        self.assertEqual(report['all_rows'], {})

    def test_nonfinite_values_excluded_and_missing_result_age_reported(self):
        report = self.summarize_text(
            'state,preprocess_ms,registration_ms,validation_ms,total_ms\n'
            'ERROR,nan,inf,,bad\nTRACKING,2,3,1,6\n')
        self.assertEqual(report['all_rows']['total_ms']['count'], 1)
        self.assertEqual(report['all_rows']['total_ms']['invalid_or_missing_count'], 1)
        self.assertEqual(report['all_rows']['total_ms']['p99'], 6)
        self.assertTrue(any('result_age_s unavailable' in warning for warning in report['warnings']))


if __name__ == '__main__':
    unittest.main()
