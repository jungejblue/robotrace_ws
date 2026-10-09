import importlib.util
from pathlib import Path
import struct
from types import SimpleNamespace
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'audit_bag.py'
SPEC = importlib.util.spec_from_file_location('audit_bag', SCRIPT)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def cloud(points, bigendian=False, row_padding=0, ring_type=4):
    prefix = '>' if bigendian else '<'
    fields = [SimpleNamespace(name=name, offset=offset, datatype=dtype, count=1)
              for name, offset, dtype in [('x', 0, 7), ('y', 4, 7), ('z', 8, 7),
                  ('intensity', 16, 7), ('t', 20, 6), ('ring', 26, ring_type)]]
    payload = bytearray()
    for x, y, z, t, ring in points:
        item = bytearray(48)
        struct.pack_into(prefix + 'fff', item, 0, x, y, z)
        struct.pack_into(prefix + 'fI', item, 16, 10.0, t)
        struct.pack_into(prefix + 'H', item, 26, ring)
        payload.extend(item)
    payload.extend(bytes(row_padding))
    return SimpleNamespace(fields=fields, width=len(points), height=1,
                           point_step=48, row_step=48*len(points)+row_padding,
                           is_bigendian=bigendian, data=bytes(payload))


class InputAuditTests(unittest.TestCase):
    def test_zero_nan_and_blind_filter_have_distinct_counts(self):
        msg = cloud([(0, 0, 0, 0, 0), (float('nan'), 1, 2, 1, 1),
                     (5, 0, 0, 49000000, 63)])
        report = module.inspect_cloud(msg, stride=1)
        self.assertEqual(report['raw_points'], 3)
        self.assertEqual(report['finite_xyz_points'], 2)
        self.assertEqual(report['nonfinite_xyz_points'], 1)
        self.assertEqual(report['preprocess_candidate_points'], 1)
        self.assertEqual(report['selected_point_time_max_ms'], 49.0)
        self.assertFalse(report['field_type_mismatches'])

    def test_uint8_ring_is_reported_as_incompatible(self):
        report = module.inspect_cloud(cloud([(5, 0, 0, 1, 1)], ring_type=2), stride=1)
        self.assertEqual(report['field_type_mismatches']['ring'], {'expected': 4, 'observed': 2})

    def test_bigendian_and_row_padding_preserve_values(self):
        msg = cloud([(6, 0, 0, 50000000, 32)], bigendian=True, row_padding=24)
        report = module.inspect_cloud(msg, stride=1)
        self.assertEqual(report['range_max_m'], 6.0)
        self.assertEqual(report['point_time_max_ms'], 50.0)
        self.assertFalse(report['frontend_schema_compatible'])

    def test_ring_time_and_blind_boundary_match_frontend_filters(self):
        msg = cloud([(4, 0, 0, 1, 0), (5, 0, 0, 100000001, 0),
                     (6, 0, 0, 50000000, 64), (7, 0, 0, 100000000, 63)])
        report = module.inspect_cloud(msg, stride=1)
        self.assertEqual(report['preprocess_candidate_points'], 1)
        self.assertEqual(report['invalid_ring_points'], 1)
        self.assertEqual(report['invalid_relative_time_points'], 1)
        self.assertEqual(report['selected_point_time_max_ms'], 100.0)

    def test_truncated_payload_is_rejected(self):
        msg = cloud([(5, 0, 0, 10, 0)])
        msg.data = msg.data[:-1]
        with self.assertRaisesRegex(ValueError, 'truncated'):
            module.inspect_cloud(msg)

    def test_duplicate_required_field_is_rejected(self):
        msg = cloud([(5, 0, 0, 10, 0)])
        msg.fields.append(msg.fields[0])
        with self.assertRaisesRegex(ValueError, 'duplicate'):
            module.inspect_cloud(msg)

    def test_timestamp_resets_are_detected(self):
        report = module.timing_summary([(1.0, 2.0), (1.1, 2.1), (1.2, 1.0)])
        self.assertEqual(report['backward_headers'], 1)


if __name__ == '__main__':
    unittest.main()
