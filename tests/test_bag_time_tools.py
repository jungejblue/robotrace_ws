import importlib.util
import copy
import io
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'prepare_validation_bag.py'
SPEC = importlib.util.spec_from_file_location('prepare_validation_bag', SCRIPT)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


class BagTimeTests(unittest.TestCase):
    def stats(self, epoch=False, drift=0.0):
        base = 1779438003 * module.NS
        result = {topic: module.TimingStats(10) for topic in module.TIMING_TOPICS}
        for index in range(30):
            elapsed = index * 100000000
            receipt = base + elapsed
            for topic in (module.POINTS, module.IMU):
                stamp = (base if epoch else 6883 * module.NS) + elapsed
                result[topic].add(receipt + int(drift * module.NS) * (index >= 20), stamp)
            for topic in module.GNSS_TOPICS:
                result[topic].add(receipt, receipt - 20000000)
        return result

    def test_same_offset_preserves_exact_intervals_and_gnss_delay_anchor(self):
        offset, _, _, _ = module.choose_offset(self.stats(), 0.08, 0.08)
        expected = (1779438003 - 6883) * module.NS - 20000000
        self.assertEqual(offset, expected)
        original = [6883000000123, 6883050000456, 6883100000789]
        shifted = [stamp + offset for stamp in original]
        self.assertEqual([b-a for a, b in zip(original, original[1:])],
                         [b-a for a, b in zip(shifted, shifted[1:])])

    def test_epoch_already_normalized_is_not_shifted_again(self):
        self.assertEqual(module.choose_offset(self.stats(epoch=True), 0.08, 0.08)[0], 0)

    def test_clock_reset_rejected(self):
        stats = self.stats()
        stats[module.IMU].add(1779439000 * module.NS, 6800 * module.NS)
        with self.assertRaisesRegex(ValueError, 'backward'):
            module.choose_offset(stats, 0.08, 0.08)

    def test_start_end_drift_rejected(self):
        with self.assertRaisesRegex(ValueError, 'changed'):
            module.choose_offset(self.stats(drift=0.1), 0.08, 0.08)

    def test_independent_imu_lidar_clock_rejected(self):
        stats = self.stats()
        stats[module.POINTS].first_offsets = [value + 0.2 for value in stats[module.POINTS].first_offsets]
        stats[module.POINTS].last_offsets = module.deque(stats[module.POINTS].first_offsets, maxlen=10)
        with self.assertRaisesRegex(ValueError, 'may not share a clock'):
            module.choose_offset(stats, 0.08, 0.08)

    def test_signed_nanosecond_utc_subsecond(self):
        msg = SimpleNamespace(valid=7, year=2026, month=5, day=22,
                              hour=8, min=20, sec=4, nano=-100000000)
        expected = 1779438004 * module.NS - 100000000
        self.assertEqual(module.navpvt_utc_ns(msg), expected)
        msg.valid = 3
        with self.assertRaisesRegex(ValueError, 'fully resolved'):
            module.navpvt_utc_ns(msg)

    def test_static_zero_and_epoch_stamps_preserved(self):
        first, last = 6883 * module.NS, 7000 * module.NS
        offset = 1779431120 * module.NS
        self.assertFalse(module.should_shift_static_stamp(0, first, last, offset))
        self.assertFalse(module.should_shift_static_stamp(1779438003 * module.NS, first, last, offset))
        self.assertFalse(module.should_shift_static_stamp(5 * module.NS, first, last, offset))
        self.assertTrue(module.should_shift_static_stamp(6882 * module.NS, first, last, offset))

    def test_filtered_output_preserves_points_gnss_record_time_and_latching(self):
        class FakeTime:
            def __init__(self, secs, nsecs=0):
                self.nanoseconds = int(secs) * module.NS + int(nsecs)

            def to_nsec(self):
                return self.nanoseconds

        rows = []
        for index in range(20):
            nano = (index % 10) * 100000000
            sec = 4 + index // 10
            receipt = FakeTime(1779438004 + index // 10, nano)
            for topic in (module.POINTS, module.IMU):
                msg = SimpleNamespace(header=SimpleNamespace(stamp=FakeTime(6883 + index // 10, nano)),
                                      data=b'unchanged-cloud-and-relative-point-t',
                                      fields=['x', 'y', 'z', 't'], point_step=48)
                rows.append((topic, msg, receipt, {'type': topic}))
            for topic in module.GNSS_TOPICS:
                msg = SimpleNamespace(valid=7, year=2026, month=5, day=22, hour=8,
                                      min=20, sec=sec, nano=nano - 20000000,
                                      iTOW=123456 + index * 100, flags=131)
                rows.append((topic, msg, receipt, {'type': topic}))
        static = SimpleNamespace(transforms=[SimpleNamespace(header=SimpleNamespace(stamp=FakeTime(0)))])
        rows.append(('/tf_static', static, FakeTime(1779438004), {'latching': '1'}))
        rows.append(('/unwanted/image', SimpleNamespace(data=b'image'), FakeTime(1779438004), {}))
        written = []

        class FakeBag:
            def __init__(self, path, mode):
                self.path, self.mode = path, mode
                if mode == 'w':
                    Path(path).write_bytes(b'fake-output-bag')

            def __enter__(self):
                return self

            def __exit__(self, *args):
                return False

            def read_messages(self, topics, return_connection_header=False):
                for row in rows:
                    if row[0] in topics:
                        yield copy.deepcopy(row if return_connection_header else row[:3])

            def write(self, topic, msg, receipt, connection_header):
                written.append(copy.deepcopy((topic, msg, receipt, connection_header)))

        fake_rosbag = SimpleNamespace(Bag=FakeBag, bag=SimpleNamespace(ROSBagException=RuntimeError))
        fake_rospy = SimpleNamespace(Time=FakeTime)
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'original.bag'
            output = Path(directory) / 'normalized.bag'
            source.write_bytes(b'original-never-modified')
            with patch.dict('sys.modules', {'rosbag': fake_rosbag, 'rospy': fake_rospy}), \
                    patch('sys.stdout', new_callable=io.StringIO):
                module.main([str(source), str(output), '--sample-window', '10'])
            self.assertEqual(source.read_bytes(), b'original-never-modified')
            self.assertTrue(output.is_file())
            self.assertTrue(output.with_suffix('.time_alignment.json').is_file())
        self.assertEqual(len(written), 81)
        self.assertNotIn('/unwanted/image', [row[0] for row in written])
        expected_offset = (1779438004 - 6883) * module.NS - 20000000
        for original, normalized in zip(rows[:80], written[:80]):
            topic, before, receipt, connection = original
            after = normalized[1]
            self.assertEqual(normalized[2].to_nsec(), receipt.to_nsec())
            self.assertEqual(normalized[3], connection)
            if topic in (module.POINTS, module.IMU):
                self.assertEqual(after.data, before.data)
                self.assertEqual(after.fields, before.fields)
                self.assertEqual(after.point_step, before.point_step)
                self.assertEqual(after.header.stamp.to_nsec(), before.header.stamp.to_nsec() + expected_offset)
            else:
                self.assertEqual(vars(after), vars(before))
        self.assertEqual(written[-1][3]['latching'], '1')
        self.assertEqual(written[-1][1].transforms[0].header.stamp.to_nsec(), 0)


if __name__ == '__main__':
    unittest.main()
