#!/usr/bin/env python3
"""Regression tests for split-bag ordering, cropping and partial recordings."""
import sqlite3
import sys
import tempfile
import unittest
from contextlib import closing
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2/metropilot_ros/scripts'))
from bag_sequence import bag_clouds, discover_bag_files


class SequenceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def bag(self, name, values):
        with closing(sqlite3.connect(self.root / name)) as db, db:
            db.executescript('CREATE TABLE topics(id INTEGER,name TEXT,type TEXT);'
                             'CREATE TABLE messages(id INTEGER,timestamp INTEGER,topic_id INTEGER,data BLOB);')
            db.execute('INSERT INTO topics VALUES (1,?,?)', ('/lidar_points','sensor_msgs/msg/PointCloud2'))
            db.executemany('INSERT INTO messages VALUES (?,?,1,?)',
                           [(i,i,bytes([n])) for i,n in enumerate(values)])

    def test_natural_order_and_global_crop(self):
        self.bag('bag_10.db3', [4,5]); self.bag('bag_2.db3', [2,3]); self.bag('bag_0.db3', [0,1])
        files = discover_bag_files(self.root)
        self.assertEqual([p.name for p in files], ['bag_0.db3','bag_2.db3','bag_10.db3'])
        rows = list(bag_clouds(files, start=1, end=4))
        self.assertEqual([(i,b[0]) for i,b,_ in rows], [(1,1),(2,2),(3,3),(4,4)])

    def test_metadata_order_missing_and_corrupt(self):
        self.bag('bag_0.db3', [1]); self.bag('bag_2.db3', [2])
        (self.root/'bag_3.db3').write_bytes(b'not a SQLite database')
        (self.root/'metadata.yaml').write_text('rosbag2_bagfile_information:\n  relative_file_paths:\n'
            '    - bag_2.db3\n    - bag_0.db3\n    - bag_1.db3\n    - bag_3.db3\n  files:\n')
        with self.assertRaises(ValueError): discover_bag_files(self.root)
        with self.assertWarns(UserWarning): files = discover_bag_files(self.root, True)
        self.assertEqual([p.name for p in files], ['bag_2.db3','bag_0.db3'])
        self.assertEqual([b[0] for _,b,_ in bag_clouds(files)], [2,1])

    def test_missing_topic_and_empty_selection(self):
        self.bag('bag_0.db3', [1,2])
        files=discover_bag_files(self.root)
        with self.assertRaises(ValueError): list(bag_clouds(files, topic='/wrong'))
        self.assertEqual(list(bag_clouds(files, start=2)), [])

    def test_timestamp_order_and_source_is_unchanged(self):
        self.bag('bag.db3', [10,20,30])
        path = self.root/'bag.db3'
        with closing(sqlite3.connect(path)) as db, db:
            db.execute('UPDATE messages SET timestamp=2-id')
        before = path.read_bytes()
        self.assertEqual([b[0] for _,b,_ in bag_clouds([path])], [30,20,10])
        self.assertEqual(path.read_bytes(), before)
        self.assertEqual(sorted(p.name for p in self.root.iterdir()), ['bag.db3'])

    def test_null_payload_and_invalid_range(self):
        self.bag('bag.db3', [10])
        path = self.root/'bag.db3'
        with closing(sqlite3.connect(path)) as db, db:
            db.execute('UPDATE messages SET data=NULL')
        with self.assertRaisesRegex(ValueError, 'NULL'):
            list(bag_clouds([path]))
        with self.assertRaisesRegex(ValueError, 'interval'):
            list(bag_clouds([path], start=-1))

    def test_metadata_cannot_escape_directory(self):
        (self.root/'metadata.yaml').write_text('rosbag2_bagfile_information:\n'
            '  relative_file_paths:\n    - ../outside.db3\n')
        with self.assertRaisesRegex(ValueError, 'escapes'):
            discover_bag_files(self.root, True)


if __name__ == '__main__':
    unittest.main()
