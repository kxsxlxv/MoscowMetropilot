#!/usr/bin/env python3
"""Tiny SQLite/CDR smoke test for the native replay; no ROS or dataset needed."""
import csv
import sqlite3
import struct
import subprocess
import sys
import tempfile
from pathlib import Path


def origin_cloud():
    data = bytearray(b'\x00\x01\x00\x00')
    def scalar(fmt, value, alignment=4):
        data.extend(bytes((-(len(data) - 4)) % alignment))
        data.extend(struct.pack('<' + fmt, value))
    def string(value):
        encoded = value.encode() + b'\0'
        scalar('I', len(encoded))
        data.extend(encoded)
    scalar('i', 1000); scalar('I', 0); string('hesai_lidar')
    scalar('I', 1); scalar('I', 1); scalar('I', 3)
    for offset, name in enumerate(('x', 'y', 'z')):
        string(name); scalar('I', offset * 4); scalar('B', 7, 1); scalar('I', 1)
    scalar('B', 0, 1); scalar('I', 12); scalar('I', 12)
    scalar('I', 12); data.extend(bytes(12)); scalar('B', 1, 1)
    return bytes(data)


def main():
    binary = Path(sys.argv[1]).resolve()
    root = Path(__file__).resolve().parents[1]
    # Keep even the tiny fixture on the project disk, not Docker or system tmp.
    parent = root / 'build'
    parent.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(dir=parent) as folder:
        folder = Path(folder)
        database = folder/'tiny.db3'
        db = sqlite3.connect(database)
        try:
            db.executescript('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT);'
                'CREATE TABLE messages(id INTEGER PRIMARY KEY,timestamp INTEGER,topic_id INTEGER,data BLOB);')
            db.execute("INSERT INTO topics VALUES(1,'/lidar','sensor_msgs/msg/PointCloud2')")
            db.executemany('INSERT INTO messages VALUES(?,?,1,?)',
                           [(1,30,None), (2,20,b'bad'), (3,10,origin_cloud())])
            db.commit()
        finally:
            db.close()
        original = database.read_bytes()
        listing = folder/'list.txt'
        listing.write_text(str(database) + '\n')
        prefix = folder/'result'
        subprocess.run([str(binary), str(listing), str(prefix),
            str(root/'config/sensor_mount_challenge_assumed.yaml'),
            str(root/'config/vehicle_geometry.yaml')], check=True)
        with (folder/'result_frames.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        assert [r['row_id'] for r in rows] == ['3', '2', '1']
        assert all(r['flag'] == '0' for r in rows)
        assert not rows[0]['error'], rows[0]['error']
        assert rows[1]['error'] and 'NULL' in rows[2]['error']
        assert database.read_bytes() == original
        print('PASS: 3 tiny replay frames; timestamp order, valid CDR, malformed/NULL input, unchanged source')


if __name__ == '__main__':
    main()
