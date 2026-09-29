"""Read-only discovery and ordered playback of single/split SQLite ROS bags."""
import re
import sqlite3
import warnings
from pathlib import Path


def natural_key(path):
    return [int(s) if s.isdigit() else s for s in re.split(r'(\d+)', path.name)]


def discover_bag_files(path, allow_incomplete=False):
    path = Path(path)
    if path.is_file():
        files = [path]
    elif path.is_dir():
        metadata = path / 'metadata.yaml'
        names = []
        if metadata.exists():
            # rosbag2 metadata uses this explicit list, separate from files/path.
            text = metadata.read_text()
            match = re.search(r'^  relative_file_paths:\s*\n((?:    - [^\n]+\n?)+)', text, re.M)
            if match:
                names = [line.strip()[2:].strip().strip('"\'') for line in match[1].splitlines()]
        files = [path / n for n in names] if names else sorted(path.glob('*.db3'), key=natural_key)
        for file in files:
            if not file.resolve().is_relative_to(path.resolve()):
                raise ValueError('Metadata file path escapes bag directory')
        missing = [f for f in files if not f.is_file()]
        if missing:
            message = f'{path}: {len(missing)} missing bag parts; first: {missing[0].name}'
            if not allow_incomplete:
                raise ValueError(message + '; use --skip-unreadable only for an explicitly partial recording')
            warnings.warn(message)
        files = [f for f in files if f.is_file()]
    else:
        raise FileNotFoundError(path)
    usable = []
    for file in files:
        try:
            db = sqlite3.connect(file.resolve().as_uri() + '?mode=ro', uri=True)
            try:
                db.execute('SELECT count(*) FROM messages').fetchone()
            finally:
                db.close()
        except sqlite3.Error as error:
            if not allow_incomplete:
                raise ValueError(f'Unreadable bag {file}: {error}') from error
            warnings.warn(f'Skipping unreadable bag {file}: {error}')
            continue
        usable.append(file)
    if not usable:
        raise ValueError(f'No readable SQLite bags in {path}')
    return usable


def bag_clouds(paths, topic=None, start=0, end=None):
    if start < 0 or (end is not None and end < start):
        raise ValueError('Invalid frame interval')
    if isinstance(paths, (str, Path)):
        paths = [Path(paths)]
    offset = 0
    for path in paths:
        db = sqlite3.connect(Path(path).resolve().as_uri() + '?mode=ro', uri=True)
        try:
            topics = db.execute("SELECT id,name FROM topics WHERE type='sensor_msgs/msg/PointCloud2'").fetchall()
            selected = [(i, name) for i, name in topics if topic is None or name == topic]
            if len(selected) != 1:
                raise ValueError(f'Select --source-topic; PointCloud2 topics: {topics}')
            topic_id = selected[0][0]
            total = db.execute('SELECT count(*) FROM messages WHERE topic_id=?', (topic_id,)).fetchone()[0]
            if start >= offset + total:
                offset += total
                continue
            local_start = max(0, start - offset)
            local_end = total - 1 if end is None else min(total - 1, end - offset)
            if local_end < local_start:
                return
            # Sort identifiers, never multi-megabyte BLOBs: bags without an
            # appropriate index must not spool clouds to SQLite temp storage.
            rows = db.execute('SELECT id,length(data) FROM messages WHERE topic_id=? ORDER BY timestamp,id LIMIT ? OFFSET ?',
                              (topic_id, local_end - local_start + 1, local_start))
            for index, (row_id, size) in enumerate(rows, offset + local_start):
                if size is None or size > 64 * 1024 * 1024 + 65536:
                    raise ValueError(f'Oversized or NULL serialized cloud at frame {index}')
                data = db.execute('SELECT data FROM messages WHERE id=?', (row_id,)).fetchone()[0]
                # Keep CDR bytes intact: no Python per-point conversions.
                yield index, data, None
            offset += total
            if end is not None and offset > end:
                return
        finally:
            db.close()
