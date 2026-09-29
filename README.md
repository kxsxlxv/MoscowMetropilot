# MoscowMetropilot

Minimal final runtime package for the LiDAR-only obstacle detector.

## Runtime interface

- ROS 2 Humble / Ubuntu 22.04.
- Input: `/lidar_points` — `sensor_msgs/msg/PointCloud2`.
- Output: `/obstacle_detected` — `std_msgs/msg/Bool`.
- `true` means a candidate was confirmed by the conservative temporal gate.
- `false` means "not confirmed"; it also covers insufficient/invalid geometry or input and must not be interpreted as an independent proof of free track.

The included calibration profile is the challenge-data assumption for `hesai_lidar`. For another sensor/frame, replace it with measured extrinsics before interpreting physical results.

## Quick start without a dataset

Requirements: Docker Engine with Compose v2 and Linux containers.

```bash
mkdir -p data artifacts
docker compose build
docker compose up -d detector
docker compose run --rm debug
```

The default synthetic sequence is 30 frames: empty -> obstacle -> empty.
Expected summary:

```text
"sent": 30
"responses": 30
"true": 8
```

Stop:

```bash
docker compose down
```

## End-to-end ROS/DDS validation

With the detector running:

```bash
docker compose exec detector /entrypoint.sh python3 /ws/tools/test_ros_integration.py
```

The test exercises XYZ/XYZI/enriched PointCloud2, confirmation/reset behavior, stale input, below-rail rejection, low obstacles, split-bag playback, a large cloud, malformed input, unknown frame IDs and timestamp discontinuities.

## Native validation without ROS

Ubuntu dependencies:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libsqlite3-dev python3 python3-numpy
```

Run:

```bash
cmake -S cpp -B cpp/build -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build --parallel
ctest --test-dir cpp/build --output-on-failure

python3 tools/test_bag_sequence.py
python3 tools/test_precision_replay.py cpp/build/metropilot_precision_replay
```

## Optional bag playback

No dataset is included in this repository. Put a ROS 2 SQLite bag or split-bag directory under `./data`; it is mounted read-only as `/data`.

Example:

```bash
docker compose run --rm debug \
  ros2 run metropilot_ros lidar_debug \
  --db /data/my_bag \
  --start 0 --end 100 \
  --report /artifacts/result.csv
```

For deliberately slow visual playback you may temporarily start the detector with a larger wall-clock stale timeout; keep the value a floating-point literal:

```bash
LIDAR_STALE_TIMEOUT_S=5.0 docker compose up -d --force-recreate detector
```

Restore the normal value afterwards:

```bash
docker compose down
docker compose up -d detector
```

## Source layout

- `cpp/` — ROS-independent detection core and minimal native tests.
- `ros2/metropilot_ros/` — ROS 2 detector and debug publisher.
- `config/` — challenge calibration assumption and vehicle geometry.
- `tools/` — dataset-free smoke/integration tests.
- `Dockerfile`, `compose.yaml` — reproducible ROS 2 Humble runtime.
