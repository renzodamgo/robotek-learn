# robotek_common

Library (no nodes) shared by your strategy (`my_open`) and the simulation tools:

- `vehicle.py` — car geometry and sensor mounting (must match `robotek_description`).
- `lidar_odometry.py` — travelled distance from consecutive LiDAR scans (no encoder).

It must not depend on simulation or evaluation packages.
