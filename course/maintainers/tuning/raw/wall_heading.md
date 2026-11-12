### Gyroscope scale error with and without the island-wall heading correction (WALL_HEADING_GAIN)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| WALL_HEADING_GAIN 0 | 12/12 | 30.0 | 41.86 | 33.78 | 10.5 |  |
| WALL_HEADING_GAIN 0.1 | 12/12 | 30.0 | 41.90 | 33.78 | 10.8 |  |
| WALL_HEADING_GAIN 0.3 | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| WALL_HEADING_GAIN 0.6 | 12/12 | 30.0 | 41.85 | 33.78 | 9.4 |  |
| WALL_HEADING_GAIN 1 | 12/12 | 30.0 | 41.81 | 33.68 | 8.3 |  |
| gyro scale 0.9, gain 0 (off) | 0/12 | 3.3 | - | - | 0.0 | wall in FOLLOW x12 |
| gyro scale 0.9, gain 0.3 | 9/12 | 28.5 | 43.17 | - | 1.8 | stopped short in DONE x3 |
| gyro scale 0.94, gain 0 (off) | 0/12 | 6.2 | - | - | 0.0 | wall in FOLLOW x10; wall in TURN x2 |
| gyro scale 0.94, gain 0.3 | 10/12 | 29.5 | 43.46 | - | 9.2 | stopped short in DONE x2 |
| gyro scale 0.97, gain 0 (off) | 0/12 | 13.0 | - | - | 0.0 | wall in FOLLOW x6; wall in TURN x6 |
| gyro scale 0.97, gain 0.3 | 12/12 | 30.0 | 41.86 | 33.78 | 10.1 |  |
| gyro scale 1.03, gain 0 (off) | 2/12 | 20.8 | 35.38 | 35.48 | 0.0 | wall in TURN x4; wall in FOLLOW x6 |
| gyro scale 1.03, gain 0.3 | 12/12 | 30.0 | 41.86 | 33.78 | 10.1 |  |
| gyro scale 1.06, gain 0 (off) | 0/12 | 11.7 | - | - | 0.0 | wall in FOLLOW x10; wall in TURN x2 |
| gyro scale 1.06, gain 0.3 | 12/12 | 30.0 | 41.78 | 33.68 | 9.7 |  |
| gyro scale 1.1, gain 0 (off) | 0/12 | 8.0 | - | - | 0.0 | wall in FOLLOW x9; wall in TURN x3 |
| gyro scale 1.1, gain 0.3 | 12/12 | 30.0 | 41.91 | 33.48 | 9.3 |  |
| gyro drift 0.0002 rad/s per s, gain 0 (off) | 8/12 | 28.6 | 40.39 | 33.88 | 0.0 | wall in FOLLOW x1; stopped short in DONE x3 |
| gyro drift 0.0005 rad/s per s, gain 0 (off) | 4/12 | 24.3 | 38.81 | 34.58 | 0.0 | wall in FOLLOW x5; wall in FINISH x1; stopped short in DONE x2 |
