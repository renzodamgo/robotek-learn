### Gyroscope: drift after the calibration and scale error

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| gyro drift 2e-05 rad/s per s | 12/12 | 30.0 | 40.72 | 32.58 | 9.5 |  |
| gyro drift 5e-05 rad/s per s | 12/12 | 30.0 | 40.72 | 32.68 | 8.9 |  |
| gyro drift 0.0001 rad/s per s | 12/12 | 30.0 | 40.77 | 32.78 | 8.8 |  |
| gyro drift 0.0002 rad/s per s | 11/12 | 29.6 | 39.63 | 33.08 | 1.6 | stopped short in DONE x1 |
| gyro drift 0.0005 rad/s per s | 4/12 | 24.8 | 35.53 | 34.18 | 0.0 | wall in FOLLOW x4; wall in TURN x1; stopped short in DONE x3 |
| gyro scale 0.9 | 0/12 | 3.5 | - | - | 0.0 | wall in FOLLOW x12 |
| gyro scale 0.95 | 0/12 | 11.0 | - | - | 0.0 | wall in FINISH x1; wall in FOLLOW x9; wall in TURN x1; stopped short in DONE x1 |
| gyro scale 0.97 | 2/12 | 15.1 | 35.48 | 35.68 | 0.0 | wall in FOLLOW x8; wall in TURN x2 |
| gyro scale 0.98 | 1/12 | 20.0 | 33.18 | 33.18 | 0.0 | wall in FOLLOW x10; wall in TURN x1 |
| gyro scale 0.99 | 12/12 | 30.0 | 39.65 | 32.28 | 2.6 |  |
| gyro scale 1.01 | 12/12 | 30.0 | 41.50 | 33.78 | 5.7 |  |
| gyro scale 1.02 | 5/12 | 25.4 | 40.32 | 37.58 | 0.0 | wall in FOLLOW x3; wall in TURN x1; stopped short in DONE x3 |
| gyro scale 1.03 | 0/12 | 19.4 | - | - | 0.0 | wall in FOLLOW x10; stopped short in DONE x2 |
| gyro scale 1.05 | 0/12 | 11.8 | - | - | 0.0 | wall in FOLLOW x11; stopped short in DONE x1 |
