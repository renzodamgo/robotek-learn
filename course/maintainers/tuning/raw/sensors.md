### Sensors: LiDAR dropout and range noise, gyroscope bias and noise

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| LiDAR dropout 0% | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| LiDAR dropout 4% | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| LiDAR dropout 8% | 12/12 | 30.0 | 40.73 | 32.58 | 9.1 |  |
| LiDAR dropout 12% | 12/12 | 30.0 | 40.67 | 32.58 | 9.1 |  |
| LiDAR dropout 15% | 12/12 | 30.0 | 40.71 | 32.58 | 7.8 |  |
| LiDAR dropout 25% | 2/12 | 13.2 | 32.23 | 32.38 | 0.0 | wall in TURN x10 |
| LiDAR dropout 40% | 2/12 | 7.0 | 31.53 | 31.98 | 0.0 | wall in TURN x7; wall in FOLLOW x3 |
| LiDAR range noise 0 mm | 12/12 | 30.0 | 40.78 | 32.58 | 9.4 |  |
| LiDAR range noise 8 mm | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| LiDAR range noise 20 mm | 10/12 | 25.8 | 40.72 | 32.58 | 0.0 | wall in TURN x2 |
| LiDAR range noise 40 mm | 1/12 | 6.2 | 30.68 | - | 0.0 | wall in TURN x7; wall in FOLLOW x3; stopped short in DONE x1 |
| LiDAR range noise 80 mm | 0/12 | 3.5 | - | - | 0.0 | wall in TURN x8; wall in FOLLOW x3; stopped short in DONE x1 |
| gyro bias 0 rad/s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| gyro bias 0.006 rad/s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| gyro bias 0.02 rad/s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| gyro bias 0.05 rad/s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| gyro bias 0.1 rad/s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| gyro noise 0 rad/s | 12/12 | 30.0 | 40.73 | 32.48 | 9.3 |  |
| gyro noise 0.0019 rad/s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| gyro noise 0.01 rad/s | 12/12 | 30.0 | 40.76 | 32.68 | 9.0 |  |
| gyro noise 0.03 rad/s | 12/12 | 30.0 | 40.78 | 33.08 | 8.7 |  |
| gyro noise 0.1 rad/s | 11/12 | 29.1 | 40.64 | 33.98 | 0.0 | wall in TURN x1 |
