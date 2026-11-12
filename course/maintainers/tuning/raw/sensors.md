### Sensors: LiDAR dropout and range noise, gyroscope bias and noise

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| LiDAR dropout 0% | 12/12 | 30.0 | 41.85 | 33.68 | 10.2 |  |
| LiDAR dropout 4% | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| LiDAR dropout 8% | 12/12 | 30.0 | 41.77 | 33.78 | 10.1 |  |
| LiDAR dropout 12% | 12/12 | 30.0 | 41.76 | 33.78 | 10.0 |  |
| LiDAR dropout 15% | 12/12 | 30.0 | 41.80 | 33.68 | 9.9 |  |
| LiDAR dropout 25% | 12/12 | 30.0 | 41.82 | 33.78 | 9.9 |  |
| LiDAR dropout 40% | 12/12 | 30.0 | 41.79 | 33.68 | 9.6 |  |
| LiDAR range noise 0 mm | 12/12 | 30.0 | 41.85 | 33.78 | 10.6 |  |
| LiDAR range noise 8 mm | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| LiDAR range noise 20 mm | 10/12 | 27.7 | 41.95 | 33.38 | 0.0 | wall in TURN x1; stopped short in DONE x1 |
| LiDAR range noise 40 mm | 0/12 | 5.3 | - | - | 0.0 | wall in TURN x7; wall in FOLLOW x3; stopped short in DONE x2 |
| LiDAR range noise 80 mm | 0/12 | 2.2 | - | - | 0.0 | wall in FOLLOW x7; wall in TURN x4; stopped short in DONE x1 |
| gyro bias 0 rad/s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| gyro bias 0.006 rad/s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| gyro bias 0.02 rad/s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| gyro bias 0.05 rad/s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| gyro bias 0.1 rad/s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| gyro noise 0 rad/s | 12/12 | 30.0 | 41.85 | 33.78 | 10.1 |  |
| gyro noise 0.0019 rad/s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| gyro noise 0.01 rad/s | 12/12 | 30.0 | 41.80 | 33.78 | 10.1 |  |
| gyro noise 0.03 rad/s | 12/12 | 30.0 | 41.82 | 33.78 | 10.0 |  |
| gyro noise 0.1 rad/s | 12/12 | 30.0 | 41.82 | 33.78 | 9.2 |  |
