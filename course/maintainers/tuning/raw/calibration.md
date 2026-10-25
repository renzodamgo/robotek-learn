### Calibration time (CALIBRATE_SECONDS) with different gyro biases

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| calibrate 0.05 s | 12/12 | 30.0 | 40.73 | 32.88 | 8.3 |  |
| calibrate 0.1 s | 12/12 | 30.0 | 40.74 | 32.68 | 9.4 |  |
| calibrate 0.3 s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| calibrate 1 s | 12/12 | 30.0 | 41.52 | 33.18 | 7.4 |  |
| calibrate 2 s | 12/12 | 30.0 | 42.48 | 34.28 | 7.9 |  |
| calibrate 0.05 s, gyro noise 0.01 | 2/12 | 19.8 | 35.38 | - | 0.0 | wall in FOLLOW x8; wall in TURN x1; stopped short in DONE x1 |
| calibrate 0.3 s, gyro noise 0.01 | 12/12 | 30.0 | 40.76 | 32.68 | 9.0 |  |
