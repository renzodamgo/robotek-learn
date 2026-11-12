### Gyroscope: drift after the calibration and scale error

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| gyro drift 2e-05 rad/s per s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| gyro drift 5e-05 rad/s per s | 12/12 | 30.0 | 41.81 | 33.78 | 10.1 |  |
| gyro drift 0.0001 rad/s per s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| gyro drift 0.0002 rad/s per s | 12/12 | 30.0 | 41.76 | 33.78 | 10.1 |  |
| gyro drift 0.0005 rad/s per s | 12/12 | 30.0 | 41.86 | 33.88 | 10.1 |  |
| gyro scale 0.9 | 9/12 | 28.5 | 43.17 | - | 1.8 | stopped short in DONE x3 |
| gyro scale 0.95 | 11/12 | 29.8 | 41.88 | 33.88 | 9.8 | stopped short in DONE x1 |
| gyro scale 0.97 | 12/12 | 30.0 | 41.86 | 33.78 | 10.1 |  |
| gyro scale 0.98 | 12/12 | 30.0 | 41.87 | 33.78 | 10.1 |  |
| gyro scale 0.99 | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| gyro scale 1.01 | 12/12 | 30.0 | 41.90 | 33.78 | 10.1 |  |
| gyro scale 1.02 | 12/12 | 30.0 | 41.87 | 33.78 | 10.1 |  |
| gyro scale 1.03 | 12/12 | 30.0 | 41.86 | 33.78 | 10.1 |  |
| gyro scale 1.05 | 12/12 | 30.0 | 41.83 | 33.58 | 9.3 |  |
