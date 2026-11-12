### Motor: slower response (drive_tau, drive_delay)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| drive_tau 0.05 s | 12/12 | 30.0 | 41.85 | 33.78 | 9.0 |  |
| drive_tau 0.15 s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| drive_tau 0.3 s | 6/12 | 28.5 | 45.00 | - | 10.4 | stopped short in DONE x6 |
| drive_tau 0.6 s | 0/12 | 27.0 | - | - | 9.0 | stopped short in DONE x12 |
| drive_delay 0 s | 12/12 | 30.0 | 41.82 | 33.78 | 10.4 |  |
| drive_delay 0.06 s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| drive_delay 0.15 s | 10/12 | 29.5 | 42.62 | - | 9.5 | stopped short in DONE x2 |
| drive_delay 0.3 s | 7/12 | 28.8 | 43.85 | - | 10.6 | stopped short in DONE x5 |
| accel_max 0.3 m/s2 | 12/12 | 30.0 | 43.07 | 34.98 | 8.1 |  |
| accel_max 0.6 m/s2 | 12/12 | 30.0 | 42.21 | 33.98 | 10.4 |  |
| accel_max 2 m/s2 | 12/12 | 30.0 | 41.58 | 33.28 | 10.0 |  |
