### Motor: slower response (drive_tau, drive_delay)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| drive_tau 0.05 s | 12/12 | 30.0 | 40.85 | 32.68 | 8.9 |  |
| drive_tau 0.15 s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| drive_tau 0.3 s | 12/12 | 30.0 | 40.76 | 32.68 | 8.4 |  |
| drive_tau 0.6 s | 0/12 | 27.0 | - | - | 10.1 | no finish in DONE x12 |
| drive_delay 0 s | 12/12 | 30.0 | 40.80 | 32.78 | 8.6 |  |
| drive_delay 0.06 s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| drive_delay 0.15 s | 12/12 | 30.0 | 40.79 | 32.78 | 8.8 |  |
| drive_delay 0.3 s | 12/12 | 30.0 | 40.68 | 32.48 | 8.6 |  |
| accel_max 0.3 m/s2 | 12/12 | 30.0 | 41.80 | 33.68 | 7.8 |  |
| accel_max 0.6 m/s2 | 12/12 | 30.0 | 41.06 | 32.98 | 8.4 |  |
| accel_max 2 m/s2 | 12/12 | 30.0 | 40.53 | 32.38 | 8.5 |  |
