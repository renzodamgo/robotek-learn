### Servo speed limit (steer_rate; x1 = 1.2 rad/s = 69 deg/s)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| rate 4.4 rad/s | 12/12 | 30.0 | 40.90 | 32.78 | 9.5 |  |
| rate 2.4 rad/s | 12/12 | 30.0 | 41.15 | 32.88 | 10.1 |  |
| rate 1.2 rad/s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| rate 0.9 rad/s | 12/12 | 30.0 | 42.32 | 34.38 | 7.8 |  |
| rate 0.6 rad/s | 12/12 | 30.0 | 43.38 | 35.46 | 2.9 |  |
| rate 0.4 rad/s | 5/12 | 14.7 | 42.33 | 40.54 | 0.0 | wall in TURN x7 |
| rate 0.3 rad/s | 0/12 | 2.2 | - | - | 0.0 | wall in FOLLOW x6; wall in TURN x6 |
| rate 0.2 rad/s | 0/12 | 1.5 | - | - | 0.0 | wall in FOLLOW x6; wall in TURN x6 |
