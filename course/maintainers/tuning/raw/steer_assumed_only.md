### Maximum steering angle: strategy assumes it but the car stays at 0.307

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| strategy assumes 0.2 rad (11.5 deg) | 7/12 | 21.2 | 39.24 | 34.18 | 0.0 | wall in TURN x5 |
| strategy assumes 0.25 rad (14.3 deg) | 12/12 | 30.0 | 41.10 | 33.08 | 4.2 |  |
| strategy assumes 0.35 rad (20.1 deg) | 12/12 | 30.0 | 40.76 | 32.58 | 9.5 |  |
| strategy assumes 0.4 rad (22.9 deg) | 12/12 | 30.0 | 40.76 | 32.58 | 9.5 |  |
| strategy assumes 0.4363 rad (25.0 deg) | 12/12 | 30.0 | 40.76 | 32.58 | 9.5 |  |
