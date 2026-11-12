### Maximum steering angle: strategy assumes it but the car stays at 0.307

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| strategy assumes 0.2 rad (11.5 deg) | 8/12 | 22.5 | 41.32 | 35.58 | 0.0 | wall in TURN x3; stopped short in DONE x1 |
| strategy assumes 0.25 rad (14.3 deg) | 12/12 | 30.0 | 42.43 | 34.38 | 6.1 |  |
| strategy assumes 0.35 rad (20.1 deg) | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| strategy assumes 0.4 rad (22.9 deg) | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| strategy assumes 0.4363 rad (25.0 deg) | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
