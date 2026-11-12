### Servo: slow servo (delay x3) with different fixes

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| delay x3, no change | 12/12 | 30.0 | 43.40 | 35.38 | 2.9 |  |
| delay x3, DAMPING 0 | 12/12 | 30.0 | 43.20 | 34.22 | 2.0 |  |
| delay x3, DAMPING 0.05 | 12/12 | 30.0 | 43.15 | 34.62 | 2.4 |  |
| delay x3, DAMPING 0.2 | 12/12 | 30.0 | 43.02 | 34.98 | 3.7 |  |
| delay x3, DAMPING 0.3 | 12/12 | 30.0 | 43.19 | 34.98 | 3.3 |  |
| delay x3, TURN_TOLERANCE 0.2 | 12/12 | 30.0 | 43.37 | 34.88 | 3.8 |  |
| delay x3, TURN_TOLERANCE 0.4 | 12/12 | 30.0 | 43.29 | 35.28 | 3.8 |  |
| delay x3, TURN_TOLERANCE 0.5 | 12/12 | 30.0 | 43.26 | 35.08 | 3.8 |  |
| delay x3, corner speed 0.8 | 12/12 | 30.0 | 46.40 | 37.68 | 6.1 |  |
| delay x3, corner speed 0.6 | 12/12 | 30.0 | 50.32 | 41.18 | 10.7 |  |
| delay x3, TURN_FRONT 1.25 | 12/12 | 30.0 | 43.47 | 34.84 | 2.3 |  |
| delay x3, TURN_FRONT 1.3 | 12/12 | 30.0 | 43.69 | 35.08 | 3.4 |  |
| delay x3, TURN_FRONT 1.4 | 12/12 | 30.0 | 43.89 | 35.78 | 3.8 |  |
| delay x3, cruise 0.9 | 12/12 | 30.0 | 46.13 | 37.18 | 3.9 |  |
| delay x3, cruise 0.7 | 12/12 | 30.0 | 48.38 | 38.28 | 8.1 |  |
