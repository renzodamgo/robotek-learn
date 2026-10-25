### Servo: slow servo (delay x3) with different fixes

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| delay x3, no change | 12/12 | 30.0 | 42.24 | 33.78 | 2.0 |  |
| delay x3, DAMPING 0 | 10/12 | 28.3 | 42.48 | 33.48 | 0.0 | wall in TURN x1 |
| delay x3, DAMPING 0.05 | 11/12 | 28.3 | 42.51 | 33.78 | 0.0 | wall in TURN x1 |
| delay x3, DAMPING 0.2 | 9/12 | 30.0 | 42.89 | 35.48 | 0.6 |  |
| delay x3, DAMPING 0.3 | 9/12 | 22.8 | 40.92 | 35.28 | 0.0 | wall in TURN x3 |
| delay x3, EXIT_ANGLE 0.1 | 12/12 | 30.0 | 42.24 | 33.78 | 2.0 |  |
| delay x3, EXIT_ANGLE 0.4 | 12/12 | 30.0 | 42.24 | 33.78 | 2.0 |  |
| delay x3, EXIT_ANGLE 0.6 | 12/12 | 30.0 | 42.24 | 33.78 | 2.0 |  |
| delay x3, TURN_FRONT 1.25 | 12/12 | 30.0 | 42.30 | 33.88 | 2.0 |  |
| delay x3, TURN_FRONT 1.3 | 12/12 | 30.0 | 42.31 | 33.78 | 2.0 |  |
| delay x3, TURN_FRONT 1.4 | 12/12 | 30.0 | 42.34 | 33.78 | 2.0 |  |
| delay x3, cruise 0.9 | 12/12 | 30.0 | 48.24 | 39.08 | 6.3 |  |
| delay x3, cruise 0.7 | 12/12 | 30.0 | 52.42 | 42.08 | 7.9 |  |
