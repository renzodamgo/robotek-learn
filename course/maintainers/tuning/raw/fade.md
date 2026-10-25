### Corner when the car turns FASTER than the strategy believes (car wheelbase 25 % shorter): DAMPING, EXIT_ANGLE

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| car turns 33 % faster, defaults | 12/12 | 30.0 | 39.97 | 31.88 | 4.3 |  |
| faster, DAMPING 0 | 12/12 | 30.0 | 40.01 | 31.88 | 4.3 |  |
| faster, DAMPING 0.3 | 12/12 | 30.0 | 39.97 | 31.88 | 4.3 |  |
| faster, EXIT_ANGLE 0.02 | 12/12 | 30.0 | 39.97 | 31.88 | 4.3 |  |
| faster, EXIT_ANGLE 0.6 | 12/12 | 30.0 | 40.00 | 31.88 | 4.8 |  |
| faster, EXIT_ANGLE 0.9 | 12/12 | 30.0 | 40.01 | 31.88 | 6.1 |  |
| faster, DAMPING 0 + EXIT_ANGLE 0.02 | 12/12 | 30.0 | 40.01 | 31.88 | 4.3 |  |
