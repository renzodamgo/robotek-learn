### Corner when the car turns FASTER than the strategy believes (car wheelbase 25 % shorter): DAMPING, TURN_TOLERANCE

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| car turns 33 % faster, defaults | 12/12 | 30.0 | 41.04 | 32.88 | 6.3 |  |
| faster, DAMPING 0 | 12/12 | 30.0 | 40.96 | 32.88 | 3.5 |  |
| faster, DAMPING 0.3 | 12/12 | 30.0 | 41.09 | 32.98 | 8.2 |  |
| faster, TURN_TOLERANCE 0.12 | 6/12 | 26.2 | 44.63 | - | 2.5 | stopped short in DONE x6 |
| faster, TURN_TOLERANCE 0.5 | 12/12 | 30.0 | 41.02 | 32.58 | 9.0 |  |
| faster, TURN_TOLERANCE 0.8 | 12/12 | 30.0 | 41.05 | 32.78 | 9.0 |  |
