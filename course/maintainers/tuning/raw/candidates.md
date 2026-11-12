### Faster candidates, 3 seeds (use --seeds 0,1,2)

Seeds 0,1,2, 36 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| baseline | 36/36 | 30.0 | 41.85 | 33.78 | 9.9 |  |
| corner 1.28 + finish 0.6 (fast) | 18/36 | 28.5 | 41.47 | - | 8.0 | stopped short in DONE x18 |
| corner 1.15 + finish 0.45 (safer) | 20/36 | 28.7 | 42.23 | - | 8.8 | stopped short in DONE x16 |
| finish 0.45 only | 36/36 | 30.0 | 41.64 | 33.78 | 9.9 |  |
| fast + GAIN 0.6 | 18/36 | 28.5 | 41.39 | - | 8.1 | stopped short in DONE x18 |
| fast + steering 25 deg (car and strategy) | 9/36 | 14.8 | 42.68 | - | 0.0 | wall in FOLLOW x18; stopped short in DONE x9 |
