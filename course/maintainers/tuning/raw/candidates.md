### Faster candidates, 3 seeds (use --seeds 0,1,2)

Seeds 0,1,2, 36 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| baseline | 36/36 | 30.0 | 40.77 | 32.58 | 8.2 |  |
| last corner 1.1 + finish 0.6 (search result) | 36/36 | 30.0 | 38.93 | 31.58 | 8.2 |  |
| last corner 0.9 + finish 0.45 (safer) | 36/36 | 30.0 | 39.34 | 31.78 | 8.2 |  |
| search result + GAIN 0.6 | 35/36 | 29.9 | 38.87 | 31.48 | 9.1 | stopped short in DONE x1 |
| search result + steering 25 deg (car and strategy) | 36/36 | 30.0 | 38.03 | 30.88 | 6.7 |  |
| last corner 1.28 (as cruise) + finish 0.6 | 18/36 | 28.5 | 41.49 | - | 8.2 | stopped short in DONE x18 |
