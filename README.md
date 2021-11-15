# Joint Probabilistic Data Association Tracking (JPDAFTracker)
JPDAFTracker is a tracker based on joint probabilistic data association filtering.

This is a ros wrapper for the algoritm above

<p align="center">
<a href="https://www.youtube.com/watch?v=KlXpaKh8hDY"  target="_blank"><img src="https://img.youtube.com/vi/KlXpaKh8hDY/0.jpg"/></a>
</p>
<br>

## Requirements
* OpenCV
* Eigen

## How to build

TBD

## Params
```bash
[PD] #DETECTION PROBABILITY
1

[PG] #GATE PROBABILITY
0.4

[LOCAL_GSIGMA] #THRESHOLD OF GATING
15

[LOCAL_ASSOCIATION_COST] #ASSOCIATION COSTS
40

[GLOBAL_GSIGMA] #THRESHOLD OF GATING
0.1

[GLOBAL_ASSOCIATION_COST] #ASSOCIATION COSTS
50

[LAMBDA] #CONSTANT
2

[GAMMA] #G1,G2 INITIAL COVARIANCE P MATRIX
10 10

[R_MATRIX] #2x2 MEASUREMENT NOISE COVARIANCE MATRIX
100 0
0 100

[DT] #dt
0.4

[MIN_ACCPETANCE_RATE] #min rate for convalidating a track
10

[MAX_MISSED_RATE] #max rate for deleting a track
9
```

## How to use

tbd