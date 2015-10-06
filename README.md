# IROS Experimental Repository (2015)
This repository is meant to store reproducable experimental setups for:

###Title:    
`Adaptive Integration for Controlling Speed vs. Accuracy in Multi-Rigid Body Simulation`
###Authors:  
`Sam Zapolsky, Evan Drumwright`
###Inproc.:  
`The 2015 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2015), Hamburg, Germany, Sep 2015`

## INSTALLATION:
###  This package requires: 
Moby (simulator)        : `github.com/PositronicsLab/Moby`
                  branch: `stable-int`
                 version: `1d930bdb1839304e692bb9b07d7228cade6bddfa`

Ravelin (Linear Algebra): `github.com/PositronicsLab/Ravelin`
                  branch: `master`
                 version: `36abb0ce6fffc85860581378f3b7e39e5af14d92`


Pacer (Robot control)   : `github.com/PositronicsLab/Pacer`
                  branch: `IROS2015`
                 version: *most recent*
###  Additional Requirements:
Open Scene Graph (Visualization): http://www.openscenegraph.org/
  
To visualize, run all examples with the additional option: -r, else remove this option

## To run a set of experiments: 
```
  ./run_all_test.sh <sample-directories>
```
  Experiment Directory | Sample Directories | description
  -------------- | ------------ | ----------------
  `stepsize`       | step size: `0.001 0.0015  0.00175 0.002 0.003 0.004 0.005 0.006 0.007 0.008 0.009 0.01` seconds | walking robot with fixed step size, recording max numer of steps reached before instability
  `adaptive`       | `stand walk` | comparing adaptive integration effects on step size for walking vs standing on a quadruped
  `euler`          | `ex`(explicit Euler)  `si`(semi-implicit Euler) | compare energy conserxing properties of a simplectic vs non-simplectic integrator
  `rayleigh`       | dissipation factor 1 x 10^e, e=`0 1 10 6 7 8 9` | *NOT USED IN PAPER* , a kind of dissipator, doesn't work well 
  `baumgarte`      | Baumgarte stabiliaztion factor alpha: alpha=`0 1 10 100 1000 10000 100000` | *NOT USED IN PAPER*, check effects of baumgart stabilization on stability (local energy error value)
  `exponential`    | exponential dissipation factor lambda: lambda = 1 - 0.1x`10` (0.)`0` (0.)`1` (0.)`2` (0.)`3` (0.)`4` (0.)`5` (0.)`6` (0.)`7` (0.)`8` (0.)`9` (0.)`99` | check effects of exponential dissipation on max number of     steps taken using a normally unstable 5ms step size
