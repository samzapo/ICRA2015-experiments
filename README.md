### IROS Experimental Repository (2015)
This repository is meant to store reproducable experimental setups for:

Title:    Adaptive Integration for Controlling Speed vs. Accuracy in Multi-Rigid Body Simulation
Authors:  Sam Zapolsky, Evan Drumwright
Inproc.:  The 2015 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2015), Hamburg, Germany, Sep 2015

## INSTALLATION:
#  This package requires: 
    Moby (simulator):         github.com/PositronicsLab/Moby 
                     branch:  stable-int
                    version:  1d930bdb1839304e692bb9b07d7228cade6bddfa

    Ravelin (Linear Algebra): github.com/PositronicsLab/Ravelin 
                      branch: master
                     version: 36abb0ce6fffc85860581378f3b7e39e5af14d92

#  Additional Requirements:
    Open Scene Graph (Visualization): http://www.openscenegraph.org/
      To visualize, run all examples with the additional option: -r, else remove this option

## To run a set of experiments: 
```
  ./run_all_test.sh <sample-directories>
```
  Experiment Directory | Sample Directories | description
  -------------- | ------------ | ----------------
  stepsize       |   | 
  adaptive       | |
  euler          | |
  rayleigh       | |
  baumgarte      | |
  exponential    | |
