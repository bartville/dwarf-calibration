# Dwarf-Calibration Tool #

![](http://www.syncrpg.com/sam/data/tokenImages/Devin_Night/allfreezippedpacks/dwarf_04.png)

Ultra-minimalistic tool for 3d calibration.

## Prerequisites

* Eigen3

## Compilation and test

To compile the tool simply use the standard CMake modality
~~~~
mkdir build
cd build 
cmake ..
make
~~~~

To test the dwarf solver is not drunk, run the example app

~~~~
cd bin
./dwarf-calibration-app
~~~~
A dwarf factory will generate a fake dataset and, starting from an Identity initial guess, a dwarf solver will find the correct transform.
