#!/bin/bash

cd build && cmake .. && make -j12

subdivLevel=4
lambdaG1=0.2
lambdaG2=0.05
camNum=2
eMcFile="/home/antoine/Documents/Github/ICRA_SI_DVS_MPP/servoing/spherical_visual_servoing/common/eMc_good.yaml"

./MPPSphericalVS $subdivLevel $lambdaG1 $lambdaG2 $camNum $eMcFile