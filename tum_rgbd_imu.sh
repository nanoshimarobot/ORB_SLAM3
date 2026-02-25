#!/bin/bash
pathDatasetTUM_VI='/dataset' #Example, it is necesary to change it by the dataset path

echo "Launching Corridor 1 with Stereo-Inertial sensor"
./Examples_old/RGB-D-Inertial/tum_rgbd_imu Vocabulary/ORBvoc.txt Examples/RGB-D-Inertial/aist_azurekinect_fixed.yaml "$pathDatasetTUM_VI"/hw1_3_w_imu
# ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM3.yaml "$pathDatasetTUM_VI"/hw1_3_w_imu "$pathDatasetTUM_VI"/hw1_3_w_imu/association.txt
