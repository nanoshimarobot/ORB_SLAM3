#!/bin/bash
pathDatasetTUM_VI='/dataset' #Example, it is necesary to change it by the dataset path

echo "Launching Corridor 1 with Stereo-Inertial sensor"
./Examples_old/RGB-D-Inertial/tum_rgbd_imu Vocabulary/ORBvoc.txt Examples/RGB-D-Inertial/aist_azurekinect.yaml "$pathDatasetTUM_VI"/hw1_2_w_imu
