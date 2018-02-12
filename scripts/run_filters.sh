#!/bin/bash

UKF_BIN=UnscentedKF

if [ ! -e "../build/$UKF_BIN" ]; then
	if [ ! -e "../build" ]; then
		mkdir ../build
	fi
	cd ../build
	cmake ../ && make
fi
../build/$UKF_BIN --use_simulator=0  --use_laser=1 --use_radar=1 --output_file=../data/obj_pose-fused-output-all.txt
../build/$UKF_BIN --use_simulator=0  --use_laser=1 --use_radar=0 --output_file=../data/obj_pose-fused-output-laseronly.txt
../build/$UKF_BIN --use_simulator=0  --use_laser=0 --use_radar=1 --output_file=../data/obj_pose-fused-output-radaronly.txt
