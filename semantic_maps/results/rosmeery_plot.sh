#!/bin/bash

white='\033[1;37m'
yellow='\033[1;33m'
orange='\033[0;33m'
nc='\033[0m'

if [ $# -ne 1 ]; then
    echo -ne "Wrong number of parameters. Only the root results (.json) path must be provided.\n"
    exit
fi

RESULTS_LOAD_PATH=$1 # /home/rfr4n/ws/qut/results
PLOTS_SAVE_PATH=/tmp/rosmeery/plots/
mkdir -p ${PLOTS_SAVE_PATH} &> /dev/null

declare -a environments=("apartment_4" "rococo_lab" "phd_office" "prof_office")
declare -a exploration_algorithms=("frontier_exploration" "teleop" "random_walk")
declare -a semantic_exploration=("ave" "s-ave")

## now loop through the above array
for env in "${environments[@]}" ; do
    echo -ne "-- Plotting metrics from environment ${orange}${RESULTS_LOAD_PATH}${yellow}${env}${nc}\n"
    json_load_paths=""
    for alg in "${exploration_algorithms[@]}" ; do
        #json_load_paths="${json_load_paths} ${RESULTS_LOAD_PATH}/${env}/${alg}/"
        for sem in "${semantic_exploration[@]}" ; do
            json_load_paths="${json_load_paths} ${RESULTS_LOAD_PATH}/${env}/${alg}/${sem}/"
        done
    done

# ori_distance
    python3 rosmeery_plot.py --save-dir ${PLOTS_SAVE_PATH} \
           --metric-keys time_s opi ori_surface ori_confidence  \
           --json-load-path ${json_load_paths}
    echo -en "\n"
done
