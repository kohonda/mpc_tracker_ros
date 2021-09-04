#!/bin/bash

# Script for running offline simulation 

trap "kill 0" SIGINT

SCRIPT_DIR=$(cd $(dirname $0); pwd)
PROJECT_BASE_DIR=${SCRIPT_DIR%/*}
SRC_DIR=${PROJECT_BASE_DIR%/*}
WS_DIR=${SRC_DIR%/*}

OFFLINE_SIMULATION=${WS_DIR}"/install/mpc_tracker/lib/mpc_tracker/simulation"

# INPUT_REFERENCE_PATH=${PROJECT_BASE_DIR}"/simulation/reference_path/sinwave_big.csv"
# INPUT_REFERENCE_PATH=${PROJECT_BASE_DIR}"/simulation/reference_path/sinwave_05.csv"
# INPUT_REFERENCE_PATH=${PROJECT_BASE_DIR}"/simulation/reference_path/sinwave_075.csv"
INPUT_REFERENCE_PATH=${PROJECT_BASE_DIR}"/simulation/reference_path/sinwave_1.csv"
# INPUT_REFERENCE_PATH=${PROJECT_BASE_DIR}"/simulation/reference_path/curv_90deg.csv"
# INPUT_REFERENCE_PATH=${PROJECT_BASE_DIR}"/simulation/reference_path/step_1m.csv"

OUTPUT_RESULT=${PROJECT_BASE_DIR}"/simulation/simulation_result/result.csv"

$OFFLINE_SIMULATION $INPUT_REFERENCE_PATH $OUTPUT_RESULT