#!/bin/bash

if [ ! -f "$1" ]; then
    echo "Couldn't find bag file: $1"
    return 1
fi

if [ -d "$2" ]; then
    echo "ROS2 bag already exists: $2"
    return 1
fi

echo -e "Converting ROS 1 Bag:\n - $1 \nto ROS 2 Bag: \n - $2"

# --- 1. Inicialização do roscore do ROS 1 ---

bash -c 'roscore' &

# --- 2. Inicialização da Bridge ---

bash -c 'source /bridge_ws/source_bridge.sh && ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics < /dev/null' &

# --- 3. Gravação do Bag no ROS 2 ---

bash -c "source /bridge_ws/source_bridge.sh && ros2 bag record -a --use-sim-time -o $2 < /dev/null" &

# Espera um pouco para todos os processos anteriores iniciarem
sleep 3

# --- 4. Inicia o Bag do ROS 1 e encerra todos os processos quando ele acabar ---

bash -c "rosbag play $1 --wait-for-subscribers --clock" && kill -s SIGINT -$$ && \
kill -s SIGINT %3 && wait %3 && \
kill -s SIGINT %2 && wait %2 && \
kill -s SIGINT %1 && wait %1

echo "Bag $1 recorded correctly to folder $2"