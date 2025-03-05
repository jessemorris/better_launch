#!/usr/bin/bash

# Generates memray-ros2.bin and the corresponding plot
# Memray shows similar memory usage for better_launch and ros2

rm -f ./results/memory/memray-ros2.bin ./results/memory/memray-ros2.
memray run -o ./results/memory/memray-ros2.bin bl.py better_launch 08_better_turtlesim.py
memray flamegraph ./results/memory/memray-ros2.bin
firefox ./results/memory/memray-flamegraph-ros2.html
