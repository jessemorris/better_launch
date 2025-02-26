#!/usr/bin/bash

# Generates memray-ros2.bin and the corresponding plot
# Memray shows similar memory usage for better_launch and ros2

rm -f ./reports/memray-ros2.bin ./reports/memray-ros2.
memray run -o ./reports/memray-ros2.bin bl.py better_launch 08_better_turtlesim.py
memray flamegraph ./reports/memray-ros2.bin
firefox ./reports/memray-flamegraph-ros2.html
