#!/usr/bin/bash

# Generates memray-bl.bin and the corresponding plot
# Memray shows similar memory usage for better_launch and ros2

rm -f ./reports/memray-bl.bin ./reports/memray-bl.
memray run -o ./reports/memray-bl.bin bl.py better_launch 08_better_turtlesim.py
memray flamegraph ./reports/memray-bl.bin
firefox ./reports/memray-flamegraph-bl.html
