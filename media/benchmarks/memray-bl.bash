#!/usr/bin/bash

# Generates memray-bl.bin and the corresponding plot
# Memray shows similar memory usage for better_launch and ros2

rm -f ./results/memory/memray-bl.bin ./results/memory/memray-bl.
memray run -o ./results/memory/memray-bl.bin bl.py better_launch 08_better_turtlesim.py
memray flamegraph ./results/memory/memray-bl.bin
firefox ./results/memory/memray-flamegraph-bl.html
