#!/bin/bash

# NOTE we can't profile bl because the bl script uses execvp to replace its process. 
# We profile the launch file instead, as bl is just a way to find and run the launch file.

python -m cProfile -o ./results/cprofile/bl.profile ../../scripts/bl better_launch 08_better_turtlesim.launch.py
gprof2dot -f pstats ./results/cprofile/bl.profile -o ./results/cprofile/bl.dot
dot -Tsvg ./results/cprofile/bl.dot -o ./results/cprofile/bl.svg
rm ./results/cprofile/bl.dot
