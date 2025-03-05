#!/usr/bin/env python3

"""Used for tracing down potential enhancements in memory usage.
"""

from better_launch import BetterLaunch, launch_this
from memory_profiler import memory_usage


def main():    
    bl = BetterLaunch()
    bl.include("better_launch", "08_better_turtlesim.py")


mem_usage = memory_usage((main, ), interval=0.2, timeout=5)
print(mem_usage)
