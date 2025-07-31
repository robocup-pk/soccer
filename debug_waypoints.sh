#!/bin/bash
# Run basic_demo and capture relevant output
timeout 4s ./libs/algos/demo/basic_demo 2>&1 | grep -E "t=|finished|completed|waypoint" | tail -50