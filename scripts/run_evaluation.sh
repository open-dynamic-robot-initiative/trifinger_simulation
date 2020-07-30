#!/bin/sh
# This script illustrates how final submissions will be evaluated.

# arguments are difficulty, initial pose, goal pose
./evaluate_policy.py \
    1 \
    '{"position": [-0.0779, -0.0238, 0.0325], "orientation": [0.0, 0.0, 0.9984469480724629, -0.05571078786720278]}' \
    '{"position": [-0.0681, -0.0979, 0.0325], "orientation": [0, 0, 0, 1]}'

echo
echo Replay Action Log
echo =================
echo

# the above script writes a file "action_log.json".  Based on this, the result is
# verified by replaying the logged actions in the simulation.
./replay_action_log.py \
    --logfile=action_log.json \
    --difficulty=1 \
    --initial-pose='{"position": [-0.0779, -0.0238, 0.0325], "orientation": [0.0, 0.0, 0.9984469480724629, -0.05571078786720278]}' \
    --goal-pose='{"position": [-0.0681, -0.0979, 0.0325], "orientation": [0, 0, 0, 1]}'
