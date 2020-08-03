#!/bin/sh
# This script illustrates how final submissions will be evaluated.
#
# This expects a file called `evaluate_policy.py` in the directory in which
# this script is executed!
#
# Note that for the actual evaluation, "run_evaluate_policy_all_levels.py" will
# be executed in the Singularity image provided by the participants while
# "run_replay_all_levels.py" will be executed in the original challenge image
# provided by us, without any modifications by the participants.
# If the participants do not submit a custom Singularity image along with their
# code, the original image will be used for both steps.

# abort if any command returns an error
set -e

# determine directory of this script, so other scripts from the same directory
# can be called below, independent of the working directory
thisdir=$(dirname $0)

# create temporary directory to store generated files
tmpdir=$(mktemp -dt run_evaluation_XXXXXXXXX)
echo "Storing generated files in ${tmpdir}"

echo
echo Run Policy on Random Goals
echo ==========================
echo

python3 ${thisdir}/run_evaluate_policy_all_levels.py ${tmpdir}

echo
echo Replay Action Log
echo =================
echo

python3 ${thisdir}/run_replay_all_levels.py ${tmpdir}
