#!/bin/sh
# Evaluates a policy by executing `evaluate_policy.py` with multiple goals.
#
# This expects a file called `evaluate_policy.py` in the directory in which
# this script is executed!
# Further it expects an existing output directory to be passed as argument.

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <output_directory>"
    exit 1
fi

# abort if any command returns an error
set -e

# determine directory of this script, so other scripts from the same directory
# can be called below, independent of the working directory
thisdir=$(dirname $0)

# create temporary directory to store generated files
#tmpdir=$(mktemp -dt run_evaluation_XXXXXXXXX)

# use specified output directory
tmpdir=$1

if [ ! -d "${tmpdir}" ]; then
    echo "Output directory ${tmpdir} does not exist."
    exit 1
fi
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
