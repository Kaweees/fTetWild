#!/bin/bash
# run_profile.sh

BINARY="build/FloatTetwild_bin"
INPUT="ellipsoid.stl"

echo "Running perf record (sampling CPU at 999 Hz)..."
# Record the execution with call-graph enabled
perf record -F 999 -g -- $BINARY -i $INPUT -o temp_profile_mesh.msh --disable-filtering --max-threads 1 > /dev/null 2>&1
rm temp_*.msh && rm temp_*.csv && rm temp_*.obj

echo "Generating Flat Profile (Top Bottlenecks)..."
# Extract a flat list of where the most time is spent, ignoring who called whom.
# We limit to the top 50 lines to keep the summary concise for the AI.
perf report --stdio --no-children -F overhead,symbol | grep -v "^#" | grep -v "^$" | head -n 50 > flat_profile.txt

echo "Generating Full Call Graph (Hierarchical)..."
# Extract the full call graph so the AI can trace the exact execution paths.
# We output the whole thing, but standard perf reports are usually well within Opus's context window.
perf report --stdio > full_callgraph.txt

echo "Success! The agent can now read 'flat_profile.txt' and 'full_callgraph.txt'."