import subprocess
import time
import statistics
import sys
import os
import json

def benchmark_ftetwild(binary_path, input_file, num_runs=5):
    if not os.path.isfile(input_file):
        print(f"ERROR: Input file '{input_file}' not found.")
        sys.exit(1)

    if not os.path.isfile(binary_path):
        print(f"ERROR: Binary '{binary_path}' not found. Check the path.")
        sys.exit(1)

    print(f"Starting benchmark: {binary_path} on {input_file} ({num_runs} runs)...\n")
    times = []

    for i in range(num_runs):
        print(f"Run {i+1}/{num_runs}... ", end="", flush=True)
        start_time = time.perf_counter()
        cmd = [binary_path, "-i", input_file, "-o", "temp_agent_output.msh", "--disable-filtering", "--max-threads", "1"]
        try:
            print(cmd)
            result = subprocess.run(
                cmd,
                #stdout=subprocess.DEVNULL, # Suppresses normal output to save agent context
                stderr=subprocess.PIPE,    # Captures errors if the run fails
                text=True,
                check=True
            )
        except subprocess.CalledProcessError as e:
            print(f"FAILED!\nError during run {i+1}. fTetWild returned code {e.returncode}.")
            print(f"Stderr:\n{e.stderr}")
            sys.exit(1)

        end_time = time.perf_counter()
        elapsed = end_time - start_time
        times.append(elapsed)
        print(f"{elapsed:.4f} seconds")

    # Clean up temporary output so the disk doesn't fill up during iterations
    if os.path.exists("temp_agent_output.msh"):
        os.remove("temp_agent_output.msh")

    # Calculate statistics
    avg_time = statistics.mean(times)
    median_time = statistics.median(times)
    min_time = min(times)
    max_time = max(times)
    std_dev = statistics.stdev(times) if num_runs > 1 else 0.0

    # Print a clean, easily parsable summary for the CLI agent
    print("\n=== BENCHMARK SUMMARY ===")
    print(f"Total Runs:    {num_runs}")
    print(f"Average Time:  {avg_time:.4f} s")
    print(f"Median Time:   {median_time:.4f} s")
    print(f"Min Time:      {min_time:.4f} s")
    print(f"Max Time:      {max_time:.4f} s")
    print(f"Std Deviation: {std_dev:.4f} s")
    print("=========================")

    # Optional: Write to a JSON file if you want the agent to read it programmatically
    results_dict = {
        "runs": num_runs,
        "average_s": avg_time,
        "median_s": median_time,
        "min_s": min_time,
        "max_s": max_time,
        "std_dev_s": std_dev,
        "raw_times": times
    }
    with open("benchmark_results.json", "w") as f:
        json.dump(results_dict, f, indent=4)

if __name__ == "__main__":
    # Ensure these paths match your environment. 
    # fTetWild is often compiled as 'FloatTetwild_bin' by default.
    FTETWILD_BIN = "build/FloatTetwild_bin" 
    INPUT_STL = "ellipsoid.stl"
    NUM_ITERATIONS = 5
    
    benchmark_ftetwild(FTETWILD_BIN, INPUT_STL, NUM_ITERATIONS)