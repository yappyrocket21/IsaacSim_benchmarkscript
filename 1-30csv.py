import subprocess
outCsv=open("benchmarkCams.csv")
try:
    for i in range (30):
        subprocess.run(["_build/linux-x86_64/release/python.sh", "source/standalone_examples/benchmarks/benchmark_camera.py", "--num-cameras", i+1, "--resolution", "1920", "1080", "--num-gpus", "1"], capture_output=True, text=True, check=True)
except subprocess.CalledProcessError as e:
    print(f"Command failed with error: {e}")
    print(f"Error output: {e.stderr}")