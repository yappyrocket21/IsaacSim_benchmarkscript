import subprocess
#outCsv=open("benchmarkCams.csv", "w", encoding="utf-8")
try:
    output_bytes = subprocess.check_output(['echo', 'Hello from subprocess!'])
    output_string = output_bytes.decode('utf-8')
    print(output_string)
    for i in range (30):
        tempOutput=subprocess.check_output(["_build/linux-x86_64/release/python.sh", "source/standalone_examples/benchmarks/benchmark_camera.py", "--num-cameras", f"{i+1}", "--resolution", "1920", "1080", "--num-gpus", "1"])
        tempOutput=tempOutput.decode('utf-8')
except subprocess.CalledProcessError as e:
    print(f"Command failed with error: {e}")
    print(f"Error output: {e.stderr}")
#outCsv.close()
print("Benchmarking completed. Results saved to benchmarkCams.csv.")