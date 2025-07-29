for i in {1..30}; do
    _build/linux-x86_64/release/python.sh source/standalone_examples/benchmarks/benchmark_camera.py --num-cameras $i --resolution 1920 1080 --num-gpus 1 | grep Mean > output_$i.txt
    echo "Benchmark for $i cameras completed. Results saved to output_$i.txt."
done