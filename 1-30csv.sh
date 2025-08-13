for i in {1..30}; do
#    echo "_build/linux-x86_64/release/python.sh source/standalone_examples/benchmarks/benchmark_camera.py --num-cameras $i --resolution 1920 1080 --num-gpus 1 | grep Mean > output_$i.txt"
    rawStats=$(_build/linux-x86_64/release/python.sh source/standalone_examples/benchmarks/benchmark_camera.py --num-cameras $i --resolution 1920 1080 --num-gpus 1 | grep Mean)
    appUpdate=$(sed -n 's/Mean App_Update Frametime: \(.*\) ms.*/\1/p' <<< $rawStats)
    physFrame=$(sed -n 's.*/Mean Physics Frametime: \(.*\) ms.*/\1/p' <<< $rawStats)
    fps=$(sed -n 's.*/Mean FPS: \(.*\) FPS/\1/p' <<< $rawStats)
    $rawStats>>output_csv.txt
    echo "Benchmark for $i cameras completed. Results saved to output_csv.txt."
done
