mkdir output_100
mkdir output_1080

for i in {1..100..2}; do
    commandLine="../IsaacSim/_build/linux-x86_64/release/python.sh /home/dvangelder/IsaacSim/source/standalone_examples/benchmarks/benchmark_camera.py --num-cameras $i --resolution 100 100 --num-gpus 1 --/crashreporter/enabled=false "
    echo $commandLine
    # $commandLine > output_100/output_$i.txt
done


for i in {49..80..2}; do
    commandLine="../IsaacSim/_build/linux-x86_64/release/python.sh /home/dvangelder/IsaacSim/source/standalone_examples/benchmarks/benchmark_camera.py --num-cameras $i --resolution 1920 1080 --num-gpus 1 --/crashreporter/enabled=false"
    echo $commandLine
    $commandLine > output_1080/output_$i.txt
done


