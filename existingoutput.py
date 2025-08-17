#Author: Oscar Van Gelder
#Date: 2023-10-04
#Purpose: Given folder of benchmark metrics from either 100x100 or 1080p IsaacSim rendering, generate CSV files measuring Mean FPS, GPU Dedicated Memory after runtime, Mean App Update Frametime, and System Memory RSS
#Hypothesis: Linear relationship as camera count increases.
#Observations: FPS decreases exponentially with camera count increase, everything else has linear relationship
from pathlib import Path
import re
try:
    folder_path = Path(input("Enter the folder path: "))
    csv_path = Path(input("Enter the output CSV file path (without csv file extension): ")+".csv")
    with csv_path.open("w") as csvfile:
        headCol1 = ["Camera Count", "GPU Memory Dedicated", "Mean FPS", "Mean App Update Frametime", "System Memory RSS"]
        headCol2 = ["Camera Count", "GPU Memory Dedicated", "Mean FPS", "Mean App Update Frametime", "System Memory RSS"]
        csvfile.write(",".join(headCol1) + "\n")
        for file_path in folder_path.iterdir():
            gpuDedMemFlag=0
            with file_path.open("r") as file:
                meanFPS = 0
                meanApp = 0
                gpuMem = 0
                sysMem = 0
                cameraCount = re.search(r"output_(\d+)", file_path.name).group(1)
                print(cameraCount)
                for line in file:
                    if "GPU Memory Dedicated:" in line:
                        #Only the second instance - after program runthrough - matters
                        if gpuDedMemFlag==1:
                            gpuMem = line.split("GPU Memory Dedicated: ")[1].strip().split("GB")[0].strip()
                        gpuDedMemFlag=1
                    if "Mean FPS:" in line:
                        meanFPS = line.split("Mean FPS:")[1].strip().split("FPS")[0].strip()
                    if "Mean App_Update Frametime:" in line:
                        meanApp = line.split("Mean App_Update Frametime:")[1].strip().split("ms")[0].strip()
                    if "System Memory RSS:" in line:
                        sysMem = line.split("System Memory RSS:")[1].strip().split("GB")[0].strip()
            csvfile.write(f"{cameraCount},{gpuMem},{meanFPS},{meanApp},{sysMem}\n")
except Exception as e:
    print(f"An error occurred: {e}")
    exit(1)