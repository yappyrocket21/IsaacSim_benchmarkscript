#Author: Oscar Van Gelder
#Date: 2023-10-04
#Purpose: Given folder of benchmark metrics from either 100x100 or 1080p IsaacSim rendering, generate CSV files measuring Mean FPS, GPU Dedicated Memory after runtime, Mean App Update Frametime, and System Memory RSS
#Hypothesis: Linear relationship as camera count increases.
#Observations: FPS decreases exponentially with camera count increase, everything else has linear relationship
from pathlib import Path
import re

def try_cast_float(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

try:
    folder_path = Path(input("Enter the folder path: "))
    csv_path = Path(input("Enter the output CSV file path (without csv file extension): ")+".csv")
    with csv_path.open("w") as csvfile:
        headCol1 = ["Camera Count", "GPU Memory Dedicated", "Mean FPS", "Mean App Update Frametime", "System Memory RSS"]
        headCol2 = ["Camera Count", "GPU Memory Dedicated", "Mean FPS", "Mean App_Update Frametime", "System Memory RSS", "Mean GPU Frametime"]
        csvfile.write(",".join(headCol2) + "\n")
        for file_path in folder_path.iterdir():
            with file_path.open("r") as file:
                meanFPS = "N/A"
                meanApp = "N/A"
                gpuMem = "N/A"
                gpuFrame = "N/A"
                sysMem = "N/A"
                rowb = []
                dupeCheck = []
                cameraCount = re.search(r"output_(\d+)", file_path.name).group(1)
                print(cameraCount)
                for line in file:
                    '''
                    for header in headCol2:
                        if header in line and not header in dupeCheck and try_cast_float(line.split(header)[1].strip().split()[0]):
                            rowb.append(line.split(header)[1].strip().split()[0])
                            dupeCheck.append(header)
                    '''
                    if "GPU Memory Dedicated:" in line:
                        gpuMem = line.split("GPU Memory Dedicated: ")[1].strip().split("GB")[0].strip()
                    if "Mean GPU Frametime:" in line:
                        gpuFrame = line.split("Mean GPU Frametime:")[1].strip().split("ms")[0].strip()
                    if "Mean FPS:" in line:
                        meanFPS = line.split("Mean FPS:")[1].strip().split("FPS")[0].strip()
                    if "Mean App_Update Frametime:" in line:
                        meanApp = line.split("Mean App_Update Frametime:")[1].strip().split("ms")[0].strip()
                    if "System Memory RSS:" in line:
                        sysMem = line.split("System Memory RSS:")[1].strip().split("GB")[0].strip()
                    
            rowa = [cameraCount, gpuMem, meanFPS, meanApp, sysMem, gpuFrame]
            '''
            if len(rowb) != len(headCol2):
                if len(rowb) > len(headCol2):
                    rowb = rowb[:len(headCol2)]
                for i in range(len(headCol2) - len(rowb)):
                    rowb.append("N/A")
            '''
            csvfile.write(",".join(str(x) for x in rowa) + "\n")
except Exception as e:
    print(f"An error occurred: {e}")
    exit(1)