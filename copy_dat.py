import numpy as np

chunks = [31, 25, 32, 26, 33, 27, 34, 28, 35, 29, 36, 30]
with open("input.dat", "w") as fdat:   
    for i in range(0, len(chunks)):
        try:
            with open("results/ofmap_layer" + str(chunks[i]) + ".dat", "r") as tempfile:
                for line in tempfile:
                    if line != line[0] != "x":
                        fdat.write(str(np.int16(int(line, 0))) + "\n")
        except ValueError:
            print("Can not Open Chunk ", chunks[i])
           
