#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

img_array = np.load('/home/abkar/Carla 0.8.4/CarlaSimulator/PythonClient/_out/ep_001/CameraRGB/0.npy')

print(" image loaded")
print(img_array.shape)
fig, axes = plt.subplots(10,10, figsize=(8,8))

for i,ax in enumerate(axes.flat):
    print("reached here")
    ax.imshow(img_array[i])
