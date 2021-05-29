
import os
import numpy as np
import time
import math
# import mpmath
from scipy import interpolate

import PIL.Image
import PIL.ImageTk
import cv2 as cv

FRAME_TIME = 1.0 / 10
IM_RESOLUTION = 640


CONFIG_FILE_NAME="lens_mapping.txt"




with open(CONFIG_FILE_NAME,'r') as f_config:
    confStr = f_config.readline()[5:].strip().split(",")
    gridSize = float(confStr[0])
    gridDistance = float(confStr[1])
    GRID_X_SIZE = int(confStr[2])
    GRID_Y_SIZE = int(confStr[3])
    GRID_N_POINTS=GRID_X_SIZE*GRID_Y_SIZE
    print("Size: %.2f, dist: %.2f" % (gridSize, gridDistance))
    #read X Index
    xIndStr = f_config.readline()[5:].strip().split(",")
    yIndStr = f_config.readline()[5:].strip().split(",")
    xPosStr = f_config.readline()[5:].strip().split(",")
    yPosStr = f_config.readline()[5:].strip().split(",")



xInd = np.array([int(i) for i in xIndStr])
yInd = np.array([int(i) for i in yIndStr])
xPos = np.array([int(i) for i in xPosStr])
yPos = np.array([int(i) for i in yPosStr])

#alpha: angle projected onto xz plane
alpha=[0 for i in range(GRID_N_POINTS)]
#beta: angle projected onto yz plane
beta=[0 for i in range(GRID_N_POINTS)]



for i in range(GRID_N_POINTS):
    dx = xInd[i]*gridDistance
    dy = yInd[i]*gridDistance
    alpha[i] = math.atan(dx/gridDistance)
    beta[i]  = math.atan(dy/gridDistance)
    print("ang for X:%d, Y:%d, alp:%.10f, bet:%.10f"%(xInd[i], yInd[i], alpha[i], beta[i]))


##begin Interpolate code
f_beta = interpolate.interp2d(xPos, yPos, beta,)
