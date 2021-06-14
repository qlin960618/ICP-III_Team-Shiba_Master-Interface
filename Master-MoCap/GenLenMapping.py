
import os
import numpy as np
import time
import math
# import mpmath
from scipy import interpolate

import PIL.Image
import PIL.ImageTk
import cv2 as cv

try:
    import cPickle as pickle
except ImportError:
    import pickle

FRAME_TIME = 1.0 / 10
IM_RESOLUTION = 640


CONFIG_FILE_NAME="./camera_config/lens_mapping.txt"
INTEPOLER_FILE_NAME = "./camera_config/lens_map.sciobj"
INTERPOLORX_FILE_NAME = "./camera_config/lens_mapx.sciobj"
INTERPOLORY_FILE_NAME = "./camera_config/lens_mapy.sciobj"

LOAD_FROM_SCIOBJ=True

if not LOAD_FROM_SCIOBJ:
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
    alpha=np.zeros(GRID_N_POINTS)
    #beta: angle projected onto yz plane
    beta=np.zeros(GRID_N_POINTS)

    for i in range(GRID_N_POINTS):
        dx = xInd[i]*gridDistance
        dy = yInd[i]*gridDistance
        alpha[i] = dx
        beta[i]  = dy
        # print("ang for X:%d, Y:%d, alp:%.10f, bet:%.10f"%(xInd[i], yInd[i], alpha[i], beta[i]))

    ##begin Interpolate code
    # f_alpha = interpolate.interp2d(xPos, yPos, alpha, 'linear')
    # f_beta = interpolate.interp2d(xPos, yPos, beta, 'linear')
    f_alpha = interpolate.Rbf(xPos, yPos, alpha, function="linear")
    f_beta = interpolate.Rbf(xPos, yPos, beta, function="linear")
    #### Note
    """
    look into perhaps accounting for symmery of the camera
    for alpha - x pixel symmetry
    for beta - y pixel symmetry    
    """

    tosave = [f_alpha, f_beta, gridDistance]
    #Save interpolate to file
    with open(INTEPOLER_FILE_NAME, 'wb') as f:
        pickle.dump(tosave, f)
    # with open(INTERPOLORX_FILE_NAME, 'wb') as f:
    #     pickle.dump(f_alpha, f)
    # with open(INTERPOLORY_FILE_NAME, 'wb') as f:
    #     pickle.dump(f_beta, f)
else:
    with open(INTEPOLER_FILE_NAME, 'rb') as f:
        tosave = pickle.load(f)
        f_alpha = tosave[0]
        f_beta = tosave[1]
        gridDistance = tosave[2]
    # with open(INTERPOLORX_FILE_NAME, 'rb') as f:
    #     f_alpha = pickle.load(f)
    # with open(INTERPOLORY_FILE_NAME, 'rb') as f:
    #     f_beta = pickle.load(f)


# import matplotlib.pyplot as plt
# xnew = np.arange(0,1279,2)
# ynew = np.arange(-5.01, 5.01, 1e-2)
# znew = f_alpha(xnew, ynew)
# plt.plot(xPos, z[0, :], 'ro', xnew, znew[0, :], 'b-')
# plt.show()

import matplotlib.pyplot as plt
xnew , ynew = np.meshgrid(np.linspace(0,1279,50),np.linspace(0,719,50))
ynew = ynew.ravel()
xnew = xnew.ravel()
alpha_new = f_alpha(xnew, ynew)
beta_new = f_beta(xnew, ynew)

print(xnew.shape)
print(ynew.shape)
print(beta_new.shape)

fig = plt.figure(figsize = (10, 7))
ax = plt.axes(projection ="3d")
# Creating plot
ax.scatter3D(xnew, ynew, alpha_new, color="blue", marker='.')
ax.scatter3D(xnew, ynew, beta_new, color="green", marker='.')
ax.set_xlim(0, 1279)
ax.set_ylim(0, 719)
ax.set_xlabel('X pixel position')
ax.set_ylabel('Y pixel position')
ax.set_zlabel('Angle (rad)')
ax.legend(['Alpha (x)', 'Beta (y)'])


plt.show()



# import matplotlib.pyplot as plt
# xnew , ynew = np.meshgrid(np.linspace(0,1279,20),np.linspace(0,719,20))
# ynew = ynew.ravel()
# xnew = xnew.ravel()
# # print(xnew.shape)
# z_xnew = f_alpha(xnew, ynew)
# z_ynew = f_beta(xnew, ynew)
# plt.quiver(xnew,ynew,z_xnew,z_ynew,linewidths=1)
# # plt.plot(xPos, alpha, 'r*', xnew, znew[0, :], 'b')
# plt.show()
