from dqrobotics import *
from scipy.spatial.transform import Rotation
import math
import numpy as np
import multiprocessing as mp


from BallTracker import BallTracker


############################## program parameter ##############################
BACKEND='py'
recvPort = 3344 #+1 will be taken also by frontend
sendPort = 2345 #+1 will be taken also by frontend
############################## program parameter ##############################

############################## Target Setup ##############################
#in format of Lower H, S, V
#             Upper H, S, V
DEFAULT_greenLimit = [[71, 59, 31], [108, 144, 78]]
DEFAULT_redLimit = [[111, 106, 24], [184, 255, 210]]
INTEPOLER_FILE_NAME = "./camera_config/lens_map.sciobj"
############################## Target Setup ##############################

############################## Frame Setup ##############################
#     initialize: camPV_DQ[0,1] //DQ transformation of camera in space
L=25
CAM_r1=DQ([math.cos(math.pi/4),0,0,math.sin(math.pi/4)])*DQ([math.cos(3*math.pi/8),math.sin(3*math.pi/8),0,0])
CAM_t1=DQ([0, 0, L, 2*L])
CAM1_POV = CAM_r1+E_*CAM_t1*CAM_r1*0.5
CAM_r2=DQ([math.cos(3*math.pi/8),math.sin(3*math.pi/8),0,0])
CAM_t2=DQ([0, L , 2*L,  2*L])
CAM2_POV = CAM_r2+E_*CAM_t2*CAM_r2*0.5
print(CAM1_POV)
print(CAM2_POV)
############################## Frame Setup ##############################


############################## Start of Program ##############################

##event for monitoring error
eError = mp.Event()

##Initializeing Camera Tracker Setting
#     initialize: hCam[0] = BallTracker(0)
#     initialize: hCam[1] = BallTracker(1)
tracker1=BallTracker(0, eError, recvPort, sendPort, backend=BACKEND)
tracker2=BallTracker(1, eError, recvPort-1, sendPort-1, backend=BACKEND)
if not tracker1.begin_capture():
    print("Error: with openCV")
    exit()
if not tracker2.begin_capture():
    print("Error: with openCV")
    exit()

tracker1.set_mask_color(DEFAULT_greenLimit, DEFAULT_redLimit)
tracker2.set_mask_color(DEFAULT_greenLimit, DEFAULT_redLimit)
print("Frame Size: %d x %d"%tracker1.get_frame_size())
tracker1.set_lens_mapping(INTEPOLER_FILE_NAME)
tracker2.set_lens_mapping(INTEPOLER_FILE_NAME)

#     //initialize processing of first frame before enter loop
#     FOR i in 0,1: //camera
#         hCam[i].set_next_frame()
#     ENDFOR
print("Holding start")
time.sleep(2)
print("sending start signal")
tracker1.set_next_frame()
tracker2.set_next_frame()

#     //initialize variables that will be used in loop
#     initialize: ball[2]_from_cam[2]_DQ -- 2x2 DQ array
ball_i_from_cam_j = [[DQ(0) for j in range(2)]  for i in range(2)]
#     Initialize: ball[2]_pos -- 2x1 vec3
ball_i_pos = [np.Array([0,0,0]) for i in range(2)]
#     Initialize: ball[2]_pos_old = None -- 2x1 vec3
ball_i_pos_old = [np.Array([0,0,0]) for i in range(2)]

#     run = True
run = True

#     //mainloop
#     WHILE run:
while run:
    #         //Preforming CV component
#   if not ready exit
    if not tracker1.frame_ready(10):
        break
    if not tracker2.frame_ready(10):
        break

    # for i in range(1000):
    #         print("getting Frame Timed out")
    #         tracker1.exit()
    #         exit()
    #     tracker1.set_next_frame()
    #     # pos = tracker1.get_ball_angle(1)
    #     # print("cnt: %d -- x: %.5f, y: %.5f"%(i, pos[1], pos[2]))
    #     pos = tracker1.get_ball_dq_origin_plucker(1)
    #     print("cnt: %d -- DQ: %s"%(i, str(pos[1])))
    # tracker1.exit()
# PROGRAM MoCap-Master:
#
#
#
#
#             ready = hCam[i].frame_ready(TIMEOUT)
#             IF not ready:
#             ENDIF
#         ENDFOR
#         FOR i in 0,1:
#             hCam[i].set_next_frame()
#         ENDFOR
#
#         //Calculating Ball0 and Ball1 position of 3d Space
#         error = False
#         FOR id in 0,1: //Ball ID
#             detected0, ball[id]_from_cam[0]_DQ = hCam[i].get_ball_dq_pov_plucker(camPV_DQ[i])
#             detected1, ball[id]_from_cam[1]_DQ = hCam[i].get_ball_dq_pov_plucker(camPV_DQ[i])
#             IF detected0 AND detected1:
#                 err, ptA, ptB = find_closest_point_on_lines(ball[id]_from_cam[0]_DQ, ball[id]_from_cam[1]_DQ)
#                 IF not err:
#                     ball[id]_pos = midpoint(ptA, ptB)
#                 ELSE:
#                     error = True
#                     break
#                 ENDIF
#             ELSE:
#                 error = True
#                 break
#             ENDIF
#         ENDFOR
#
#         IF error:
#             ball[:]_pos = ball[:]_pos_old
#         ELSE:
#             ball[:]_pos_old = ball[:]_pos
#         ENDIF
#
#         IF not any ball[:]_pos == None:  //make sure wait for first valid data before sending
#             //Calculating where Robot needs to be and what orientation
#             //ball[0] define where the end effector is located,
#             //ball[1] to ball[0] direction, gives orientation.
#             xd_DQ = get_taskspace_from_balls(ball[0]_pos, ball[1]_pos)
#
#             //send result to vrep interface
#             send_to_vrep(xd_DQ)
#         ENDIF
#
#     ENDWHILE
# END_PROGRAM
