

from BallTracker import BallTracker





# PROGRAM MoCap-Master:
#     initialize: camPV_DQ[0,1] //DQ transformation of camera in space
#     initialize: hCam[0] = BallTracker(0)
#     initialize: hCam[1] = BallTracker(1)
#     run = True
#
#     //initialize variables that will be used in loop
#     initialize: ball[2]_from_cam[2]_DQ -- 2x2 DQ array
#     Initialize: ball[2]_pos -- 2x1 vec3
#     Initialize: ball[2]_pos_old = None -- 2x1 vec3
#
#
#     //initialize processing of first frame before enter loop
#     FOR i in 0,1: //camera
#         hCam[i].set_next_frame()
#     ENDFOR
#
#     //mainloop
#     WHILE run:
#         //Preforming CV component
#         FOR i in 0,1: //camera
#             ready = hCam[i].frame_ready(TIMEOUT)
#             IF not ready:
#                 exit()
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
