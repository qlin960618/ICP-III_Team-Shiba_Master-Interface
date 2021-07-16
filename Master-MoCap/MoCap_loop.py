from dqrobotics import *
from scipy.spatial.transform import Rotation
import math
import numpy as np


from BallTracker import BallTracker
from ctypes import Structure, c_double, c_int, c_bool
import multiprocessing as mp
from multiprocessing.sharedctypes import Array

from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics import *
from umirobot_vrep_robot import UMIRobotVrepRobot
from ardu_nano_interface import SerialMasterInterface

#Import Variable defined in main
from MoCap_main import *

        ##xd use unit of cm and deg
class MasterCommDataStruc(Structure):
    _fields_ = [('xd', c_double*6), ('xd_DQ', c_double*8),
                ('rot', c_double), ('gripper', c_double),
                ('xdovrd', c_int), ('zero_offset', c_bool),
                ('master_on', c_bool)]


def get_xd_from_trans_rot(t_i, t_j, t_k, r_i, r_j, r_k):
    _t = [t_i, t_j, t_k]
    _t = DQ(_t)
    _ri = DQ([math.cos(math.radians(r_i)/2), math.sin(math.radians(r_i)/2), 0, 0])
    _rj = DQ([math.cos(math.radians(r_j)/2), 0, math.sin(math.radians(r_j)/2), 0])
    _rk = DQ([math.cos(math.radians(r_k)/2), 0, 0, math.sin(math.radians(r_k)/2)])
    _r=_ri*_rk*_rj
    return (_r+DQ.E*_t*_r*0.5)

def get_closest_point_between_lines(plk_l1, plk_l2):

    m1 = DQ.D(plk_l1)
    l1 = DQ.P(plk_l1)
    m2 = DQ.D(plk_l2)
    l2 = DQ.P(plk_l2)

    p1 = (cross(-1.0*m1, cross(l2, cross(l1, l2)))+dot(m2, cross(l1, l2))*l1)*pow(1.0/vec4(cross(l1, l2)).norm(), 2)
    p2 = (cross(m2, cross(l1, cross(l1, l2)))-dot(m1, cross(l1, l2))*l2)*pow(1.0/vec4(cross(l1, l2)).norm(), 2)

    # Need to do some validation logic here to make sure ball is somewhat calibrated
    valid=True

    return p1, p2, valid

def master_loop(MasterCommDataArray, eExit, eError, tracker, serialInterface):

    def shutdown():
        if USE_SECONDARY:
            serialInterface.reset()
        if vrep_interface is not None:
            vrep_interface.disconnect_all()
        for t in tracker:
            if t is not None:
                t.exit()

    print("Entered master for testing")
    if serialInterface is None:
        USE_SECONDARY = False
    else:
        USE_SECONDARY = True

    try:
        ######################### initialize vrep interface #########################
        vrep_interface = DQ_VrepInterface()
        if not vrep_interface.connect(VREP_ADDRESS, 20001, 100, 10):
            # If connection fails, disconnect and throw exception
            vrep_interface.disconnect_all()
            vrep_interface=None
            eError.set()
            print("MasterLoop: Vrep Connection Failed")
            shutdown()
        umirobot_vrep = UMIRobotVrepRobot(vrep_interface=vrep_interface)
        ######################### initialize vrep interface #########################
        ######################### Get robot pose info       #########################
        umirobot_kinematics = umirobot_vrep.kinematics()
        q_init = umirobot_vrep.get_q_from_vrep()
        x_init = umirobot_kinematics.fkm(q_init)
        umirobot_vrep.show_xd_in_vrep(x_init)
        x_master_ref = vrep_interface.get_object_pose("x_master_ref")
        ######################### Get robot pose info       #########################

        ######################### Initalize Loop variable   #########################
        gripper_val = 0
        xd_t_offset = [0,-0.16,0]
        xd_component = [0, 0, 0, 0, 0, 0]
        #     //initialize variables that will be used in loop
        #     initialize: ball[2]_from_cam[2]_DQ -- 2x2 DQ array
        plucker_i_from_cam_j = [[DQ([0]) for j in range(2)]  for i in range(2)]
        #     Initialize: ball[2]_pos -- 2x1 vec3
        ball_i_pos = [np.array([0,0,0]) for i in range(2)]
        #     Initialize: ball[2]_pos_old = None -- 2x1 vec3
        ball_i_pos_old = [np.array([0,0,0]) for i in range(2)]
        ######################### Initalize Loop variable   #########################
        ######################### Begin Loop #########################
        while not eExit.is_set() and not eError.is_set():
            # print("gripper val: %.3f"%MasterCommDataArray[0].gripper)

            if USE_SECONDARY:
                secMasterData = serialInterface.get_data()
                gripper_val = math.radians(secMasterData[0]/1024.0*180-90)/2
                MasterCommDataArray[0].gripper = math.degrees(gripper_val)
                xd_component[4] = secMasterData[1]/1024.0*360-180
                MasterCommDataArray[0].rot = xd_component[4]
            else:
                gripper_val = math.radians(MasterCommDataArray[0].gripper)
                xd_component[4] = MasterCommDataArray[0].rot


            # copy loop control variable
            zero_offset = MasterCommDataArray[0].zero_offset
            master_on = MasterCommDataArray[0].master_on
            #initalize xd_component
            ######################### When Using UI Override
            if MasterCommDataArray[0].xdovrd == 1:
                # print("MasterLoop: Enable Override: ", end="")
                _factor=0.01
                for i in range(6):
                    if i != 4:
                        xd_component[i]=MasterCommDataArray[0].xd[i]*_factor
                    if i == 2:
                        _factor = 1
                ##adding loop delay
                time.sleep(0.1)

            ######################### When Using Motion Capture
            elif zero_offset == 0 and master_on == 1:
                ########## Request Data from frame
                if not tracker[0].frame_ready(10) or not tracker[1].frame_ready(10):
                    # Camera Broke for some reason
                    eError.set()
                    print("MasterLoop: Camera Broke for some reason")
                    break

                _, plucker_i_from_cam_j[1][0] = tracker[0].get_ball_dq_pov_plucker(1, CAM1_POV)
                _, plucker_i_from_cam_j[0][0] = tracker[0].get_ball_dq_pov_plucker(0, CAM1_POV)
                _, plucker_i_from_cam_j[1][1] = tracker[1].get_ball_dq_pov_plucker(1, CAM2_POV)
                _, plucker_i_from_cam_j[0][1] = tracker[1].get_ball_dq_pov_plucker(0, CAM2_POV)
                pt11, pt12, val = get_closest_point_between_lines(plucker_i_from_cam_j[1][0],
                            plucker_i_from_cam_j[1][1])
                pt01, pt02, val = get_closest_point_between_lines(plucker_i_from_cam_j[0][0],
                            plucker_i_from_cam_j[0][1])
                print("Loop Result: %s"%(str(pt11)))




                ########## Ser to calculate Next Frame
                tracker[0].set_next_frame()
                tracker[1].set_next_frame()
            # else:
                # xd_component=[0,0,0,0,0,0]
                # print(vv)


            if MasterCommDataArray[0].zero_offset:
                for i in range(3):
                    xd_t_offset[i] += xd_component[i]
                    MasterCommDataArray[0].xd[i] = 0
                    xd_component[i]=0
                MasterCommDataArray[0].zero_offset = 0

            ###get xd
            _tmp = xd_component.copy()
            _tmp[0] += xd_t_offset[0]
            _tmp[1] += xd_t_offset[1]
            _tmp[2] += xd_t_offset[2]
            xd = get_xd_from_trans_rot(*_tmp)
            # xd = get_xd_from_trans_rot(xd_t_offset[0]+vv[0]/100, xd_t_offset[1]+vv[1]/100,
            # xd_t_offset[2]+vv[2]/100, vv[3], vv[4], vv[5])
            ###write to vrep
            umirobot_vrep.show_xd_in_vrep(x_master_ref*xd)
            umirobot_vrep.send_gripper_value_to_vrep(gripper_val)

        #########################   End Loop ########################
    except Exception as e:
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        print(exc_type, fname, exc_tb.tb_lineno)
        print(e)

    shutdown()
    return













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



if __name__ == '__main__':
    main()
