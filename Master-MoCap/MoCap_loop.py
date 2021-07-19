from dqrobotics import *
from scipy.spatial.transform import Rotation
import math
import numpy as np
import numpy.linalg as linalg

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

    p1 = (cross(-1.0*m1, cross(l2, cross(l1, l2)))+dot(m2, cross(l1, l2))*l1)*((1.0/linalg.norm(vec4(cross(l1, l2))))**2)
    p2 = (cross(m2, cross(l1, cross(l1, l2)))-dot(m1, cross(l1, l2))*l2)*((1.0/linalg.norm(vec4(cross(l1, l2))))**2)
    cl12 = cross(l1, l2)
    pow = inv(norm(DQ(vec4(cl12))))
    p1 = (cross(-1.0*m1, cross(l2, cl12))+dot(m2, cl12)*l1)*pow*pow
    p2 = (cross(     m2, cross(l1, cl12))-dot(m1, cl12)*l2)*pow*pow

    # Need to do some validation logic here to make sure ball is somewhat calibrated
    valid=True

    return vec3(p1), vec3(p2), valid

def get_rik_from_pos(pos2):
    dir=pos2[0]-pos2[1]
    dir_l=linalg.norm(dir)
    if dir_l<0.001:
        return 0, 0
    i_rad=math.atan2(-dir[1],-dir[0])
    #new
    _tmpPOV=DQ([math.cos(-i_rad/2),0,0,math.sin(-i_rad/2)])
    _neLine = DQ.vec3(_tmpPOV*DQ(dir/dir_l)*DQ.conj(_tmpPOV))
    k_rad=math.atan2(_neLine[2],-_neLine[0])

    #old
    # k_rad=math.atan2(dir[2],dir[0])
    return math.degrees(i_rad), math.degrees(k_rad)



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
            return

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
        ################I I think I will break something here...
        xd_t_offset = np.array([0,-0.16,0], dtype=np.float32)
        # xd_t_offset = [0,-0.16,0]
        xd_component = np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)
        # xd_component = [0, 0, 0, 0, 0, 0]

        #     //initialize variables that will be used in loop
        #     initialize: ball[2]_from_cam[2]_DQ -- 2x2 DQ array
        plucker_i_from_cam_j = [[DQ([0]) for j in range(2)]  for i in range(2)]
        #     Initialize: ball[2]_pos -- 2x1 vec3
        # ballPos = [np.array([0,0,0]) for i in range(2)]
        #     Initialize: ball[2]_pos_old = None -- 2x1 vec3
        mocapPos_old = [np.array([0,0,0], dtype=np.float32) for i in range(2)]
        mocapCurrentZero=np.array([0,0,0], dtype=np.float32)
        master_on_flag=False
        master_init_clear_flag=True
        ######################### Initalize Loop variable   #########################
        ######################### Begin Loop #########################
        while not eExit.is_set() and not eError.is_set():

            ##send previous frame to vrep ..... because of reason ...
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


            # print("gripper val: %.3f"%MasterCommDataArray[0].gripper)

            if USE_SECONDARY:
                secMasterData = serialInterface.get_data()
                gripper_val = secMasterData[0]/1024.0*90-45
                MasterCommDataArray[0].gripper = gripper_val
                gripper_val=math.radians(gripper_val)
                xd_component[4] = secMasterData[1]/1024.0*360
                MasterCommDataArray[0].rot = xd_component[4]
                if secMasterData[5]>0:
                    master_on_flag = True
                elif secMasterData[5]<0
                    master_on_flag = False
                MasterCommDataArray[0].master_on = master_on_flag

            else:
                gripper_val = math.radians(MasterCommDataArray[0].gripper)
                xd_component[4] = MasterCommDataArray[0].rot


            # copy loop control variable
            xd_ui_override_flag = MasterCommDataArray[0].xdovrd
            zero_offset_flag = MasterCommDataArray[0].zero_offset
            master_on2off_flag=master_on_flag and not MasterCommDataArray[0].master_on
            master_off2on_flag=not master_on_flag and MasterCommDataArray[0].master_on
            master_on_flag = MasterCommDataArray[0].master_on

            #initalize xd_component
            ######################### When Using UI Override
            if xd_ui_override_flag:
                # print("MasterLoop: Enable Override: ", end="")
                _factor=0.01
                for i in range(6):
                    if i != 4:
                        xd_component[i]=MasterCommDataArray[0].xd[i]*_factor
                    if i == 2:
                        _factor = 1
                ##adding loop delay
                time.sleep(0.1)

                if zero_offset_flag:
                    for i in range(3):
                        xd_t_offset[i] += xd_component[i]
                        MasterCommDataArray[0].xd[i] = 0
                        xd_component[i]=0
                    MasterCommDataArray[0].zero_offset = 0
            ######################### When Motion Capture just turn off
            elif master_on2off_flag:
                print("MasterLoop: Mocap Master turnned of")
                # print("xd offset old: %s "%str(xd_t_offset))
                # print("xd component old: %s "%str(xd_component))
                for i in range(3):
                    xd_t_offset[i]+=xd_component[i]
                    xd_component[i]=0
                # print("xd offset old: %s "%str(xd_t_offset))
                # print("xd component old: %s "%str(xd_component))
            ######################### When Using Motion Capture
            elif master_on_flag == 1:
                #trigger on first time turning on
                if master_off2on_flag:    ##When Master turn off after on
                    print("MasterLoop: Mocap Master turnned on")
                    master_init_clear_flag=False
                    tracker[0].set_next_frame()
                    tracker[1].set_next_frame()
                    time.sleep(0.5)


                ########## Request Data from frame
                if not tracker[0].frame_ready(10) or not tracker[1].frame_ready(10):
                    # Camera Broke for some reason
                    eError.set()
                    print("MasterLoop: Camera Broke for some reason")
                    break

                #grab this frame data
                valid=[True, True]
                for (i, j) in [(1,0),(0,0),(1,1),(0,1)]:
                    _tmp, plucker_i_from_cam_j[i][j] = tracker[j].get_ball_dq_pov_plucker(i, CAM_POV[j])
                    valid[i] = valid[i] and _tmp
                ########## Ser to calculate Next Frame
                tracker[0].set_next_frame()
                tracker[1].set_next_frame()

                pt10, pt11, val = get_closest_point_between_lines(plucker_i_from_cam_j[1][0],
                            plucker_i_from_cam_j[1][1])
                pt00, pt01, val = get_closest_point_between_lines(plucker_i_from_cam_j[0][0],
                            plucker_i_from_cam_j[0][1])
                pt=[(pt00+pt01)/2-PT_ZERO, (pt10+pt11)/2-PT_ZERO]
                difPt=[linalg.norm(pt00-pt01), linalg.norm(pt10-pt11)]
                valid[0] = valid[0] and difPt[0]<MOCAP_ERROR_THRESHOLD
                valid[1] = valid[1] and difPt[1]<MOCAP_ERROR_THRESHOLD

                # mocapPos_old
                # _, _tmp=tracker[1].get_ball_dq_origin_plucker(0)
                # print("greenball: %s "%str(_tmp))
                # print("greenball: %s "%str(plucker_i_from_cam_j[0][0]))
                # print("green: %s "%np.array2string(pt[0], precision=3, separator=',') , end="")
                # print("distance %.3f"%( difPt[0] ))
                # print("red: %s "%np.array2string(pt[1], precision=3, separator=',') , end="")
                # print("distance %.3f"%( difPt[1] ))

                if not master_init_clear_flag:
                    if not valid[BASE_B]:
                        continue
                        master_init_clear_flag=True
                        mocapCurrentZero=pt[BASE_B].copy()
                        # print("cap Zero : %.3f,%.3f,%.3f "%(mocapCurrentZero[0],mocapCurrentZero[1],mocapCurrentZero[2]))
                        # for i in range(3):
                        #     xd_t_offset[i] += (pt[BASE_B][i]-mocapCurrentZero[i])*MOCAP_MOTION_SCALING
                        #     xd_component[i]=0


                #use previous if error
                for i in range(2):
                    if valid[i]:
                        mocapPos_old[i] = pt[i].copy()
                    else:
                        pt[i] = mocapPos_old[i].copy()


                # print("xdconti : %s "%str(xd_component))
                # print("sdfsdF %s"%str(xd_component))
                # print("sdfsdF %s"%str(mocapCurrentZero))
                # print("sdfsdF %s"%str(pt0))
                #x
                xd_component[1] = (pt[BASE_B][0]-mocapCurrentZero[0])*MOCAP_MOTION_SCALING
                #y
                xd_component[2] = (pt[BASE_B][1]-mocapCurrentZero[1])*MOCAP_MOTION_SCALING
                #z
                xd_component[0] = (pt[BASE_B][2]-mocapCurrentZero[2])*MOCAP_MOTION_SCALING

                if valid[0] and valid[1]:
                    ri, rk = get_rik_from_pos(pt)
                    xd_component[3]=ri
                    xd_component[5]=rk


                #send xd info back to UI
                for i in range(3):
                    MasterCommDataArray[0].xd[i]=xd_component[i]*100
                MasterCommDataArray[0].xd[3]=xd_component[3]
                MasterCommDataArray[0].xd[5]=xd_component[5]



        #########################   End Loop ########################
    except Exception as e:
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        print(exc_type, fname, exc_tb.tb_lineno)
        print(e)

    shutdown()
    return



if __name__ == '__main__':
    main()
