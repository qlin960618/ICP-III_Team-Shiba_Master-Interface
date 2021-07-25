from dqrobotics.interfaces.vrep import DQ_VrepInterface
# from dqrobotics.robot_modeling import DQ_Kinematics
from umirobot_task_space_control.umirobot_vrep_robot import UMIRobotVrepRobot
from dqrobotics import *
# from dqrobotics.solvers import DQ_QuadprogSolver

from ardu_nano_interface import SerialMasterInterface

import multiprocessing as mp
import numpy as np
import time
import math
from umirobot import UMIRobot
import sys, os



SERIAL_PID = 29987
# VREP_ADDRESS = '192.168.10.102'
# VREP_ADDRESS = '10.213.113.136'
# VREP_ADDRESS = '34.85.17.130'
VREP_ADDRESS = '172.20.10.7'
VREP_ADDRESS = '192.168.137.163'

# VREP_PORT = 1995
VREP_PORT = 20001

####program flow control
LOOP_RATE = 30 #hz

####################### Master Dimension
LINK_L=10.0/100
ANGLE_OFFSET = np.array([0.103, -0.0042,0.011937,0.099285,0])
ANGLE_SIGN = np.array([1,1,1,-1,1])

####################### Motion Parameter
T_XY_M_SCALING = 1.3
T_Z_M_SCALING = 1/16.0
GRIPPER_ANGULAR_RATE=0.8

MOTION_RUNNING_WEIGHT = 0.2

def main():
    # Instantiate a DQ_VrepInterface
    vrep_interface = DQ_VrepInterface()


    eError = mp.Event()
    # initialize Secondary Master Interface controller
    masterInterface = SerialMasterInterface(eError)
    # masterInterface.list_all_port()
    path = masterInterface.get_path_from_pid(SERIAL_PID)
    print("making connection to Master at %s" % path)
    masterInterface.connect_master(path)
    #force wait until data ready
    q1 = masterInterface.get_data()
    print("master online!")
    time.sleep(1)


    # Try to connect to VREP
    try:
        ######################### initialize vrep interface #########################
        vrep_interface = DQ_VrepInterface()
        if not vrep_interface.connect(VREP_ADDRESS, VREP_PORT, 100, 10):
            # If connection fails, disconnect and throw exception
            vrep_interface.disconnect_all()
            vrep_interface=None
            masterInterface.reset()
            eError.set()
            print("MasterLoop: Vrep Connection Failed")
            return

        umirobot_vrep = UMIRobotVrepRobot(vrep_interface=vrep_interface)
        ######################### initialize vrep interface #########################
        ######################### Get robot pose info       #########################
        umirobot_kinematics = umirobot_vrep.kinematics()
        q_init = umirobot_vrep.get_q_from_vrep()
        x_init = umirobot_kinematics.fkm(q_init)
        t_init = DQ.vec3(DQ.translation(x_init))
        # umirobot_vrep.show_xd_in_vrep(x_init)
        x_master_ref = vrep_interface.get_object_pose("x_master_ref")

        ######################### Get robot pose info       #########################

        ######################### Initalize Loop variable   #########################
        gripper_val = 0
        xd_t_offset = np.array([0,0,0], dtype=np.float32)
        xd_component = np.array([0, -0.16, 0, 0, 0, 0], dtype=np.float32)
        xd_component_history = xd_component.copy()

        masterAngle = np.zeros(5)

        last_time = time.time()
        loop_time_true = 0.001

        print("Loop Start...")
        while True:
            # Master ref to apply pose transform
            x_master_ref = vrep_interface.get_object_pose("x_master_ref")

            masterData = masterInterface.get_data()
            # if isinstance(q1[0], float):
            # print("gripper = %s"%str(masterData))

            ### POSE CONTROL
            #from angle value convert to angle
            for i in range(5):
                masterAngle[i] = convert_pot_to_radians(masterData[i])
            masterAngle = ANGLE_SIGN*(ANGLE_OFFSET + masterAngle)

            l1_x = LINK_L * math.sin(masterAngle[2])
            l1_y = LINK_L * math.cos(masterAngle[2])
            l2_x = LINK_L * math.sin(masterAngle[2]+masterAngle[3])
            l2_y = LINK_L * math.cos(masterAngle[2]+masterAngle[3])
            xd_component[2] = -(l1_x+l2_x)*T_XY_M_SCALING
            xd_component[1] = -(l1_y+l2_y)*T_XY_M_SCALING
            xd_component[0] = (masterAngle[4])*T_Z_M_SCALING
            xd_component[3] = -math.degrees(math.atan2(xd_component[1], xd_component[2]))-90
            xd_component[4] = math.degrees(masterAngle[0])*1.333
            xd_component[5] = math.degrees(masterAngle[1])*1.333

            ### GRIPPER CONTROL
            gripper_val += masterData[5]*loop_time_true*GRIPPER_ANGULAR_RATE
            #### UPDATE TO VREP
            xd_component_history = MOTION_RUNNING_WEIGHT*xd_component +\
                        (1-MOTION_RUNNING_WEIGHT)*xd_component_history
            _tmp = xd_component_history.copy()
            _tmp[0] += xd_t_offset[0]
            _tmp[1] += xd_t_offset[1]
            _tmp[2] += xd_t_offset[2]
            x_master = get_xd_from_trans_rot(*_tmp)

            xd = x_master_ref * x_master

            umirobot_vrep.show_xd_in_vrep(xd)
            umirobot_vrep.send_gripper_value_to_vrep(gripper_val)

            time.sleep(1.0/LOOP_RATE)
            _this_time=time.time()
            loop_time_true = _this_time-last_time
            last_time = _this_time


    except Exception as e:
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        print(exc_type, fname, exc_tb.tb_lineno)
        print("Exception caught: ", e)
    except KeyboardInterrupt:
        print("KeyboardInterrupt")

    # Disconnect from VREP
    vrep_interface.disconnect()
    masterInterface.reset()


def convert_pot_to_radians(val):
    return math.radians((val/1024.0*270)-135)


def get_xd_from_trans_rot(t_i, t_j, t_k, r_i, r_j, r_k):
    _t = [t_i, t_j, t_k]
    _t = DQ(_t)
    _ri = DQ([math.cos(math.radians(r_i)/2), math.sin(math.radians(r_i)/2), 0, 0])
    _rj = DQ([math.cos(math.radians(r_j)/2), 0, math.sin(math.radians(r_j)/2), 0])
    _rk = DQ([math.cos(math.radians(r_k)/2), 0, 0, math.sin(math.radians(r_k)/2)])
    _r=_ri*_rk*_rj
    return (_r+DQ.E*_t*_r*0.5)


if __name__ == '__main__':
    main()
