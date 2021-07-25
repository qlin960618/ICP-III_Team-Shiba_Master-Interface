"""
Copyright (C) 2020 Murilo Marques Marinho (www.murilomarinho.info)
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not,
see <https://www.gnu.org/licenses/>.
"""
import numpy as np
from dqrobotics import *
from math import *

from umirobot.shared_memory import UMIRobotSharedMemoryReceiver
from umirobot_task_space_controller import UMIRobotTaskSpaceController
from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics.utils.DQ_Math import rad2deg
from umirobot_vrep_robot import UMIRobotVrepRobot


def master_control_two(pot1, pot2, pot3, pot4, pot5):
    """ This function takes the three potentiometer values from the master interface 2 of the
    unparalleled shibas and calculates the pose transformation of the end effector of the master robot.
    Using that it calculates and returns the unit DQ pose transformation of the target."""
    # Change these values to scale the mapping how you like
    # master 2 lengths
    l1 = 150 / 1000
    l2 = 150 / 1000
    # linear conversion to rad variables
    m = 1.012290966123
    c = -2.530727415
    # linear conversion to z-axis translation
    c2 = -165 / 1000
    m2 = 66 / 1000
    # Convert pot values to approximate angles in radians

    theta1 = m * pot1 + c
    theta2 = -m * pot2 - c
    theta4 = m * pot4 + c
    theta5 = m * pot5 + c

    # Convert pot3 for vertical translation
    theta3 = m2 * pot3 + c2
    # Find the rotation transformation DQs
    r1 = cos(theta1 / 2) + sin(theta1 / 2) * i_
    r2 = cos(theta2 / 2) + sin(theta2 / 2) * i_
    r4 = cos(theta4 / 2) + sin(theta4 / 2) * j_
    r5 = cos(theta5 / 2) + sin(theta5 / 2) * k_
    # Find the translation DQs
    t1 = 1 + 0.5 * E_ * (l1 * -j_)
    t2 = 1 + 0.5 * E_ * (l2 * -j_)
    t3 = 1 + 0.5 * E_ * (theta3 * i_)
    # Find the total pose transformation DQs
    x_t = 1 * r1 * t1 * r2 * t2 * t3 * conj(r1) * conj(r2) * r4 * r5
    # initial pose dq

    return x_t


def gripper_calc(pot6):
    "A function to convert the 0-5 V potentiometer values to integer angles in degrees."
    m3 = 0.4
    c3 = -1
    gripper_qd = int(m3 * pot6 + c3)

    return gripper_qd


configuration = {
    "controller_gain": 4.0,
    "damping": 0.01,
    "alpha": 0.999,  # Soft priority between translation and rotation [0,1] ~1 Translation, ~0 Rotation
    "use_real_umirobot": True,
    "umirobot_port": "COM3"
}


def control_loop(umirobot_smr, cfg):
    # Instantiate a DQ_VrepInterface
    vrep_interface = DQ_VrepInterface()

    try:
        # Try to connect to robot
        if cfg["use_real_umirobot"]:
            umirobot_smr.send_port(cfg["umirobot_port"])
            print("main::Trying to connect to umirobot at port={}.".format(cfg["umirobot_port"]))

        # Try to connect to VREP
        if not vrep_interface.connect(20000, 100, 10):
            # If connection fails, disconnect and throw exception
            vrep_interface.disconnect_all()
            raise Exception("Unable to connect to VREP.")

        # This object is used to communicate more easily with VREP
        umirobot_vrep = UMIRobotVrepRobot(vrep_interface=vrep_interface)

        # This is a DQ_SerialManipulatorDH instance. Used to calculate kinematics of the UMIRobot.
        umirobot_kinematics = umirobot_vrep.kinematics()

        # UMIRobot Task Space Controler
        umirobot_controller = UMIRobotTaskSpaceController(kinematics=umirobot_kinematics)
        umirobot_controller.set_gain(cfg["controller_gain"])
        umirobot_controller.set_damping(cfg["damping"])
        umirobot_controller.set_alpha(cfg["alpha"])

        # Initialize the objects in VREP to reflect what we calculate using DQRobotics
        q_init = umirobot_vrep.get_q_from_vrep()
        x_init = umirobot_kinematics.fkm(q_init)
        umirobot_vrep.show_x_in_vrep(x_init)
        umirobot_vrep.show_xd_in_vrep(x_init)

        # Loop parameters
        sampling_time = 0.008

        # Control loop (We're going to control it open loop, because that is how we operate the real robot)
        # Initialize q with its initial value
        q = q_init
        x_master_ref = vrep_interface.get_object_pose("x_master_ref")

        umirobot_smr.send_port('COM3')
        while True:
            # Change how you calculate xd
            q1 = umirobot_smr.get_potentiometer_values()
            if isinstance(q1[0], float):
                x_master = master_control_two(q1[0], q1[1], q1[2], q1[3], q1[4])
                gripper_qd = gripper_calc(q1[5])
            else:
                x_master = x_master_ref
                gripper_qd = 0

            # xd = umirobot_vrep.get_xd_from_vrep()
            xd = x_master_ref * x_master
            # umirobot_vrep.send_gripper_value_to_vrep(gripper_qd)

            # Solve the quadratic program
            u = umirobot_controller.compute_setpoint_control_signal(q, xd)

            # Update the current joint positions
            q = q + u * sampling_time
            # Manually change last 3 joints

            # Update vrep with the new information we have
            umirobot_vrep.send_q_to_vrep(q)
            umirobot_vrep.show_x_in_vrep(umirobot_controller.get_last_robot_pose())
            umirobot_vrep.show_xd_in_vrep(xd)

            # Update real robot if needed
            if cfg["use_real_umirobot"]:
                if umirobot_smr.is_open():
                    # Get the desired gripper value from VREP
                    # gripper_value_d = umirobot_vrep.get_gripper_value_from_vrep()
                    # gripper_value_d = -20
                    # Control the gripper somehow
                    q_temp = np.hstack((q, gripper_calc(q1[5])))
                    # The joint information has to be sent to the robot as a list of integers
                    umirobot_smr.send_qd(rad2deg(q_temp).astype(int).tolist())
                else:
                    raise Exception("UMIRobot port not opened at {}.".format(cfg["umirobot_port"]))
            else:
                if umirobot_smr.is_open():
                    umirobot_smr.send_qd([0,0,0,0,0,0])

    except Exception as e:
        print("Exception caught: ", e)
    except KeyboardInterrupt:
        print("KeyboardInterrupt")

    # Disconnect from VREP
    vrep_interface.disconnect()


def run(shared_memory_info, lock):
    umirobot_smr = UMIRobotSharedMemoryReceiver(shared_memory_info, lock)

    try:
        control_loop(umirobot_smr, cfg=configuration)
    except Exception as e:
        print("umirobot_task_space_control::run::Error::" + str(e))
    except KeyboardInterrupt:
        print("umirobot_task_space_control::run::Info::Interrupted by user.")

    umirobot_smr.send_shutdown_flag(True)
