from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics.robot_modeling import DQ_Kinematics
from umirobot_task_space_control.umirobot_vrep_robot import UMIRobotVrepRobot
from dqrobotics import *
from dqrobotics.solvers import DQ_QuadprogSolver

import numpy as np
from umirobot import UMIRobot
from math import *

# Change these values to scale the mapping how you like
# master 2 lengths
l1 = 150 / 1000
l2 = 150 / 1000
# linear conversion to rad variables
m = 1.012290966123
c = -2.530727415
# linear conversion to z-axis translation
c2 = -165 / 1000
m2 = 66/1000


def master_control_two(pot1, pot2, pot3):
    """ This function takes the three potentiometer values from the master interface 2 of the
    unparalleled shibas and calculates the pose transformation of the end effector of the master robot.
    Using that it calculates and returns the unit DQ pose transformation of the target."""

    # Convert pot values to approximate angles in radians
    theta1 = m * pot1 + c
    theta2 = -m * pot2 - c
    # Convert pot3 for vertical translation
    theta3 = m2 * pot3 + c2
    # Find the rotation transformation DQs
    r1 = cos(theta1 / 2) + sin(theta1 / 2) * i_
    r2 = cos(theta2 / 2) + sin(theta2 / 2) * i_
    # Find the translation DQs
    t1 = 1 + 0.5 * E_ * (l1 * -j_)
    t2 = 1 + 0.5 * E_ * (l2 * -j_)
    t3 = 1 + 0.5 * E_ * (theta3 * i_)
    # Find the total pose transformation DQs
    x_t = 1 * r1 * t1 * r2 * t2 * t3
    # initial pose dq

    return x_t


def _get_rotation_error(x, xd):
    # Calculate error from invariant
    error_1 = vec4(rotation(x) - rotation(xd))
    error_2 = vec4(rotation(x) + rotation(xd))

    # Calculate 'distance' from invariant
    norm_1 = np.linalg.norm(error_1)
    norm_2 = np.linalg.norm(error_2)

    # Check the closest invariant and return the proper error
    if norm_1 < norm_2:
        return error_1
    else:
        return error_2


# Instantiate a DQ_VrepInterface
vrep_interface = DQ_VrepInterface()

try:
    # Try to connect to VREP
    if not vrep_interface.connect(20001, 100, 10):
        # If connection fails, disconnect and throw exception
        vrep_interface.disconnect_all()
        raise Exception("Unable to connect to VREP.")

    # This object is used to communicate more easily with VREP
    umirobot_vrep = UMIRobotVrepRobot(vrep_interface=vrep_interface)

    # This is a DQ_SerialManipulatorDH instance. Used to calculate kinematics of the UMIRobot.
    umirobot_kinematics = umirobot_vrep.kinematics()

    # QP Solver
    qp_solver = DQ_QuadprogSolver()

    # Initialize the objects in VREP to reflect what we calculate using DQRobotics
    q_init = umirobot_vrep.get_q_from_vrep()
    x_init = umirobot_kinematics.fkm(q_init)
    umirobot_vrep.show_x_in_vrep(x_init)
    umirobot_vrep.show_xd_in_vrep(x_init)

    # Controller parameters (Change these to see behavior if desired)
    eta = 4
    damping = 0.01
    sampling_time = 0.008
    alpha = 0.999  # close to 1 means translation is prioritised

    # Control loop (We're going to control it open loop, because that is how we operate the real robot)
    # Initialize q with its initial value
    q = q_init
    motion_scaling = 1.75

    if __name__ == '__main__':
        with UMIRobot() as umirobot:
            umirobot.set_port('COM3')
            while True:
                try:
                    # Master ref to apply pose transform
                    x_master_ref = vrep_interface.get_object_pose("x_master_ref")

                    q1 = umirobot.get_potentiometer_values()
                    if isinstance(q1[0], float):
                        x_master = master_control_two(q1[0], q1[1], q1[2])
                    umirobot.set_qd([10, 10, 10, 10, 10, 10])
                    umirobot.update()

                    # Get current pose information
                    x = umirobot_kinematics.fkm(q)
                    # xd = umirobot_vrep.get_xd_from_vrep()
                    # Motion scaling transformation
                    """ r_master = rotation(x_master)
                    t_master = translation(x_master)
                    tx_master = motion_scaling * t_master.q[1]
                    ty_master = motion_scaling * t_master.q[2]
                    tz_master = t_master.q[3]
                    t_master_ms = tx_master * i_ * ty_master * j_ * tz_master * k_

                    x_master_ms = r_master + 0.5 * E_ * t_master_ms * r_master"""

                    xd = x_master_ref * x_master

                    # Calculate errors
                    et = vec4(translation(x) - translation(xd))
                    er = _get_rotation_error(x, xd)

                    # Get the Translation Jacobian and Rotation Jacobian
                    Jx = umirobot_kinematics.pose_jacobian(q)
                    Jr = DQ_Kinematics.rotation_jacobian(Jx)
                    Jt = DQ_Kinematics.translation_jacobian(Jx, x)

                    # ++++++++++++++++Calculate the control step according to++++++++++++++++
                    # "A Unified Framework for the Teleoperation of Surgical Robots in Constrained Workspaces".
                    # Marinho, M. M; et al.
                    # In 2019 IEEE International Conference on Robotics and Automation (ICRA), pages 2721–2727, May 2019. IEEE
                    # http://doi.org/10.1109/ICRA.2019.8794363

                    # Translation term
                    Ht = Jt.transpose() @ Jt + np.eye(6, 6) * damping
                    ft = eta * Jt.transpose() @ et

                    # Rotation term
                    Hr = Jr.transpose() @ Jr + np.eye(6, 6) * damping
                    fr = eta * Jr.transpose() @ er

                    # Combine terms using the soft priority
                    H = alpha * Ht + (1.0 - alpha) * Hr
                    f = alpha * ft + (1.0 - alpha) * fr

                    # Joint (position) limit constraints
                    lower_joint_limits = -np.pi / 2.0 * (np.ones((6,)))
                    upper_joint_limits = np.pi / 2.0 * (np.ones((6,)))
                    W_jl = np.vstack((-1.0 * np.eye(6, 6), np.eye(6, 6)))
                    w_jl = np.hstack((-1.0 * (lower_joint_limits - q), 1.0 * (upper_joint_limits - q)))

                    # Solve the quadratic program
                    u = qp_solver.solve_quadratic_program(H, f, W_jl, w_jl, np.zeros((1, 6)), np.zeros(1))

                    # ++++++++++++++++End of the control calculation++++++++++++++++

                    # Update the current joint positions
                    q = q + u * sampling_time

                    # Update vrep with the new information we have
                    umirobot_vrep.send_q_to_vrep(q)
                    umirobot_vrep.show_x_in_vrep(x)
                    umirobot_vrep.show_xd_in_vrep(xd)
                except Exception as e:
                    print("umirobot_test::Error::" + str(e))
                except KeyboardInterrupt:
                    print("umirobot_test::Info::Execution ended by user.")
                    print("Move the robot back to 0 degrees in each joint.")
                    umirobot.set_qd([0, 0, 0, 0, 0, 0])
                    umirobot.update()
                    break

except Exception as e:
    print("Exception caught: ", e)
except KeyboardInterrupt:
    print("KeyboardInterrupt")

# Disconnect from VREP
vrep_interface.disconnect()
