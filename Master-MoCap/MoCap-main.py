from dqrobotics import *
from scipy.spatial.transform import Rotation
import math
import numpy as np


from BallTracker import BallTracker
from ctypes import Structure, c_double, c_int
import multiprocessing as mp
from multiprocessing.sharedctypes import Array

from dqrobotics.interfaces.vrep import DQ_VrepInterface
from dqrobotics import *
from umirobot_vrep_robot import UMIRobotVrepRobot
from ardu_nano_interface import SerialMasterInterface

from sys import version_info
if version_info.major == 2:
	import Tkinter as tk
elif version_info.major == 3:
	import tkinter as tk
from functools import partial
import time
import os, sys


TEST_ONLY_UI = False
USE_PHYSICAL_SECONDARY_MASTER = False



def main():

	masterCommDataLock = mp.Lock()
	MasterCommDataArray = mp.sharedctypes.Array(MasterCommDataStruc, [MasterCommDataStruc()],
	lock=masterCommDataLock)
	eExit = mp.Event()
	eExit.clear()
	eError = mp.Event()


	masterSecondaryInterface = None
	if USE_PHYSICAL_SECONDARY_MASTER:
		    masterSecondaryInterface = SerialMasterInterface(eError)
		    # masterInterface.list_all_port()
		    path = masterSecondaryInterface.get_path_from_pid(SERIAL_PID)
		    print("making connection to Master at %s" % path)
		    masterSecondaryInterface.connect_master(path)
		    #force wait until data ready
		    q1 = masterSecondaryInterface.get_data()
		    print("master online!")

	if not TEST_ONLY_UI:
		##event for monitoring error
		##start the Master loop
		masterMainLoopHandler = mp.Process(
		    target=master_loop,
		    args=[MasterCommDataArray, eExit, eError, masterSecondaryInterface]
		)
		masterMainLoopHandler.start()

	#start UI
	UIApp=App(tk.Tk(), MasterCommDataArray, eError)

	#safe exit the master control loop
	eExit.set()
	if not TEST_ONLY_UI:
		masterMainLoopHandler.join(10)
		if masterMainLoopHandler.is_alive():
			print("Force Terminating master loop")
			masterMainLoopHandler.terminate()
	exit()

class App():
	def __init__(self, masterWindow, MasterCommDataArray,eError):

		#Store Ssm instance
		self.MasterCommDataArray = MasterCommDataArray
		self.eError = eError

		self.masterWindow=masterWindow
		#initialize TK
		self.masterWindow.geometry("700x500")
		self.masterWindow.title("Motion Capture Master Interface UI")


		sliderFrame = tk.Frame(masterWindow, highlightthickness = 1,
		            highlightbackground="black", width=510, height=150)
		sliderFrame.grid(row=0, column=0)
		sliderFrame.grid_propagate(0)

		gripperSliderTxt = tk.Label(sliderFrame, text="Gripper Value")
		gripperSliderTxt.grid(row=0, column=0)
		self.gripperSlider = tk.Scale(sliderFrame, from_=-90, to=90,
		            orient=tk.HORIZONTAL, length=500, resolution=1)
		self.gripperSlider.set(0)
		self.gripperSlider.config(command=self.gripper_slider_callback)
		self.gripperSlider.grid(row=1, column=0)

		xdRotSliderTxt = tk.Label(sliderFrame, text="Xd rot Value")
		xdRotSliderTxt.grid(row=2, column=0)
		self.xdRotSlider = tk.Scale(sliderFrame, from_=0, to=360,
		            orient=tk.HORIZONTAL, length=500, resolution=1)
		self.xdRotSlider.set(0)
		self.xdRotSlider.config(command=self.xdrot_slider_callback)
		self.xdRotSlider.grid(row=3, column=0)


		######xd override Frame
		xdControlFrame = tk.Frame(masterWindow, highlightthickness = 1,
		            highlightbackground="black",width=510, height=270)
		xdControlFrame.grid(row=1, column=0)
		xdControlFrame.grid_propagate(0)
		self.overrideXdFromUIVar = tk.IntVar()
		overrideXdFromUICkbox = tk.Checkbutton(xdControlFrame, text="override xd control from UI",
					command=self.toggle_xd_override, variable=self.overrideXdFromUIVar)
		overrideXdFromUICkbox.grid(row=0, column=0, columnspan=2)
		xdControlFrameUnitTxt = tk.Label(xdControlFrame, text="Translation: cm, Rotation: deg")
		xdControlFrameUnitTxt.grid(row=1, column=0, columnspan=2)
		#"R_j" is controlled in rot
		self.xdOvrdTxt_list = ["T_i", "T_j", "T_k", "R_i", "R_k"]
		self.xdOvrdSliderTxt = [None for i in range(len(self.xdOvrdTxt_list))]
		self.xdOvrdSlider = [None for i in range(len(self.xdOvrdTxt_list))]
		for i in range(len(self.xdOvrdTxt_list)):
			self.xdOvrdSliderTxt[i] = tk.Label(xdControlFrame, width=10,
						text="%s: 0"%self.xdOvrdTxt_list[i], anchor='w')
			self.xdOvrdSliderTxt[i].grid(row=i+2, column=0)
			#translation:rotation
			_from = -180
			_to   = 180
			if i<=2:
				_from = -30
				_to = 30
			self.xdOvrdSlider[i] = tk.Scale(xdControlFrame, from_=_from, to=_to,
			            orient=tk.HORIZONTAL, length=500, resolution=0.01, state='disable')
			self.xdOvrdSlider[i].set(0)
			self.xdOvrdSlider[i].config(command=partial(self.xd_override_slider_callback, i))
			self.xdOvrdSlider[i].grid(row=i+2, column=1)

		##Disable frame according to parameter
		if USE_PHYSICAL_SECONDARY_MASTER:
			for child in sliderFrame.winfo_children():
			    child.configure(state='disable')

		#on on_closing
		masterWindow.protocol("WM_DELETE_WINDOW", self.on_closing)
		#set tk after updating loop
		self.masterWindow.after(50, self.ui_after_loop)
		#start tkloop
		self.masterWindow.mainloop()


	def gripper_slider_callback(self, val):
		self.MasterCommDataArray[0].gripper = float(val)
		# print("Set gripper val: %.3f"%float(val))
	def xdrot_slider_callback(self, val):
		self.MasterCommDataArray[0].rot = float(val)
		# print("Set gripper val: %.3f"%float(val))

	def toggle_xd_override(self):
		if self.overrideXdFromUIVar.get() == 1:
			print("Enable xd Override")
			self.MasterCommDataArray[0].xdovrd = 1
			for i in range(len(self.xdOvrdTxt_list)):
				i_d = i
				if i_d == 4: i_d = 5
				self.xdOvrdSlider[i].config(state='normal')
				self.xdOvrdSlider[i].set(self.MasterCommDataArray[0].xd[i_d])
		else:
			print("Diable xd Override")
			self.MasterCommDataArray[0].xdovrd = 0
			for i in range(len(self.xdOvrdTxt_list)):
				self.xdOvrdSlider[i].config(state='disable')

	def xd_override_slider_callback(self, i, val):
		if i == 4: i = 5
		self.MasterCommDataArray[0].xd[i] = float(val)

	def ui_after_loop(self):

		#check for error state
		if self.eError.is_set():
			self.on_closing()

		for i in range(len(self.xdOvrdTxt_list)):
			i_d = i
			if i_d == 4: i_d = 5
			self.xdOvrdSliderTxt[i].config(
						text="%s: %.1f"%(self.xdOvrdTxt_list[i],
						self.MasterCommDataArray[0].xd[i_d]))

		self.masterWindow.after(50, self.ui_after_loop)

	def on_closing(self):
		self.masterWindow.destroy()


		##xd use unit of cm and deg
class MasterCommDataStruc(Structure):
    _fields_ = [('xd', c_double*6), ('xd_DQ', c_double*8),
				('rot', c_double), ('gripper', c_double),
				('xdovrd', c_int)]


def get_xd_from_trans_rot(t_i, t_j, t_k, r_i, r_j, r_k):
	_t = [t_i, t_j, t_k]
	_t = DQ(_t)
	_ri = DQ([math.cos(math.radians(r_i)/2), math.sin(math.radians(r_i)/2), 0, 0])
	_rj = DQ([math.cos(math.radians(r_j)/2), 0, math.sin(math.radians(r_j)/2), 0])
	_rk = DQ([math.cos(math.radians(r_k)/2), 0, 0, math.sin(math.radians(r_k)/2)])
	_r=_ri*_rj*_rk
	return (_r+DQ.E*_t*_r*0.5)

def master_loop(MasterCommDataArray, eExit, eError, serialInterface):

	print("Entered master for testing")
	if serialInterface is None:
		USE_SECONDARY = False
	else:
		USE_SECONDARY = True

	try:
		vrep_interface = DQ_VrepInterface()
		if not vrep_interface.connect(20001, 100, 10):
			# If connection fails, disconnect and throw exception
			vrep_interface.disconnect_all()
			eError.set()
			print("MasterLoop: Vrep Connection Failed")
			return
		umirobot_vrep = UMIRobotVrepRobot(vrep_interface=vrep_interface)
		umirobot_kinematics = umirobot_vrep.kinematics()
		q_init = umirobot_vrep.get_q_from_vrep()
		x_init = umirobot_kinematics.fkm(q_init)
		umirobot_vrep.show_xd_in_vrep(x_init)
		x_master_ref = vrep_interface.get_object_pose("x_master_ref")
		xd_t_offset = [0,-0.16,0]


		gripper_val = 0
		xd=get_xd_from_trans_rot(xd_t_offset[0], xd_t_offset[1],
									xd_t_offset[2], 0, 0, 0)
		while not eExit.is_set() and not eError.is_set():
			# print("gripper val: %.3f"%MasterCommDataArray[0].gripper)
			time.sleep(0.1)

			if USE_SECONDARY:
				secMasterData = serialInterface.get_data()
				gripper_val = math.radians(secMasterData[0]/1024.0*180-90)/2
			else:
				gripper_val = math.radians(MasterCommDataArray[0].gripper)


			if MasterCommDataArray[0].xdovrd == 1:
				# print("MasterLoop: Enable Override: ", end="")
				vv=[MasterCommDataArray[0].xd[i] for i in range(6)]
				vv[4] = MasterCommDataArray[0].rot
				#calculate xd from translation and rotation
				_t = xd_t_offset.copy()
				_t[0] = _t[0]+vv[0]/100
				_t[1] = _t[1]+vv[1]/100
				_t[2] = _t[2]+vv[2]/100
				_t = DQ(_t)
				_ri = DQ([math.cos(math.radians(vv[3])/2), math.sin(math.radians(vv[3])/2), 0, 0])
				_rj = DQ([math.cos(math.radians(vv[4])/2), 0, math.sin(math.radians(vv[4])/2), 0])
				_rk = DQ([math.cos(math.radians(vv[5])/2), 0, 0, math.sin(math.radians(vv[5])/2)])
				_r=_ri*_rj*_rk
				xd = get_xd_from_trans_rot(xd_t_offset[0]+vv[0]/100, xd_t_offset[1]+vv[1]/100,
											xd_t_offset[2]+vv[2]/100, vv[3], vv[4], vv[5])
				# print(vv)



			###write to vrep
			umirobot_vrep.show_xd_in_vrep(x_master_ref*xd)
			umirobot_vrep.send_gripper_value_to_vrep(gripper_val)


	except Exception as e:
		exc_type, exc_obj, exc_tb = sys.exc_info()
		fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
		print(exc_type, fname, exc_tb.tb_lineno)
		print(e)

	if USE_SECONDARY:
		serialInterface.reset()
	return





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
	while not eExit.is_set():
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



if __name__ == '__main__':
    main()
