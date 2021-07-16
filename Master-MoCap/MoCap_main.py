from sys import version_info
if version_info.major == 2:
    import Tkinter as tk
elif version_info.major == 3:
    import tkinter as tk
from functools import partial
import time
import os, sys

#Import Variable defined in loop
from MoCap_loop import *

TEST_ONLY_UI = False
USE_PHYSICAL_SECONDARY_MASTER = False
VREP_ADDRESS = '192.168.10.102'

############################## program parameter ##############################
BACKEND='cpp'
recvPort = 3344 #+1 will be taken also by frontend
sendPort = 2345 #+1 will be taken also by frontend
############################## program parameter ##############################

############################## Target Setup ##############################
#in format of Lower H, S, V
#             Upper H, S, V
DEFAULT_greenLimit = [[71, 59, 31], [108, 255, 192]]
DEFAULT_redLimit = [[111, 152, 175], [184, 255, 255]]
INTEPOLER_FILE_NAME = "./camera_config/lens_map.sciobj"
############################## Target Setup ##############################
############################## Frame Setup ##############################
#     initialize: camPV_DQ[0,1] //DQ transformation of camera in space
L=55*0.5/100
CAM_r1=DQ([math.cos(math.pi/4),0,0,math.sin(math.pi/4)])\
        *DQ([math.cos(3*math.pi/8),math.sin(3*math.pi/8),0,0])
CAM_t1=DQ([0, 0, L, 2*L])
CAM1_POV = CAM_r1+E_*CAM_t1*CAM_r1*0.5
CAM_r2=DQ([math.cos(3*math.pi/8),math.sin(3*math.pi/8),0,0])
CAM_t2=DQ([0, L , 2*L,  2*L])
CAM2_POV = CAM_r2+E_*CAM_t2*CAM_r2*0.5
# print(CAM1_POV)
# print(CAM2_POV)
############################## Frame Setup ##############################
def main():
    masterCommDataLock = mp.Lock()
    MasterCommDataArray = mp.sharedctypes.Array(MasterCommDataStruc, [MasterCommDataStruc()],lock=masterCommDataLock)
    eExit = mp.Event()
    eExit.clear()
    eError = mp.Event()
    ############################## Initialise tracking setup ##############################
    ##Initializeing Camera Tracker Setting
    #     initialize: hCam[0] = BallTracker(0)
    #     initialize: hCam[1] = BallTracker(1)
    tracker = [None, None]
    eExitTracker1 = mp.Event()
    tracker[0]=BallTracker(0, eExitTracker1, recvPort, sendPort, backend=BACKEND)
    tracker[1]=BallTracker(1, eError, recvPort-1, sendPort-1, backend=BACKEND)

    if not tracker[0].begin_capture():
        print("MasterMain: Error: with openCV")
        tracker[0].exit()
        return
    if not tracker[1].begin_capture():
        print("MasterMain: Error: with openCV")
        tracker[0].exit()
        tracker[1].exit()
        return

    tracker[0].set_mask_color(DEFAULT_greenLimit, DEFAULT_redLimit)
    tracker[1].set_mask_color(DEFAULT_greenLimit, DEFAULT_redLimit)
    print("MasterMain: Frame Size: %d x %d"%tracker[0].get_frame_size())
    tracker[0].set_lens_mapping(INTEPOLER_FILE_NAME)
    tracker[1].set_lens_mapping(INTEPOLER_FILE_NAME)

    #     //initialize processing of first frame before enter loop
    #     FOR i in 0,1: //camera
    #         hCam[i].set_next_frame()
    #     ENDFOR
    print("MasterMain: Holding start")
    # time.sleep(2)
    print("MasterMain: sending start signal")
    tracker[0].set_next_frame()
    tracker[1].set_next_frame()

    count=0
    while eError.is_set():
        time.sleep(0.5)
        count+=1
        if count>100:
            print("MasterMain: Waiting for camera time out")
            tracker[0].exit()
            tracker[1].exit()
            return
    ############################## Initialise tracking setup ##############################

    masterSecondaryInterface = None
    if USE_PHYSICAL_SECONDARY_MASTER:
        masterSecondaryInterface = SerialMasterInterface(eError)
        # masterInterface.list_all_port()
        path = masterSecondaryInterface.get_path_from_pid(SERIAL_PID)
        print("MasterMain: making connection to Master at %s" % path)
        masterSecondaryInterface.connect_master(path)
        #force wait until data ready
        q1 = masterSecondaryInterface.get_data()
        print("MasterMain: master online!")

    if not TEST_ONLY_UI:
        ##event for monitoring error
        ##start the Master loop
        masterMainLoopHandler = mp.Process(
            target=master_loop,
            args=[MasterCommDataArray, eExit, eError, tracker, masterSecondaryInterface]
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
                    command=self.toggle_xd_override_callback, variable=self.overrideXdFromUIVar)
        overrideXdFromUICkbox.grid(row=0, column=0, columnspan=2)
        xdControlFrameUnitTxt = tk.Label(xdControlFrame, text="Translation: cm, Rotation: deg")
        xdControlFrameUnitTxt.grid(row=1, column=0, columnspan=2)
        #"R_j" is controlled in rot
        self.xdOvrdTxt_list = ["T_i", "T_j", "T_k", "R_i", "R_k"]
        self.xdOvrdSliderTxt = [None for i in range(len(self.xdOvrdTxt_list))]
        self.xdOvrdSlider = [None for i in range(len(self.xdOvrdTxt_list))]
        self.xdOvrdSliderVar = [None for i in range(len(self.xdOvrdTxt_list))]
        for i in range(len(self.xdOvrdTxt_list)):
            self.xdOvrdSliderTxt[i] = tk.Label(xdControlFrame, width=8,
                        text="%s: 0"%self.xdOvrdTxt_list[i], anchor='w')
            self.xdOvrdSliderTxt[i].grid(row=i+2, column=0)
            #translation:rotation
            _from = -180
            _to   = 180
            if i<=2:
                _from = -30
                _to = 30
            self.xdOvrdSliderVar[i] = tk.DoubleVar()
            self.xdOvrdSlider[i] = tk.Scale(xdControlFrame, from_=_from, to=_to,
                        orient=tk.HORIZONTAL, length=430, resolution=0.01,
                        variable=self.xdOvrdSliderVar[i], state='disable')
            self.xdOvrdSlider[i].set(0)
            self.xdOvrdSlider[i].config(command=partial(self.xd_override_slider_callback, i))
            self.xdOvrdSlider[i].grid(row=i+2, column=1)

        extraCtlFrame = tk.Frame(masterWindow, highlightthickness = 1,
                    highlightbackground="black", width=150, height=150)
        extraCtlFrame.grid(row=0, column=1)
        extraCtlFrame.grid_propagate(0)
        self.mocapOnCkboxVar = tk.IntVar()
        mocapOnCkbox = tk.Checkbutton(extraCtlFrame, text="Enable Motion Capture",
                    variable=self.mocapOnCkboxVar, command=self.mocap_on_callback)
        mocapOnCkbox.grid(row=0, column=0)
        self.excutePostZero = False
        self.offsetZeroBtn = tk.Button(extraCtlFrame, text="Zero offset",
                    bg='white', command=self.toggle_zero_callback)
        self.offsetZeroBtn.grid(row=1, column=0)


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

    def toggle_xd_override_callback(self):
        if self.overrideXdFromUIVar.get() == 1:
            print("Enable xd Override")
            self.MasterCommDataArray[0].xdovrd = 1
            for i in range(len(self.xdOvrdTxt_list)):
                i_d = i
                if i_d == 4: i_d = 5
                self.xdOvrdSlider[i].config(state='normal')
                self.xdOvrdSliderVar[i].set(self.MasterCommDataArray[0].xd[i_d])
            self.mocapOnCkboxVar.set(0)
            self.MasterCommDataArray[0].master_on = self.mocapOnCkboxVar.get()
        else:
            print("Diable xd Override")
            self.MasterCommDataArray[0].xdovrd = 0
            for i in range(len(self.xdOvrdTxt_list)):
                self.xdOvrdSlider[i].config(state='disable')

    def xd_override_slider_callback(self, i, val):
        if i == 4: i = 5
        self.MasterCommDataArray[0].xd[i] = float(val)

    def toggle_zero_callback(self):
        self.offsetZeroBtn.config(bg='green')
        self.MasterCommDataArray[0].zero_offset = 1
        self.excutePostZero = True

    def mocap_on_callback(self):
        if self.overrideXdFromUIVar.get() == 1:
             print("UI:: Unable to set mocap on, override is on")
             self.mocapOnCkboxVar.set(0)
        self.MasterCommDataArray[0].master_on = self.mocapOnCkboxVar.get()
        if self.mocapOnCkboxVar.get():
            print("Set mocap to on and zero offset")
            self.toggle_zero_callback()


    def ui_after_loop(self):

        #check for error state
        if self.eError.is_set():
            self.on_closing()
            return
        #update override information
        for i in range(len(self.xdOvrdTxt_list)):
            i_d = i
            if i_d == 4: i_d = 5
            self.xdOvrdSliderTxt[i].config(
                        text="%s: %.1f"%(self.xdOvrdTxt_list[i],
                        self.MasterCommDataArray[0].xd[i_d]))
        #update extra control
        if self.excutePostZero and not self.MasterCommDataArray[0].zero_offset:
            self.offsetZeroBtn.config(bg='white')
            for i in range(len(self.xdOvrdTxt_list)):
                self.xdOvrdSliderVar[i].set(0)
            self.excutePostZero = False

        self.masterWindow.after(50, self.ui_after_loop)

    def on_closing(self):
        self.masterWindow.destroy()

if __name__ == '__main__':
    main()
