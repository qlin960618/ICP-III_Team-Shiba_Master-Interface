# import the necessary packages

import numpy as np
import multiprocessing as mp
import time
import timeit
import os
import socket
import subprocess
import select
try:
    import cPickle as pickle
except ImportError:
    import pickle

from dqrobotics import *
from scipy.spatial.transform import Rotation


DEFAULT_FRAME_HEIGHT = 720
DEFAULT_FRAME_WIDTH = 1280
SHOW_REALTIME = True



INITIALIZATION_TIMEOUT = 10
CIRCLE_R_THRESHOLD = 10

def main():

    #in format of Lower H, S, V
    #             Upper H, S, V
    DEFAULT_greenLimit = [[66, 179, 101], [101, 255, 255]]
    DEFAULT_redLimit = [[118, 95, 116], [189, 255, 255]]

    INTERPOLORX_FILE_NAME = "./camera_config/lens_mapx.sciobj"
    INTERPOLORY_FILE_NAME = "./camera_config/lens_mapy.sciobj"
    ##### Determined Using the ColorRanger Script

    ##event for monitoring error
    eError = mp.Event()

    tracker1=BallTracker(0, eError)
    if not tracker1.begin_capture():
        print("Error: with openCV")
        tracker1.exit()
        exit()

    tracker1.set_mask_color(DEFAULT_greenLimit, DEFAULT_redLimit)
    print("Frame Size: %d x %d"%tracker1.get_frame_size())
    tracker1.set_lens_mapping(INTERPOLORX_FILE_NAME, INTERPOLORY_FILE_NAME)

    print("Holding start")
    time.sleep(2)
    print("sending start signal")
    tracker1.set_next_frame()

    for i in range(1000):
        if not tracker1.frame_ready(10):
            print("getting Frame Timed out")
            tracker1.exit()
            exit()
        tracker1.set_next_frame()
        # pos = tracker1.get_ball_angle(1)
        # print("cnt: %d -- x: %.5f, y: %.5f"%(i, pos[1], pos[2]))
        pos = tracker1.get_ball_dq_origin(1)
        print("cnt: %d -- DQ: %s"%(i, str(pos[1])))
    tracker1.exit()
def main_2instants():

    #in format of Lower H, S, V
    #             Upper H, S, V
    DEFAULT_greenLimit = [[66, 179, 101], [101, 255, 255]]
    DEFAULT_redLimit = [[118, 95, 116], [189, 255, 255]]

    INTERPOLORX_FILE_NAME = "./camera_config/lens_mapx.sciobj"
    INTERPOLORY_FILE_NAME = "./camera_config/lens_mapy.sciobj"
    ##### Determined Using the ColorRanger Script

    ##event for monitoring error
    eError = mp.Event()

    tracker1=BallTracker(0, eError)
    tracker2=BallTracker(1, eError)

    print("Holding start")
    time.sleep(2)
    print("sending start signal")

    if not tracker1.begin_capture() or not tracker2.begin_capture():
        print("Error: with openCV")
        tracker1.exit()
        tracker2.exit()
        exit()

    tracker1.set_mask_color(DEFAULT_greenLimit, DEFAULT_redLimit)
    tracker2.set_mask_color(DEFAULT_greenLimit, DEFAULT_redLimit)
    print("Frame Size: %d x %d"%tracker1.get_frame_size())
    tracker1.set_lens_mapping(INTERPOLORX_FILE_NAME, INTERPOLORY_FILE_NAME)
    tracker2.set_lens_mapping(INTERPOLORX_FILE_NAME, INTERPOLORY_FILE_NAME)



    start_t = timeit.default_timer()
    for i in range(1000):
        tracker1.set_next_frame()
        tracker2.set_next_frame()
        if not tracker1.frame_ready(10) or not tracker2.frame_ready(10) or eError.is_set():
            print("getting Frame Timed out")
            tracker1.exit()
            tracker2.exit()
            exit()
        # pos = tracker1.get_ball_angle(1)
        # print("cnt: %d -- x: %.5f, y: %.5f"%(i, pos[1], pos[2]))
        pos1_1 = tracker1.get_ball_dq_origin(1)
        pos2_1 = tracker2.get_ball_dq_origin(1)
        pos1_0 = tracker1.get_ball_dq_origin(0)
        pos2_0 = tracker2.get_ball_dq_origin(0)

        end_t = timeit.default_timer()
        print("Program Rate: %.3f"%(1/(end_t-start_t)))
        start_t=end_t

        # print("cnt: %d -- DQ: %s | DQ: %s"%(i, str(pos1[1]), str(pos2[1])))
    tracker1.exit()
    tracker2.exit()




class BallTracker():


    def __init__(self, cameraID, eError):

        self.sendToIP="localhost"

        self.cameraID = cameraID
        self.eError = eError
        self.ball_0_lowerHSV = [0,0,0]
        self.ball_0_upperHSV = [0,0,0]
        self.ball_1_lowerHSV = [0,0,0]
        self.ball_1_upperHSV = [0,0,0]

        ##IPC objects
        self.eExit = mp.Event()
        pipeCmd2, self.pipeCmd = mp.Pipe()
        self.pipeData, pipeData2 = mp.Pipe()

        #initialize Sending only Socket
        self.sendSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        #initialize the Processing thread
        self.eExit.set()
        self.pTracker = BallTrackingThreadHandler(cameraID, eError, self.eExit,
                    pipeCmd2, pipeData2)
        self.pTracker.start()

        ##initialize Ball 0,1 position
        self.ballPosX = [0, 0]
        self.ballPosY = [0, 0]
        self.ballPosVisible = [False, False]

        ##initialize mapping file
        self.lensMapAlpha = None
        self.lensMapBeta = None

    """
    - return: bool -- success
        - If the initialization is successful return "True" else is "False"
    """
    def begin_capture(self):
        #signal to start cpp backend
        self.eExit.clear()

        if not self.pipeData.poll(INITIALIZATION_TIMEOUT):
            print("Camera %d timmed-out opening OpenCV"%(self.cameraID))
            return False

        ##grab frame size from subprocess
        data = self.pipeData.recv()
        self.vidWidth = data[0]
        self.vidHeight = data[1]

        print("Camera %d Capture Begin"%(self.cameraID))
        return True

    """
    Signal to start next frame processing, should be call at begining of each loop
    """
    def set_next_frame(self):
        self.pipeCmd.send("n".encode('utf-8'))


    """
    - [[int, int, int], [int, int, int]], [[int, int, int], [int, int, int]]
        - Pass in [[Lower HSV Color], [Upper HSV Color]]
         set filter color threshold for both ball 1 and 2
    """
    def set_mask_color(self, ball0Limit, ball1Limit):
        self.ball_0_lowerHSV = ball0Limit[0]
        self.ball_0_upperHSV = ball0Limit[1]
        self.ball_1_lowerHSV = ball1Limit[0]
        self.ball_1_upperHSV = ball1Limit[1]
        self._set_mask_color()

    def _set_mask_color(self):
        def append_3(msg, arr):
            for i in range(3):
                try:
                    msg+=arr[i].to_bytes(1,'little')
                except:
                    msg+=b'\xff'
            return msg
        message=b"c:"
        message = append_3(message, self.ball_0_lowerHSV)
        message = append_3(message, self.ball_0_upperHSV)
        message = append_3(message, self.ball_1_lowerHSV)
        message = append_3(message, self.ball_1_upperHSV)

        self.pipeCmd.send(message)


    """
    - path, path
        - Pass in camera Lens mapping for both x and y angle
    """
    def set_lens_mapping(self, xMapFile, yMapFile):
        if not os.path.isfile(xMapFile) or not os.path.isfile(yMapFile):
            print("Can't find or load mapping file")
            return False
        with open(xMapFile, 'rb') as f:
            self.lensMapAlpha = pickle.load(f)
        with open(yMapFile, 'rb') as f:
            self.lensMapBeta = pickle.load(f)
        return True

    """
    - return: int, int -- x_size, y_size
        - x_size, y_size of the camera Frame
    """
    def get_frame_size(self):
        return self.vidWidth, self.vidHeight

    """
    - int -- timeout
        - timeout value of how long the function should wait: -1=Inf, 0=no wait
        time
    - return: bool -- ready
        - return true if ball is in frame and new data/frame is ready for extracting.
        else return false when function timed out. (This is a blocking function)
        At the same time, update the ball position and store in variable
    """
    def frame_ready(self, timeout=0):
        ##basically Wait till "eStartProcessing" is cleared
        ## with blocking indefinitly
        if timeout < 0:
            self.pipeData.poll()
            self._update_ball_position()
            return True

        ## with timeout
        ret = self.pipeData.poll(timeout)
        self._update_ball_position()
        return ret


    """
    - get information from the Thread for position of the ball
    """
    def _update_ball_position(self):
        ################################################ Put Ball Posisiton Transfere Here
        if self.pipeData.poll(0):
            ballPos = self.pipeData.recv()
            self.ballPosVisible = [ballPos[0][0], ballPos[1][0]]
            if self.ballPosVisible[0]:
                self.ballPosX[0] = ballPos[0][1]
                self.ballPosY[0] = ballPos[0][2]
            if self.ballPosVisible[1]:
                self.ballPosX[1] = ballPos[1][1]
                self.ballPosY[1] = ballPos[1][2]

        ### Start the next Frame Capture and begin processing right after
        # self.eStartProcessing.set()

    """
    - int -- ballID
        - The id of the ball to locate. the ID can be hardcoded with color. Ex.
        Green=0, Red=1
    - return: bool, int, int -- ball_present, x, y
        - Boolean of if the ball with given ID is in frame
        - x, y pixel of where the ball with given ID is located in the camera.
        if ball is not present, function should return the last known position of
        the ball.
    """
    def get_ball_position(self, i):
        ## return old position if frame is not yet ready
        ## or getting second ball position, since Event will be set
        return self.ballPosVisible[i], self.ballPosX[i], self.ballPosY[i]

    """
    - int -- ballID
        - The id of the ball to locate. the ID can be hardcoded with color. Ex.
        Green=0, Red=1
    - return: float, float -- angle_alpha, angle_beta
        get the angle of the line connecting center point of camera to the ball
    """
    def get_ball_angle(self, i):
        data = self.get_ball_position(i)
        ####################need to adjust for camera orientation here
        alpha = self.lensMapAlpha(data[1], data[2])
        beta = self.lensMapBeta(data[1], data[2])

        return data[0], alpha, beta

    """
    - int -- ballID
        - same as above
    - return: bool, DQ -- ball_present, ball_vec
        - Boolean of if the ball with given ID is in frame
        - Unit DQ of where the ball is pointing from Camera origin Reference Frame.
        Processing of this function should also account for camera lens optical
        distortion. Question of how, should be handled internally as cameras model
        used is the same. Behavior under error condition should be same as previous.
    """
    def get_ball_dq_origin(self, i):

        data = self.get_ball_angle(i)
        rot_quat = Rotation.from_euler('zyx', [0, data[2] , data[1]],
                    degrees=False).as_quat()
        r = DQ([rot_quat[3], rot_quat[0],rot_quat[1],rot_quat[2]])

        return data[0], r
    """
    - int -- ballID
        - same as above
    - DQ -- cameraDQ
        - The DQ representing the camera position and facing direction
    - return: bool, DQ -- ball_present, ball_vec
        - Boolean of if the ball with given ID is in frame
        - Unit DQ representing a line in space of where the ball with give ID could
        be located.
    """
    def get_ball_dq_pov(self, i, povDQ):
        pass


    """
    - graceful handling of the exiting process and resource deallocation.
    """
    def exit(self):
        self.eExit.set()
        self.pTracker.join(10)
        if not self.pTracker.is_alive():
            print("Camera %d Thread Joined"%(self.cameraID))
        else:
            self.pTracker.terminate()
            print("Camera %d Force Terminated"%(self.cameraID))



class BallTrackingThreadHandler(mp.Process):
    def __init__(self, cameraID, eError, eExit, pipeCmd,
                 pipeData, ):
        # Assigning Variable
        self.cameraID = cameraID
        self.eError = eError
        self.eExit = eExit
        self.pipeCmd = pipeCmd
        self.pipeData = pipeData
        ## set error flag before initialized
        self.eError.set()
        ##Launch Process
        super(BallTrackingThreadHandler, self).__init__()

    def run(self):
        #wait till eExit is cleared to start
        while self.eExit.is_set():
            time.sleep(0.5)

        command = ['./BallTrackerThread', "%d"%self.cameraID,
        "-width=%d"%DEFAULT_FRAME_WIDTH,
        "-height=%d"%DEFAULT_FRAME_HEIGHT,
        "-show=%d"%SHOW_REALTIME]
        trackerProc = subprocess.Popen(command, stdout=subprocess.PIPE,
                                    stdin=subprocess.PIPE,
                                    stderr=subprocess.STDOUT)
        ##setting up pipe
        print("setting up process pipe")
        try:
            out = trackerProc.stdout.readline()
        except Exception as e:
            print(e)
            print("something went wrong when communicating")
            trackerProc.kill()
            return
        data = out.decode("utf-8")
        datas = data.split(":")
        if(datas[1]=="Camera"):
            print("showing realtime window")
            print(data)
            out = trackerProc.stdout.readline()
            data = out.decode("utf-8")
            datas = data.split(":")
        #grab Frame Size information
        xy = datas[1][1:].split(",")
        self.vidWidth = int(xy[0])
        self.vidHeight = int(xy[1])
        #pipe to main
        print("reciving frame information")
        self.pipeData.send([self.vidWidth, self.vidHeight])

        #check if camera can open
        out = trackerProc.stdout.readline()
        data = out.decode("utf-8")
        data = data.strip()
        datas = data.split(":")
        if(datas[0]=="erro"):
            print("camera encountered error")
            return
        else:
            print(data)
            print("entering camera loop for cam %d"%self.cameraID)

        self.eError.clear()
        while trackerProc.poll() is None:
            if self.pipeCmd.poll(0.5):
                cmd = self.pipeCmd.recv()
                print("recived cmd:%s"%(str(cmd)))

                #send command message to cpp sub process
                trackerProc.stdin.write(cmd)
                if(cmd[0]==99):
                    print("skipping this loop")
                    continue
                out = trackerProc.stdout.readline()
                # data, err = self.trackerProc.communicate(input=cmd)
                ##process Data
                data = out.decode("utf-8")
                print("got line: %s"%(data))
                data = data.strip()

                cell = data.split(":")
                if(cell[0] == "erro"): #recived error code
                    print("recv Error")
                    self.eExit.set()

                if(cell[0] == "data"):
                    arr = cell[1].split(",")
                    for i in range(6):
                        arr[i]=int(arr[i])
                    self.pipeData.send([[arr[0], arr[1], arr[2]],
                                        [arr[3], arr[4], arr[5]]])
            if(self.eExit.is_set()):
                print("exit Event set")
                trackerProc.stdin.write(b'e'.encode("utf-8"))
                trackerProc.terminate()
                break;
        trackerProc.wait(10)
        self.eError.set()







if __name__=='__main__':
    main()
    # main_2instants()
