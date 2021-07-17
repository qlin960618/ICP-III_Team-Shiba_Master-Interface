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
    recvPort = 3344 #+1 will be taken also by frontend
    sendPort = 2345 #+1 will be taken also by frontend

    DEFAULT_greenLimit = [[71, 59, 31], [108, 255, 192]]
    DEFAULT_redLimit = [[111, 152, 175], [184, 255, 255]]
    INTEPOLER_FILE_NAME = "./camera_config/lens_map.sciobj"
    ##### Determined Using the ColorRanger Script

    ##event for monitoring error
    eError = mp.Event()
    eExit = mp.Event()

    tracker1=BallTracker(2, eError, eExit, recvPort, sendPort, backend='py')
    if not tracker1.begin_capture():
        print("Test_main: Error: with openCV")
        exit()

    tracker1.set_mask_color(DEFAULT_greenLimit, DEFAULT_redLimit)
    print("Test_main: Frame Size: %d x %d"%tracker1.get_frame_size())
    tracker1.set_lens_mapping(INTEPOLER_FILE_NAME)

    print("Test_main: Holding start")
    time.sleep(2)
    print("Test_main: sending start signal")
    tracker1.set_next_frame()

    for i in range(10000):
        if not tracker1.frame_ready(10):
            print("Test_main: getting Frame Timed out")
            tracker1.exit()
            exit()
        tracker1.set_next_frame()
        # pos = tracker1.get_ball_angle(1)
        # print("cnt: %d -- x: %.5f, y: %.5f"%(i, pos[1], pos[2]))
        # pos = tracker1.get_ball_dq_origin_plucker(0)
        pos = tracker1.get_ball_line_pos(0)
        # print("Test_main: cnt: %d -- DQ: %s"%(i, str(pos[1])))
        print("Test_main: cnt: %d : x: %d, y: %d, z:%d"%(i, pos[2],pos[3], pos[1]))

    tracker1.exit()
def main_2instants():

    #in format of Lower H, S, V
    #             Upper H, S, V
    recvPort = 3344 #+1 will be taken also by frontend
    sendPort = 2345 #+1 will be taken also by frontend
    DEFAULT_greenLimit = [[66, 179, 101], [101, 255, 255]]
    DEFAULT_redLimit = [[118, 95, 116], [189, 255, 255]]

    INTEPOLER_FILE_NAME = "./camera_config/lens_map.sciobj"
    BACKEND='cpp'
    ##### Determined Using the ColorRanger Script

    tracker = [None, None]
    eError = mp.Event()
    eExit = mp.Event()
    tracker[0]=BallTracker(0, eError, eExit, recvPort, sendPort, backend=BACKEND)
    tracker[1]=BallTracker(1, eError, eExit, recvPort-1, sendPort-1, backend=BACKEND)

    if not tracker[0].begin_capture():
        print("Test_main2: Error: with openCV")
        tracker[0].exit()
        return
    if not tracker[1].begin_capture():
        print("Test_main2: Error: with openCV")
        tracker[0].exit()
        tracker[1].exit()
        return

    time.sleep(2)
    tracker[0].set_mask_color(DEFAULT_greenLimit, DEFAULT_redLimit)
    tracker[1].set_mask_color(DEFAULT_greenLimit, DEFAULT_redLimit)
    print("Test_main2: Frame Size: Cam %d: %d x %d"%(0, *tracker[0].get_frame_size()))
    print("Test_main2: Frame Size: Cam %d: %d x %d"%(1, *tracker[1].get_frame_size()))
    tracker[0].set_lens_mapping(INTEPOLER_FILE_NAME)
    tracker[1].set_lens_mapping(INTEPOLER_FILE_NAME)

    #     //initialize processing of first frame before enter loop
    #     FOR i in 0,1: //camera
    #         hCam[i].set_next_frame()
    #     ENDFOR
    print("Test_main2: Holding start")
    print("Test_main2: sending start signal")
    tracker[0].set_next_frame()
    tracker[1].set_next_frame()

    count=0
    while eError.is_set():
        time.sleep(1)
        print("Test_main2: Waiting...")
        count+=1
        if count>20:
            print("Test_main2: Waiting for camera time out")
            tracker[0].exit()
            tracker[1].exit()
            return
    ############################## Initialise tracking setup ##############################



    start_t = timeit.default_timer()
    for i in range(100000):
        tracker[0].set_next_frame()
        tracker[1].set_next_frame()
        if not tracker[0].frame_ready(10) or not tracker[1].frame_ready(10) or eError.is_set():
            print("Test_main2: getting Frame Timed out")
            tracker[0].exit()
            tracker[1].exit()
            exit()
        # pos = tracker1.get_ball_angle(1)
        # print("cnt: %d -- x: %.5f, y: %.5f"%(i, pos[1], pos[2]))
        pos1_1 = tracker[0].get_ball_dq_origin_plucker(1)
        pos2_1 = tracker[1].get_ball_dq_origin_plucker(1)
        pos1_0 = tracker[0].get_ball_dq_origin_plucker(0)
        pos2_0 = tracker[1].get_ball_dq_origin_plucker(0)

        end_t = timeit.default_timer()
        print("Test_main2: Program Rate: %.3f"%(1/(end_t-start_t)))
        start_t=end_t

        # print("cnt: %d -- DQ: %s | DQ: %s"%(i, str(pos1[1]), str(pos2[1])))
    tracker[0].exit()
    tracker[1].exit()




class BallTracker():


    def __init__(self, cameraID, eError, eExit, recvPort, sendPort, backend='cpp'):

        #set backend
        self.backend=backend
        if self.backend == 'py':
            import cv2 as cv
            import imutils
        else:
            #initialize Sending only Socket
            self.recvPort = recvPort
            self.sendPort = sendPort

        self.cameraID = cameraID
        self.eError = eError
        self.ePause = mp.Event()
        self.eExit = eExit
        self.ball_0_lowerVVV = [0,0,0]
        self.ball_0_upperVVV = [0,0,0]
        self.ball_1_lowerVVV = [0,0,0]
        self.ball_1_upperVVV = [0,0,0]

        ##IPC objects
        # self.eNextFrame = mp.Event()
        self.pipeData, pipeData2 = mp.Pipe()
        #pip for command, not used when in cpp backend
        pipeCmd2, self.pipeCmd = mp.Pipe()


        #initialize the Processing thread
        self.ePause.set()
        self.pTracker = BallTrackingThreadHandler(cameraID, self.eError,
                    self.eExit, self.ePause, sendPort,
                    recvPort, pipeData2, pipeCmd2, self.backend)
        self.pTracker.start()


        ##initialize Ball 0,1 position
        self.ballPosX = [0, 0]
        self.ballPosY = [0, 0]
        self.ballPosVisible = [False, False]

        ##initialize mapping file
        self.lensMapX = None
        self.lensMapY = None
        self.gridDistance = None

    """
    - return: bool -- success
        - If the initialization is successful return "True" else is "False"
    """
    def begin_capture(self):
        #signal to start cpp backend
        self.ePause.clear()

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
        if self.backend == 'py':
            self.pipeCmd.send('n')
        else:
            self.pipeCmd.send(b"n")

            # self.sendSock.sendto(message, (self.sendToIP, self.sendPort))


    """
    - [[int, int, int], [int, int, int]], [[int, int, int], [int, int, int]]
        - Pass in [[Lower HSV Color], [Upper HSV Color]]
         set filter color threshold for both ball 1 and 2
    """
    def set_mask_color(self, ball0Limit, ball1Limit):
        self.ball_0_lowerVVV = ball0Limit[0]
        self.ball_0_upperVVV = ball0Limit[1]
        self.ball_1_lowerVVV = ball1Limit[0]
        self.ball_1_upperVVV = ball1Limit[1]
        self._set_mask_color()

    def _set_mask_color(self):
        def append_3(msg, arr):
            for i in range(3):
                try:
                    msg+=arr[i].to_bytes(1,'little')
                except:
                    msg+=b'\xff'
            return msg
        if self.backend == 'py':
            self.pipeCmd.send([self.ball_0_lowerVVV,
                               self.ball_0_upperVVV,
                               self.ball_1_lowerVVV,
                               self.ball_1_upperVVV])
            print("color: ", end='')
            print(self.ball_0_lowerVVV, end='')
            print(self.ball_0_upperVVV, end='')
            print(self.ball_1_lowerVVV, end='')
            print(self.ball_1_upperVVV)
        else:
            message=b"c:"
            message = append_3(message, self.ball_0_lowerVVV)
            message = append_3(message, self.ball_0_upperVVV)
            message = append_3(message, self.ball_1_lowerVVV)
            message = append_3(message, self.ball_1_upperVVV)

            print("color msg: %s"%message)
            self.pipeCmd.send(message)


    """
    - path, path
        - Pass in camera Lens mapping for both x and y angle
    """
    def set_lens_mapping(self, MapFile):
        if not os.path.isfile(MapFile):
            print("Can't find or load mapping file")
            return False
        with open(MapFile, 'rb') as f:
            temp = pickle.load(f)
            self.lensMapX = temp[0]
            self.lensMapY = temp[1]
            self.gridDistance = temp[2]

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

    """pyBackend:
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
    - return: bool, float, float, float -- visible, depth, x_pos, y_pos
        get the a point in camera frame where it is in line to where the camera and ball pass through
    """
    def get_ball_line_pos(self, i):
        data = self.get_ball_position(i)
        ####################need to adjust for camera orientation here
        x = self.lensMapX(data[1], data[2])
        y = self.lensMapY(data[1], data[2])

        return data[0], self.gridDistance/100.0,  x/1000.0, y/1000.0

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
    def get_ball_dq_origin_plucker(self, i):

        data = self.get_ball_line_pos(i)
        pt=DQ([0, data[2], -data[3], data[1]])
        dir = DQ.normalize(pt)
        plk = dir + DQ.E * cross(pt, dir)
        #since moment is 0 from origin
        #m = DQ.cross(DQ([0]), dir)

        return data[0], plk# + DQ.E*m
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
    def get_ball_dq_pov_plucker(self, i, povDQ):
        data=self.get_ball_dq_origin_plucker(i)
        return data[0], povDQ * data[1] * conj(povDQ)


    """
    - graceful handling of the exiting process and resource deallocation.
    """
    def exit(self):
        self.eExit.set()
        if self.backend == 'py':
            self.pipeCmd.send('e')
        else:
            message=b"e"
            self.pipeCmd.send(message)
            # self.sendSock.sendto(message, (self.sendToIP, self.sendPort))
        # self.pTracker.join(10)
        if not self.pTracker.is_alive():
            print("Camera %d Thread Joined"%(self.cameraID))
            self.pTracker.terminate()
        else:
            print("Camera %d Force Terminated"%(self.cameraID))



class BallTrackingThreadHandler(mp.Process):
    def __init__(self, cameraID, eError, eExit, ePause, sendPort, recvPort,
                 pipeData, pipeCmd, backend='cpp'):

        #set processing backend
        self.backend=backend
        # Assigning Variable
        self.cameraID = cameraID
        self.ePause = ePause
        self.eError = eError
        self.eExit = eExit
        self.sendPort = sendPort
        self.recvPort = recvPort
        self.pipeData = pipeData
        self.pipeCmd = pipeCmd
        ## set error flag before initialized
        self.ePause.set()
        ##Launch Process
        super(BallTrackingThreadHandler, self).__init__()

    def run(self):

        ####Code section for when using cpp backend
        if self.backend == 'cpp':
            #wait till eExit is cleared to start
            while self.ePause.is_set():
                time.sleep(0.5)

            #setup UDP communication
            recvFromIP = "0.0.0.0"
            self.recvSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.recvSock.bind((recvFromIP, self.recvPort))
            self.sendToIP="localhost"
            self.sendSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            print("backend(py): Camera %d: Start cpp process"%(self.cameraID))
            command = ['./BallTrackerThread', "%d"%self.cameraID,
            "-width=%d"%DEFAULT_FRAME_WIDTH,
            "-height=%d"%DEFAULT_FRAME_HEIGHT,
            "-rPort=%d"%self.sendPort, "-sPort=%d"%self.recvPort,
            "-show=%d"%SHOW_REALTIME]
            trackerProcess = subprocess.Popen(command)

            print("backend(py): Camera %d: get frame info"%(self.cameraID))
            #grab Frame Size information
            data, srcAddr = self.recvSock.recvfrom(100)
            data = data.decode("utf-8")
            xy = data[2:].split(",")
            self.vidWidth = int(xy[0])
            self.vidHeight = int(xy[1])
            #pipe to main
            self.pipeData.send([self.vidWidth, self.vidHeight])

            #set socket to nonBlocking
            self.recvSock.setblocking(False)
            read_list=[self.recvSock]

            self.ePause.clear()
            print("backend(py): Camera %d: Enter reciv loop"%(self.cameraID))
            while trackerProcess.poll() is None:
                #if there is command send over socket
                if self.pipeCmd.poll(0):
                    message=self.pipeCmd.recv()
                    self.sendSock.sendto(message, (self.sendToIP, self.sendPort))

                readable, _, _ = select.select(read_list, [], [], 0)
                for s in readable:
                    if s==self.recvSock:
                        data, srcAddr = s.recvfrom(100)
                        ##process Data
                        data = data.decode("utf-8")
                        # "d:%d:%d,%d,%d,%d,%d,%d"
                        # "err:%d"
                        cell = data.split(":")
                        if(cell[0] == "err"): #recived error code
                            print("backend(py): recv Error")
                            self.eExit.set()

                        if(cell[0] == "d" and cell[1] == "%d"%self.cameraID):
                            arr = cell[2].split(",")
                            for i in range(6):
                                arr[i]=int(arr[i])
                            self.pipeData.send([[arr[0], arr[1], arr[2]],
                                                [arr[3], arr[4], arr[5]]])
                if(self.eExit.is_set()):
                    self.sendSock.sendto(b"e", (self.sendToIP, self.sendPort))
                    print("backend(py): exit Event set")
                    time.sleep(1)
                    trackerProcess.terminate()
                    break;

            print("backend(py): Camera %d: Exiting backend"%(self.cameraID))
            trackerProcess.wait(10)
            self.recvSock.close()
            self.sendSock.close()

        ##using python Backend
        else:
            import cv2 as cv
            import imutils

            #wait till eExit is cleared to start
            while self.ePause.is_set():
                time.sleep(0.5)

            #set color variable
            ball_0_lowerVVV = [0,0,0]
            ball_0_upperVVV = [0,0,0]
            ball_1_lowerVVV = [0,0,0]
            ball_1_upperVVV = [0,0,0]

            #camera Initialization
            print("pyBackend: Camera %d: open device"%(self.cameraID))
            capDev = cv.VideoCapture(self.cameraID)
            capDev.set(cv.CAP_PROP_FRAME_WIDTH, DEFAULT_FRAME_WIDTH)
            capDev.set(cv.CAP_PROP_FRAME_HEIGHT, DEFAULT_FRAME_HEIGHT)
            self.vidWidth = capDev.get(cv.CAP_PROP_FRAME_WIDTH)
            self.vidHeight = capDev.get(cv.CAP_PROP_FRAME_HEIGHT)

            self.pipeData.send([self.vidWidth, self.vidHeight])
            #Test capture single Frame
            ret, frame = capDev.read()
            if frame is None or not ret:
                print("pyBackend: Camera %d: Something Catastrophic Happened in Init"%(self.cameraID))
                # print("pyBackend: Camera %d: "%self.cameraID, end="")
                # print(capDev.error())
                capDev.release()
                self.eError.set()
                return

            ##Clear ePause Flag
            self.ePause.clear()
            # print("dsfljlksf: %s"%str(self.eError.is_set()))
            # print("dsfljlksf: %s"%str(self.eExit.is_set()))
            ######################### Main Loop #########################
            while not self.eError.is_set() and not self.eExit.is_set():
                #recive command from main
                cmd = self.pipeCmd.recv()

                if cmd == 'e':
                    #exit signal recive
                    capDev.release()
                    break
                elif len(cmd) == 4:
                    #command is color
                    ball_0_lowerVVV = cmd[0]
                    ball_0_upperVVV = cmd[1]
                    ball_1_lowerVVV = cmd[2]
                    ball_1_upperVVV = cmd[3]
                elif not cmd == 'n':  #didnt recive the next command
                    time.sleep(0.1)
                    continue

                #get current Frame:
                ret, frame = capDev.read()

                if frame is None:
                    print(cv.error())
                    print("pyBackend: Camera %d: Something Catastrophic Happened"%(self.cameraID))
                    self.eError.set()
                    break

                #######start detection pipeline
                blurred = cv.GaussianBlur(frame, (11, 11), 0)


                ###### if using hsv
                pre_thrsh_frame = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
                ###### if using rgb
                # pre_thrsh_frame = blurred.copy()
                ######

                c_0_mask = cv.inRange(pre_thrsh_frame,
                            (ball_0_lowerVVV[0], ball_0_lowerVVV[1], ball_0_lowerVVV[2]),
                            (ball_0_upperVVV[0], ball_0_upperVVV[1], ball_0_upperVVV[2]))
                c_1_mask = cv.inRange(pre_thrsh_frame,
                            (ball_1_lowerVVV[0], ball_1_lowerVVV[1], ball_1_lowerVVV[2]),
                            (ball_1_upperVVV[0], ball_1_upperVVV[1], ball_1_upperVVV[2]))
                c_0_mask = cv.erode(c_0_mask, None, iterations=2)
                c_0_mask = cv.dilate(c_0_mask, None, iterations=2)
                c_1_mask = cv.erode(c_1_mask, None, iterations=2)
                c_1_mask = cv.dilate(c_1_mask, None, iterations=2)
                cnts_c_0 = cv.findContours(c_0_mask, cv.RETR_EXTERNAL,
                            cv.CHAIN_APPROX_SIMPLE)
                cnts_c_1 = cv.findContours(c_1_mask, cv.RETR_EXTERNAL,
                            cv.CHAIN_APPROX_SIMPLE)

                cnts_c_0 = imutils.grab_contours(cnts_c_0)
                cnts_c_1 = imutils.grab_contours(cnts_c_1)
                ball_0 = [False, 0, 0]
                ball_1 = [False, 0, 0]

                center = None
                if len(cnts_c_0) > 0:
                    # find the largest contour in the mask, then use
                    # it to compute the minimum enclosing circle and
                    # centroid
                    c = max(cnts_c_0, key=cv.contourArea)
                    ((x, y), radius) = cv.minEnclosingCircle(c)
                    M = cv.moments(c)
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    # only proceed if the radius meets a minimum size
                    if radius > CIRCLE_R_THRESHOLD:
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        ball_0[0]=True
                        ball_0[1]=center[0]
                        ball_0[2]=center[1]
                        if SHOW_REALTIME:
                            cv.circle(frame, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
                            cv.circle(frame, center, 5, (0, 0, 255), -1)

                center = None
                if len(cnts_c_1) > 0:
                    c = max(cnts_c_1, key=cv.contourArea)
                    ((x, y), radius) = cv.minEnclosingCircle(c)
                    M = cv.moments(c)
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    # only proceed if the radius meets a minimum size
                    if radius > CIRCLE_R_THRESHOLD:
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        ball_1[0]=True
                        ball_1[1]=center[0]
                        ball_1[2]=center[1]
                        if SHOW_REALTIME:
                            cv.circle(frame, (int(x), int(y)), int(radius),
                                (0, 255, 255), 2)
                            cv.circle(frame, center, 5, (0, 0, 255), -1)
                ###display realtime live view of the camera
                if SHOW_REALTIME:
                    cv.namedWindow("Camera %d"%(self.cameraID))
                    cv.imshow("Camera %d"%(self.cameraID), cv.resize(frame, (0,0), fx=0.4, fy=0.4))
                    cv.waitKey(1)
                ## Send all data to main process here
                self.pipeData.send([ball_0, ball_1])
            print("pyBackend: Exiting loop")

        self.eError.set()

if __name__=='__main__':
    main()
    # main_2instants()
