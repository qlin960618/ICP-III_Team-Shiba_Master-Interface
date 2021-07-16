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
    DEFAULT_greenLimit = [[71, 59, 31], [108, 144, 78]]
    DEFAULT_redLimit = [[111, 106, 24], [184, 255, 210]]

    INTEPOLER_FILE_NAME = "./camera_config/lens_map.sciobj"
    ##### Determined Using the ColorRanger Script

    ##event for monitoring error
    eError = mp.Event()

    tracker1=BallTracker(0, eError, recvPort, sendPort, backend='cpp')
    if not tracker1.begin_capture():
        print("Error: with openCV")
        exit()

    tracker1.set_mask_color(DEFAULT_greenLimit, DEFAULT_redLimit)
    print("Frame Size: %d x %d"%tracker1.get_frame_size())
    tracker1.set_lens_mapping(INTEPOLER_FILE_NAME)

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
        pos = tracker1.get_ball_dq_origin_plucker(1)
        print("cnt: %d -- DQ: %s"%(i, str(pos[1])))
    tracker1.exit()
def main_2instants():

    #in format of Lower H, S, V
    #             Upper H, S, V
    recvPort = 3344 #+1 will be taken also by frontend
    sendPort = 2345 #+1 will be taken also by frontend
    DEFAULT_greenLimit = [[66, 179, 101], [101, 255, 255]]
    DEFAULT_redLimit = [[118, 95, 116], [189, 255, 255]]

    INTEPOLER_FILE_NAME = "./camera_config/lens_map.sciobj"
    ##### Determined Using the ColorRanger Script

    ##event for monitoring error
    eError = mp.Event()

    tracker1=BallTracker(0, eError, recvPort, sendPort)
    tracker2=BallTracker(1, eError, recvPort+1, sendPort+1)

    print("Holding start")
    time.sleep(2)
    print("sending start signal")

    if not tracker1.begin_capture() or not tracker2.begin_capture():
        print("Error: with openCV")
        exit()

    tracker1.set_mask_color(DEFAULT_greenLimit, DEFAULT_redLimit)
    tracker2.set_mask_color(DEFAULT_greenLimit, DEFAULT_redLimit)
    print("Frame Size: %d x %d"%tracker1.get_frame_size())
    tracker1.set_lens_mapping(INTEPOLER_FILE_NAME)
    tracker2.set_lens_mapping(INTEPOLER_FILE_NAME)



    start_t = timeit.default_timer()
    for i in range(100000):
        tracker1.set_next_frame()
        tracker2.set_next_frame()
        if not tracker1.frame_ready(10) or not tracker2.frame_ready(10) or eError.is_set():
            print("getting Frame Timed out")
            tracker1.exit()
            tracker2.exit()
            exit()
        # pos = tracker1.get_ball_angle(1)
        # print("cnt: %d -- x: %.5f, y: %.5f"%(i, pos[1], pos[2]))
        pos1_1 = tracker1.get_ball_dq_origin_plucker(1)
        pos2_1 = tracker2.get_ball_dq_origin_plucker(1)
        pos1_0 = tracker1.get_ball_dq_origin_plucker(0)
        pos2_0 = tracker2.get_ball_dq_origin_plucker(0)

        end_t = timeit.default_timer()
        print("Program Rate: %.3f"%(1/(end_t-start_t)))
        start_t=end_t

        # print("cnt: %d -- DQ: %s | DQ: %s"%(i, str(pos1[1]), str(pos2[1])))
    tracker1.exit()
    tracker2.exit()




class BallTracker():


    def __init__(self, cameraID, eError, recvPort, sendPort, backend='cpp'):

        #set backend
        self.backend=backend
        if self.backend == 'py':
            import cv2 as cv
            import imutils
        else:
            self.sendToIP="localhost"
            #initialize Sending only Socket
            self.sendSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.recvPort = recvPort
            self.sendPort = sendPort

        self.cameraID = cameraID
        self.eError = eError
        self.ball_0_lowerHSV = [0,0,0]
        self.ball_0_upperHSV = [0,0,0]
        self.ball_1_lowerHSV = [0,0,0]
        self.ball_1_upperHSV = [0,0,0]

        ##IPC objects
        self.eExit = mp.Event()
        # self.eNextFrame = mp.Event()
        self.pipeData, pipeData2 = mp.Pipe()
        #pip for command, not used when in cpp backend
        pipeCmd2, self.pipeCmd = mp.Pipe()


        #initialize the Processing thread
        self.eExit.set()
        self.pTracker = BallTrackingThreadHandler(cameraID, eError, self.eExit,
                    sendPort, recvPort, pipeData2, pipeCmd2, self.backend)
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
        if self.backend == 'py':
            self.pipeCmd.send('n')
        else:
            message=b"n"
            self.sendSock.sendto(message, (self.sendToIP, self.sendPort))


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
        if self.backend == 'py':
            self.pipeCmd.send([self.ball_0_lowerHSV,
                               self.ball_0_upperHSV,
                               self.ball_1_lowerHSV,
                               self.ball_1_upperHSV])
            print("color: ", end='')
            print(self.ball_0_lowerHSV, end='')
            print(self.ball_0_upperHSV, end='')
            print(self.ball_1_lowerHSV, end='')
            print(self.ball_1_upperHSV)
        else:
            message=b"c:"
            message = append_3(message, self.ball_0_lowerHSV)
            message = append_3(message, self.ball_0_upperHSV)
            message = append_3(message, self.ball_1_lowerHSV)
            message = append_3(message, self.ball_1_upperHSV)

            print("color msg: %s"%message)
            self.sendSock.sendto(message, (self.sendToIP, self.sendPort))


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
    - return: bool, float, float, float -- visible, depth, x_pos, y_pos
        get the a point in camera frame where it is in line to where the camera and ball pass through
    """
    def get_ball_line_pos(self, i):
        data = self.get_ball_position(i)
        ####################need to adjust for camera orientation here
        x = self.lensMapX(data[1], data[2])
        y = self.lensMapY(data[1], data[2])

        return data[0], self.gridDistance,  x, y

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
        dir = DQ.normalize(DQ([data[2], data[3], data[1]]))
        #since moment is 0 from origin
        #m = DQ.cross(DQ([0]), dir)

        return data[0], dir# + DQ.E*m
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
            self.sendSock.sendto(message, (self.sendToIP, self.sendPort))
        self.pTracker.join(10)
        if not self.pTracker.is_alive():
            print("Camera %d Thread Joined"%(self.cameraID))
        else:
            self.pTracker.terminate()
            print("Camera %d Force Terminated"%(self.cameraID))

        self.recvSock.close()
        self.sendSock.close()



class BallTrackingThreadHandler(mp.Process):
    def __init__(self, cameraID, eError, eExit, sendPort, recvPort,
                 pipeData, pipeCmd, backend='cpp'):

        #set processing backend
        self.backend=backend
        # Assigning Variable
        self.cameraID = cameraID
        self.eError = eError
        self.eExit = eExit
        self.sendPort = sendPort
        self.recvPort = recvPort
        self.pipeData = pipeData
        self.pipeCmd = pipeCmd
        ## set error flag before initialized
        self.eError.set()
        ##Launch Process
        super(BallTrackingThreadHandler, self).__init__()

    def run(self):

        ####Code section for when using cpp backend
        if self.backend == 'cpp':
            #wait till eExit is cleared to start
            while self.eExit.is_set():
                time.sleep(0.5)

            #setup UDP communication
            recvFromIP = "0.0.0.0"
            self.recvSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.recvSock.bind((recvFromIP, self.recvPort))

            command = ['./BallTrackerThread', "%d"%self.cameraID,
            "-width=%d"%DEFAULT_FRAME_WIDTH,
            "-height=%d"%DEFAULT_FRAME_HEIGHT,
            "-rPort=%d"%self.sendPort, "-sPort=%d"%self.recvPort,
            "-show=%d"%SHOW_REALTIME]
            trackerProcess = subprocess.Popen(command)

            #grab Frame Size information
            data, srcAddr = self.recvSock.recvfrom(100)
            data = data.decode("utf-8")
            xy = data[2:].split(",")
            self.vidWidth = int(xy[0])
            self.vidHeight = int(xy[1])
            #pipe to main
            self.pipeData.send([self.vidWidth, self.vidHeight])

            #set socket to nonBlocking
            self.recvSock.setblocking(0)
            read_list=[self.recvSock]

            self.eError.clear()
            while trackerProcess.poll() is None:
                readable, _, _ = select.select(read_list, [], [])
                for s in readable:
                    if s==self.recvSock:
                        data, srcAddr = s.recvfrom(100)
                        ##process Data
                        data = data.decode("utf-8")
                        # "d:%d:%d,%d,%d,%d,%d,%d"
                        # "err:%d"
                        cell = data.split(":")
                        if(cell[0] == "err"): #recived error code
                            print("recv Error")
                            self.eExit.set()

                        if(cell[0] == "d" and cell[1] == "%d"%self.cameraID):
                            arr = cell[2].split(",")
                            for i in range(6):
                                arr[i]=int(arr[i])
                            self.pipeData.send([[arr[0], arr[1], arr[2]],
                                                [arr[3], arr[4], arr[5]]])
                if(self.eExit.is_set()):
                    print("exit Event set")
                    self.recvSock.close()
                    time.sleep(1)
                    trackerProcess.terminate()
                    break;
            trackerProcess.wait(10)

        ##using python Backend
        else:
            import cv2 as cv
            import imutils

            #wait till eExit is cleared to start
            while self.eExit.is_set():
                time.sleep(0.5)

            #set color variable
            ball_0_lowerHSV = [0,0,0]
            ball_0_upperHSV = [0,0,0]
            ball_1_lowerHSV = [0,0,0]
            ball_1_upperHSV = [0,0,0]

            #camera Initialization
            capDev = cv.VideoCapture(self.cameraID)
            capDev.set(cv.CAP_PROP_FRAME_WIDTH, DEFAULT_FRAME_WIDTH)
            capDev.set(cv.CAP_PROP_FRAME_HEIGHT, DEFAULT_FRAME_HEIGHT)
            self.vidWidth = capDev.get(cv.CAP_PROP_FRAME_WIDTH)
            self.vidHeight = capDev.get(cv.CAP_PROP_FRAME_HEIGHT)

            self.pipeData.send([self.vidWidth, self.vidHeight])
            #Test capture single Frame
            ret, frame = capDev.read()
            if frame is None or not ret:
                print("Camera %d: Something Catastrophic Happened in Init"%(self.cameraID))
                capDev.release()
                self.eError.set()
                return

            ##Clear Error Flag
            self.eError.clear()
            ######################### Main Loop #########################
            while not self.eError.is_set():
                #recive command from main
                cmd = self.pipeCmd.recv()

                if cmd == 'e':
                    #exit signal recive
                    capDev.release()
                    break
                elif len(cmd) == 4:
                    #command is color
                    ball_0_lowerHSV = cmd[0]
                    ball_0_upperHSV = cmd[1]
                    ball_1_lowerHSV = cmd[2]
                    ball_1_upperHSV = cmd[3]
                elif not cmd == 'n':  #didnt recive the next command
                    time.sleep(0.1)
                    continue

                #get current Frame:
                ret, frame = capDev.read()

                if frame is None:
                    print("Camera %d: Something Catastrophic Happened"%(self.cameraID))
                    self.eError.set()
                    break

                #######start detection pipeline
                blurred = cv.GaussianBlur(frame, (11, 11), 0)
                hsv_frame = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
                c_0_mask = cv.inRange(hsv_frame,
                            (ball_0_lowerHSV[0], ball_0_lowerHSV[1], ball_0_lowerHSV[2]),
                            (ball_0_upperHSV[0], ball_0_upperHSV[1], ball_0_upperHSV[2]))
                c_1_mask = cv.inRange(hsv_frame,
                            (ball_1_lowerHSV[0], ball_1_lowerHSV[1], ball_1_lowerHSV[2]),
                            (ball_1_upperHSV[0], ball_1_upperHSV[1], ball_1_upperHSV[2]))
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
                    cv.imshow("Camera %d"%(self.cameraID), frame)
                    cv.waitKey(1)
                ## Send all data to main process here
                self.pipeData.send([ball_0, ball_1])

        self.eError.set()

if __name__=='__main__':
    main()
    # main_2instants()
