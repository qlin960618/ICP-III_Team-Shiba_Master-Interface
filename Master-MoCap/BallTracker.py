# import the necessary packages

import numpy as np
import cv2 as cv
import imutils
import multiprocessing as mp
import time

from ctypes import Structure, c_short
from multiprocessing.sharedctypes import Array

DEFAULT_FRAME_HEIGHT = 720
DEFAULT_FRAME_WIDTH = 1280


SHOW_REALTIME = True

INITIALIZATION_TIMEOUT = 100
CIRCLE_R_THRESHOLD = 10

#in format of Lower H, S, V
#             Upper H, S, V
DEFAULT_greenLimit = [[90, 86, 47], [134, 255, 152]]
DEFAULT_redLimit = [[130, 200, 150], [134, 255, 152]]
##### Determined Using the ColorRanger Script

def main():

    ##event for monitoring error
    eError = mp.Event()

    tracker1=BallTracker(0, eError)
    tracker1.set_mask_color(DEFAULT_greenLimit, DEFAULT_redLimit)

    print("Holding start")
    time.sleep(2)
    print("sending start signal")

    if not tracker1.begin_capture():
        print("Error: with openCV")
        exit()



    for i in range(200):
        if not tracker1.frame_ready(10):
            print("getting Frame Timed out")
            tracker1.exit()
            exit()




        pos = tracker1.get_ball_position(0)
        print("x: %d, y: %d"%(pos[1], pos[2]))

    tracker1.exit()





class BallTracker():
    def __init__(self, cameraID, eError):

        self.cameraID = cameraID
        self.eError = eError
        self.ball_0_lowerHSV = [0,0,0]
        self.ball_0_upperHSV = [0,0,0]
        self.ball_1_lowerHSV = [0,0,0]
        self.ball_1_upperHSV = [0,0,0]
        self.lockColorArr = mp.Lock()
        self.colorMaskArr = Array(HSVColor,
                    [tuple(self.ball_0_lowerHSV), tuple(self.ball_0_upperHSV),
                    tuple(self.ball_1_lowerHSV), tuple(self.ball_1_upperHSV)],
                    lock=self.lockColorArr)

        self.eExitThread = mp.Event()
        self.eStartProcessing = mp.Event()
        self.pipPosition, pipPosition2 = mp.Pipe()
        self.queInfo = mp.Queue()
        self.pipCommand, pipCommand2 = mp.Pipe()
        self.lockPrint = mp.Lock()

        #initialize the Processing thread
        self.pTracker = BallTrackingThread(cameraID, eError, self.eExitThread, self.eStartProcessing,
                    pipPosition2, self.queInfo, pipCommand2, self.lockPrint, self.colorMaskArr,
                    self.lockColorArr)
        self.pTracker.start()

        #grab Frame Size information
        frameSizes = self.queInfo.get()
        self.vidWidth = frameSizes[0]
        self.vidHeight = frameSizes[1]

        ##initialize Ball 0,1 position
        self.ballPosX = [0, 0]
        self.ballPosY = [0, 0]
        self.ballPosVisible = [False, False]

    def begin_capture(self):
        self.eStartProcessing.set()
        error = True
        for i in range(INITIALIZATION_TIMEOUT):
            if self.eError.is_set():
                time.sleep(0.1)
            else:
                error = False
                break
        if error:
            with self.lockPrint:
                print("Camera %d timmed-out opening OpenCV"%(self.cameraID))
            return False
        else:
            with self.lockPrint:
                print("Camera %d Capture Begin"%(self.cameraID))
            return True

    def _set_start_next_frame(self):
        self.eStartProcessing.set()

    def set_mask_color(self, ball0Limit, ball1Limit):
        self.ball_0_lowerHSV = ball0Limit[0]
        self.ball_0_upperHSV = ball0Limit[1]
        self.ball_1_lowerHSV = ball1Limit[0]
        self.ball_1_upperHSV = ball1Limit[1]
        self.colorMaskArr[0] = tuple(ball0Limit[0])
        self.colorMaskArr[1] = tuple(ball0Limit[1])
        self.colorMaskArr[2] = tuple(ball1Limit[0])
        self.colorMaskArr[3] = tuple(ball1Limit[1])

    def get_frame_size(self):
        return self.vidWidth, self.vidHeight

    def frame_ready(self, timeout=0):
        ##basically Wait till "eStartProcessing" is cleared
        ## with blocking indefinitly
        if timeout < 0:
            while not self.eStartProcessing.is_set():
                time.sleep(0.05)
            return True
        elif timeout == 0:
            return not self.eStartProcessing.is_set()

        ## with timeout
        for i in range(int(timeout/0.05)):
            if self.eStartProcessing.is_set():
                time.sleep(0.05)
            else:
                return True

        return not self.eStartProcessing.is_set()


    #BallID
    def get_ball_position(self, i):
        ## return old position if frame is not yet ready
        ## or getting second ball position, since Event will be set
        if self.eStartProcessing.is_set():
            return self.ballPosVisible[i], self.ballPosX[i], self.ballPosY[i]

        ################################################ Put Ball Posisiton Transfere Here
        if self.pipPosition.poll():
            ballPos = self.pipPosition.recv()
            self.ballPosVisible = [ballPos[0][0], ballPos[1][0]]
            self.ballPosX = [ballPos[0][1], ballPos[1][1]]
            self.ballPosY = [ballPos[0][2], ballPos[1][2]]

        #......
        ### Start the next Frame Capture and begin processing right after
        self.eStartProcessing.set()
        #return New Position
        return self.ballPosVisible[i], self.ballPosX[i], self.ballPosY[i]

    def exit(self):
        self.eExitThread.set()
        self.pTracker.join(10)
        if not self.pTracker.is_alive():
            print("Camera %d Thread Joined"%(self.cameraID))
        else:
            self.pTracker.terminate()
            print("Camera %d Force Terminated"%(self.cameraID))



class HSVColor(Structure):
    _fields_ = [('h', c_short), ('s', c_short), ('v', c_short)]


class BallTrackingThread(mp.Process):
    def __init__(self, cameraID, eError, eExit, eStart, pipPosition, queInfo,
                pipCommand, lockPrint, colorMaskArr, lockColorArr):
        # Assigning Variable
        self.cameraID = cameraID
        self.eError = eError
        self.eExit = eExit
        self.eStart = eStart
        self.pipPosition = pipPosition
        self.queInfo = queInfo
        self.pipCommand = pipCommand
        self.lockPrint = lockPrint
        self.colorMaskArr = colorMaskArr
        self.lockColorArr = lockColorArr
        ## set error flag before initialized
        self.eError.set()
        ##Launch Process
        super(BallTrackingThread, self).__init__()

    def run(self):
        #camera Initialization
        capDev = cv.VideoCapture(self.cameraID)
        capDev.set(cv.CAP_PROP_FRAME_WIDTH, DEFAULT_FRAME_WIDTH)
        capDev.set(cv.CAP_PROP_FRAME_HEIGHT, DEFAULT_FRAME_HEIGHT)
        self.vidWidth = capDev.get(cv.CAP_PROP_FRAME_WIDTH)
        self.vidHeight = capDev.get(cv.CAP_PROP_FRAME_HEIGHT)
        self.queInfo.put([self.vidWidth, self.vidHeight])

        #Test capture single Frame
        ret, frame = capDev.read()
        if frame is None or not ret:
            with self.lockPrint:
                print("Camera %d: Something Catastrophic Happened in Init"%(self.cameraID))
            capDev.release()
            return

        ##Clear Error Flag
        self.eError.clear()
        ######################### Main Loop #########################
        while not self.eExit.is_set():
            #waiting for signal to start Processing Frame
            self.eStart.wait()

            #get current Frame:
            ret, frame = capDev.read()

            if frame is None:
                with self.lockPrint:
                    print("Camera %d: Something Catastrophic Happened"%(self.cameraID))
                self.eError.set()
                break

################################################ Put OpenCV Processing Here

            # with self.lockPrint:
                # print("loop")
                # print("%d %d %d %d"%(self.colorMaskArr[0].s,
                #                 self.colorMaskArr[1].s,
                #                 self.colorMaskArr[2].s,
                #                 self.colorMaskArr[3].s))
                # while self.pipCommand.poll():
                #     print("Command: %s"%(self.pipCommand.recv()[0]))

            blurred = cv.GaussianBlur(frame, (11, 11), 0)
            hsv_frame = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
            c_0_mask = cv.inRange(hsv_frame,
                        (self.colorMaskArr[0].h, self.colorMaskArr[0].s, self.colorMaskArr[0].v),
                        (self.colorMaskArr[1].h, self.colorMaskArr[1].s, self.colorMaskArr[1].v))
            c_1_mask = cv.inRange(hsv_frame,
                        (self.colorMaskArr[2].h, self.colorMaskArr[2].s, self.colorMaskArr[2].v),
                        (self.colorMaskArr[3].h, self.colorMaskArr[3].s, self.colorMaskArr[3].v))
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

################################################ Put OpenCV Processing Here

            ###display realtime live view of the camera
            if SHOW_REALTIME:
                cv.namedWindow("Camera %d"%(self.cameraID))
                cv.imshow("Camera %d"%(self.cameraID), frame)
                cv.waitKey(1)
            ## Send all data to main process here
            self.pipPosition.send([ball_0, ball_1])
            ###Clear eStart When finished Processing Frame
            self.eStart.clear()
        ######################### Main Loop #########################


        ##########################exist Process
        with self.lockPrint:
            print("Ending camera %d thread..."%(self.cameraID))
        capDev.release()
        with self.lockPrint:
            print("Camera %d thread ended."%(self.cameraID))






if __name__=='__main__':
    main()
