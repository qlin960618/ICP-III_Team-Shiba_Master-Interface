import os
import numpy as np
from multiprocessing import Process, Queue
from queue import Empty
from enum import Enum
import time
import tkinter as tk
from functools import partial

import PIL.Image
import PIL.ImageTk
import cv2 as cv

FRAME_TIME = 1.0 / 20
IM_RESOLUTION = 960

TESTING = False

GRID_X_SIZE=21
GRID_Y_SIZE=11
GRID_N_POINTS=GRID_X_SIZE*GRID_Y_SIZE
GRID_DEFAULT_SIZE=5

CONFIG_FILE_NAME="lens_mapping.txt"


class FramePreProcessing(Enum):
    Nothing = 0
    CenterDot = 1
    CrossLine = 2
    CursorPos = 3
    MarkerPos = 4


def main():
    frameQueue = Queue()
    directiveQueue = Queue()
    preProcessingQueue = Queue()

    if not TESTING:
        camera_p = Process(target=CameraStreamerThread, args=(0, frameQueue, directiveQueue, preProcessingQueue))
        camera_p.start()

    App = CalibrationApp(tk.Tk(), frameQueue, directiveQueue, preProcessingQueue)

    # time.sleep(20)
    #ensure exit of video capture is safe
    if not TESTING:
        directiveQueue.put('exit')
        camera_p.terminate()

    print('successful in exiting program')


class CalibrationApp():
    def __init__(self, masterWindow, frameQueue, directiveQueue, preProcessingQueue):
        self.cnt = 0

        self.masterWindow = masterWindow
        self.frameQueue = frameQueue
        self.directiveQueue = directiveQueue
        self.preProcessingQueue = preProcessingQueue

        # self.read_configuration()
        # return

        if TESTING:
            self.vidWidth = 1280
            self.vidHeight = 720
        else:
            self.vidWidth = self.frameQueue.get(block=True)
            self.vidHeight = self.frameQueue.get(block=True)
        self.vidAspect = float(self.vidWidth) / self.vidHeight
        print("w: %d, h: %d" % (self.vidWidth, self.vidHeight))

        self.masterWindow.title("Camera Calibration UI")

        videoFrame = tk.Frame(self.masterWindow)
        videoFrame.grid(row=0, column=0)

        # self.videoCanvas = tk.Canvas(videoFrame, width=self.vidWidth, height=self.vidHeight)
        self.videoCanvas = tk.Canvas(videoFrame, width=IM_RESOLUTION, height=int(IM_RESOLUTION / self.vidAspect))
        self.videoCanvas.grid(row=0, column=0, columnspan=3)
        self.photo = None
        self.videoCanvas.bind("<B1-Motion>", self.process_mouse_event)
        self.videoCanvas.bind("<Button-1>", self.process_mouse_event)
        self.showCursorOnCanvas = False

        self.captureFrameBtn = tk.Button(videoFrame, text="Capture Frame", fg='red', command=self.freeze_frame)
        self.freezeVidFrame = False
        self.captureFrameBtn.grid(row=1, column=0)

        self.saveFrameBtn = tk.Button(videoFrame, text="Save", command=self.save_frame, width=25)
        self.saveFrameBtn.grid(row=1, column=1)
        self.loadFrameBtn = tk.Button(videoFrame, text="Load", command=self.load_frame, width=25)
        self.loadFrameBtn.grid(row=1, column=2)

        self.framePathVar = tk.StringVar()
        self.framePathVar.set("./tmp.bmp")
        self.framePathEntry = tk.Entry(videoFrame, textvariable=self.framePathVar, width=60)
        self.framePathEntry.grid(row=2, column=1, columnspan=2)

        calibrationFrame = tk.Frame(self.masterWindow)
        calibrationFrame.grid(row=0, column=1)

        calibrationFrameTitle = tk.Label(calibrationFrame, text="Parameter",
                    bg='yellow', fg='black', font=("Courier bold", 16), width=30)
        calibrationFrameTitle.grid(row=0, column=0, columnspan=6)


        calImgSizeLabel = tk.Label(calibrationFrame, text="Grid Size: ")
        calImgSizeLabel.grid(row=1, column=0, columnspan=2)
        self.calImgSizeVar = tk.StringVar()
        self.calImgSizeEntry = tk.Entry(calibrationFrame, textvariable=self.calImgSizeVar, width=10)
        self.calImgSizeEntry.grid(row=1, column=2, columnspan=2)
        self.calImgSizeEntry.bind("<Key>", partial(self.validate_float_Entry, self.calImgSizeVar))
        self.calImgSizeVar.set("%.3f"%GRID_DEFAULT_SIZE)
        calImgSizeUnitLabel = tk.Label(calibrationFrame, text="cm")
        calImgSizeUnitLabel.grid(row=1, column=4)

        calImgDistLabel = tk.Label(calibrationFrame, text="Distance to Image: ")
        calImgDistLabel.grid(row=2, column=0, columnspan=2)
        self.calImgDistVar = tk.StringVar()
        self.calImgDistEntry = tk.Entry(calibrationFrame, textvariable=self.calImgDistVar, width=10)
        self.calImgDistEntry.grid(row=2, column=2, columnspan=2)
        self.calImgDistEntry.bind("<Key>", partial(self.validate_float_Entry, self.calImgDistVar))
        self.calImgDistVar.set("%.3f"%20)
        calImgDistUnitLabel = tk.Label(calibrationFrame, text="cm")
        calImgDistUnitLabel.grid(row=2, column=4)

        calInstructionLabel = tk.Label(calibrationFrame, text="click on image location according to the grid index "
                    + "display in the x y field (Counted from center)", wraplength=350,
                    borderwidth=2, relief="solid")
        calInstructionLabel.grid(row=3, column=0, columnspan=6)

        xGridIndLabel = tk.Label(calibrationFrame, text="X Index: ")
        xGridIndLabel.grid(row=5, column=0)
        self.xGridIndValLabel = tk.Label(calibrationFrame, text="0",
                    bg='yellow', fg='blue', width=4)
        self.xGridIndValLabel.grid(row=5, column=1)
        yGridIndLabel = tk.Label(calibrationFrame, text="Y Index: ")
        yGridIndLabel.grid(row=6, column=0)
        self.yGridIndValLabel = tk.Label(calibrationFrame, text="0",
                    bg='yellow', fg='blue', width=4)
        self.yGridIndValLabel.grid(row=6, column=1)

        xGridPosLabel = tk.Label(calibrationFrame, text="X Position: ")
        xGridPosLabel.grid(row=5, column=2)
        self.xGridPosValLabel = tk.Label(calibrationFrame, text="0",
                    bg='yellow', fg='blue', width=4)
        self.xGridPosValLabel.grid(row=5, column=3)
        yGridPosLabel = tk.Label(calibrationFrame, text="Y Position: ")
        yGridPosLabel.grid(row=6, column=2)
        self.yGridPosValLabel = tk.Label(calibrationFrame, text="0",
                    bg='yellow', fg='blue', width=4)
        self.yGridPosValLabel.grid(row=6, column=3)

        self.gridIndList = [ [i%GRID_X_SIZE-int(GRID_X_SIZE/2),
                    int(i/GRID_X_SIZE)-int(GRID_Y_SIZE/2) ]
                    for i in range(GRID_N_POINTS)]
        self.gridPosList = [ [0, 0] for i in range(GRID_N_POINTS)]
        self.currentGridListCnt = int(GRID_N_POINTS/2)

        self.gridCntLabel = tk.Label(calibrationFrame, text="p ID: %d"%self.currentGridListCnt,
                    bg='green', fg='blue', width=10)
        self.gridCntLabel.grid(row=4, column=0)
        self.gridCntNextBtn = tk.Button(calibrationFrame, text="Next", width=10,
                    command=partial(self.rotate_grid_cnt, 1))
        self.gridCntNextBtn.grid(row=4, column=4, columnspan=2)
        self.gridCntLastBtn = tk.Button(calibrationFrame, text="Last", width=10,
                    command=partial(self.rotate_grid_cnt, -1))
        self.gridCntLastBtn.grid(row=4, column=2, columnspan=2)

        self.saveCalibrationBtn = tk.Button(calibrationFrame, text="Save to File",
                    command=self.save_configuration)
        self.saveCalibrationBtn.grid(row=7, column=0, columnspan=6)


        #read previous config
        self.read_configuration()
        ###starting main loop
        if TESTING:
            self.directiveQueue.put('load ./tmp.bmp')
            self.freeze_frame()
        else:
            self.directiveQueue.put('next')
        self.loop()

        self.masterWindow.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.masterWindow.mainloop()

    def save_configuration(self):
        try:
            gridSize = float(self.calImgSizeVar.get())
            gridDistance = float(self.calImgDistVar.get())
        except:
            print("failled to save")
            return

        with open(CONFIG_FILE_NAME,'w') as f_config:
            f_config.write("cofg %.6f,%.6f,%d,%d\n"%(gridSize, gridDistance,
                        GRID_X_SIZE, GRID_Y_SIZE))

            f_config.write("indX")
            for i in range(GRID_N_POINTS):
                f_config.write(",%d"%self.gridIndList[i][0])
            f_config.write("\nindY")
            for i in range(GRID_N_POINTS):
                f_config.write(",%d"%self.gridIndList[i][1])
            f_config.write("\nposX")
            for i in range(GRID_N_POINTS):
                f_config.write(",%d"%self.gridPosList[i][0])
            f_config.write("\nposY")
            for i in range(GRID_N_POINTS):
                f_config.write(",%d"%self.gridPosList[i][1])
            f_config.write("\n")

    def read_configuration(self):
        if not os.path.exists(CONFIG_FILE_NAME):
            print("File does not exist")
            return

        with open(CONFIG_FILE_NAME,'r') as f_config:
            confStr = f_config.readline()[5:].strip().split(",")
            gridSize = float(confStr[0])
            gridDistance = float(confStr[1])
            print("Size: %.2f, dist: %.2f" % (gridSize, gridDistance))
            #read X Index
            xIndStr = f_config.readline()[5:].strip().split(",")
            yIndStr = f_config.readline()[5:].strip().split(",")
            xPosStr = f_config.readline()[5:].strip().split(",")
            yPosStr = f_config.readline()[5:].strip().split(",")
        if len(xIndStr) != GRID_N_POINTS:
            print("Config file format mismatch")
            return
        xInd = [int(i) for i in xIndStr]
        yInd = [int(i) for i in yIndStr]
        xPos = [int(i) for i in xPosStr]
        yPos = [int(i) for i in yPosStr]

        self.calImgSizeVar.set("%.3f"%gridSize)
        self.calImgDistVar.set("%.3f"%gridDistance)
        self.gridIndList = [ [xInd[i], yInd[i]] for i in range(GRID_N_POINTS)]
        self.gridPosList = [ [xPos[i], yPos[i]] for i in range(GRID_N_POINTS)]
        self.currentGridListCnt-=1
        self.rotate_grid_cnt(1)

    def rotate_grid_cnt(self, direction):
        newCnt = self.currentGridListCnt + direction
        if newCnt < 0 :
            newCnt = GRID_N_POINTS - 1
            print("Count Wrap Around: %d"%newCnt)
        if newCnt >= GRID_N_POINTS:
            newCnt = 0
            print("Count Wrap Around: %d"%newCnt)
        self.gridCntLabel.config(text="p ID: %d"%newCnt)

        # self.gridIndList
        # self.gridPosList
        self.xGridIndValLabel.config(text="%d"%self.gridIndList[newCnt][0])
        self.yGridIndValLabel.config(text="%d"%self.gridIndList[newCnt][1])
        self.xGridPosValLabel.config(text="%d"%self.gridPosList[newCnt][0])
        self.yGridPosValLabel.config(text="%d"%self.gridPosList[newCnt][1])
        self.curX = self.gridPosList[newCnt][0]
        self.curY = self.gridPosList[newCnt][1]

        self.currentGridListCnt=newCnt

    def process_mouse_event(self, event):
        self.showCursorOnCanvas = True
        self.curX = int(event.x * self.vidWidth / IM_RESOLUTION)
        self.curY = int(event.y * self.vidHeight / IM_RESOLUTION * self.vidAspect)

        cnt=self.currentGridListCnt
        self.gridPosList[cnt][0] = self.curX
        self.gridPosList[cnt][1] = self.curY
        self.xGridPosValLabel.config(text="%d"%self.gridPosList[cnt][0])
        self.yGridPosValLabel.config(text="%d"%self.gridPosList[cnt][1])

    def freeze_frame(self):
        self.freezeVidFrame = not self.freezeVidFrame
        if self.freezeVidFrame:
            self.captureFrameBtn.config(fg='green')
        else:
            self.captureFrameBtn.config(fg='red')

    def save_frame(self):
        filePath = self.framePathVar.get()
        dirpath = os.path.dirname(filePath)
        filename = os.path.basename(filePath)
        if not os.path.isdir(dirpath):
            print("path name error")
            return
        print("Storing: %s at directory: %s" % (filename, dirpath))

        self.directiveQueue.put('save %s' % filePath)

    def load_frame(self):
        filePath = self.framePathVar.get()
        if not os.path.isfile(filePath):
            print("File does not exist")
            return
        print("Loading file: %s" % (filePath))

        self.directiveQueue.put('load %s' % filePath)

        if not self.freezeVidFrame:
            self.freeze_frame()

    def validate_float_Entry(self, handle, dc):
        try:
            setVal=0
            if type==1:
                setValStr=handle.get()
            else:
                setValStr=handle.get()
            if setValStr=="" or setValStr==".":
                return
            setVal=float(setValStr)
        except ValueError:
            print("Value entered is invalid")
            if type==1:
                handle.set(setValStr[:-1])
            else:
                handle.set(setValStr[:-1])

    def loop(self):
        try:
            if TESTING:
                self.frame = np.zeros(shape=(720, 1280, 3), dtype='uint8')
            else:
                self.frame = self.frameQueue.get(block=True, timeout=10)
        except Empty:
            print("queue Empty error")
            self.on_closing()

        # print("display %d"%self.cnt)
        # self.cnt = self.cnt + 1

        img = PIL.Image.fromarray(self.frame)
        self.photo = PIL.ImageTk.PhotoImage(image=img.resize((IM_RESOLUTION, int(IM_RESOLUTION / self.vidAspect))))
        self.videoCanvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
        self.videoCanvas.update()

        # print(self.frame.shape)

        self.masterWindow.after(int(FRAME_TIME * 1000), self.loop)

        ### Putting queue for how to process next frame
        self.preProcessingQueue.put([FramePreProcessing.CrossLine, None])
        for i in range(GRID_N_POINTS):
            if i != self.currentGridListCnt:

                self.preProcessingQueue.put([FramePreProcessing.MarkerPos,
                            [ self.gridPosList[i][0], self.gridPosList[i][1] ]])
        if self.showCursorOnCanvas:
            self.preProcessingQueue.put([FramePreProcessing.CursorPos, [self.curX, self.curY]])

        time.sleep(0.001)
        if self.freezeVidFrame:
            self.directiveQueue.put('hold')
        else:
            self.directiveQueue.put('next')

    def on_closing(self):
        self.masterWindow.destroy()


def CameraStreamerThread(devID, frameQueue, directiveQueue, preProcessingQueue):
    if os.name == 'nt':
        print("Windows env")
        capDev = cv.VideoCapture(devID)
    elif os.name == 'posix':
        print("Unix env")
        capDev = cv.VideoCapture(devID)
    else:
        print("os mismatch: %s" % os.name)
        exit(1)

    if not capDev.isOpened():
        print("cannot open camera")
        exit()

    capDev.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
    capDev.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
    vidWidth = capDev.get(cv.CAP_PROP_FRAME_WIDTH)
    vidHeight = capDev.get(cv.CAP_PROP_FRAME_HEIGHT)

    frameQueue.put(vidWidth)
    frameQueue.put(vidHeight)

    ret, frameRAW = capDev.read()
    frameGrey = cv.cvtColor(frameRAW, cv.COLOR_BGR2GRAY)
    frame = cv.cvtColor(frameGrey, cv.COLOR_GRAY2RGB)

    cnt = 0
    directive = None
    usePreviousFrame = False
    oldFrame = frame

    ##################################### Main Loop ##############################
    while True:
        time.sleep(0.001)
        directive = directiveQueue.get(block=True)
        try:
            direcList = directive.split(" ", 1)
        except:
            continue
        if direcList[0] == 'exit':
            break
        elif direcList[0] == 'next':
            usePreviousFrame = False
        elif direcList[0] == 'hold':
            usePreviousFrame = True
        elif direcList[0] == 'save':
            img = PIL.Image.fromarray(oldFrame)
            img.save(direcList[1], "BMP")
        elif direcList[0] == 'load':
            with PIL.Image.open(direcList[1]) as im:
                oldFrame = np.asarray(im)
            usePreviousFrame = True
        else:
            continue

        preProcessItems = []
        while not preProcessingQueue.empty():
            preProcessItems.append(preProcessingQueue.get(block=False))

        if usePreviousFrame:
            frame = np.copy(oldFrame)
        else:
            ret, frameRAW = capDev.read()
            frameGrey = cv.cvtColor(frameRAW, cv.COLOR_BGR2GRAY)
            frame = cv.cvtColor(frameGrey, cv.COLOR_GRAY2RGB)
            oldFrame = np.copy(frame)

        for item in preProcessItems:
            dotSize1 = 3
            lineSize = 2
            dotSize2 = 5
            if item[0] == FramePreProcessing.CenterDot:
                frame[int(vidHeight / 2) - dotSize1:int(vidHeight / 2) + dotSize1,
                int(vidWidth / 2) - dotSize1:int(vidWidth / 2) + dotSize1, 0] = 255
            if item[0] == FramePreProcessing.CrossLine:
                frame[int(vidHeight / 2) - lineSize:int(vidHeight / 2) + lineSize, :, 2] = 255
                frame[:, int(vidWidth / 2) - lineSize:int(vidWidth / 2) + lineSize, 2] = 255
                frame[int(vidHeight / 2) - lineSize:int(vidHeight / 2) + lineSize, :, 1] = 0
                frame[:, int(vidWidth / 2) - lineSize:int(vidWidth / 2) + lineSize, 1] = 0
                frame[int(vidHeight / 2) - lineSize:int(vidHeight / 2) + lineSize, :, 0] = 0
                frame[:, int(vidWidth / 2) - lineSize:int(vidWidth / 2) + lineSize, 0] = 0
            if item[0] == FramePreProcessing.CursorPos:
                frame[item[1][1] - dotSize1:item[1][1] + dotSize1,
                item[1][0] - dotSize1:item[1][0] + dotSize1, 2] = 0
                frame[item[1][1] - dotSize1:item[1][1] + dotSize1,
                item[1][0] - dotSize1:item[1][0] + dotSize1, 1] = 0
                frame[item[1][1] - dotSize1:item[1][1] + dotSize1,
                item[1][0] - dotSize1:item[1][0] + dotSize1, 0] = 255
            if item[0] == FramePreProcessing.MarkerPos:
                frame[item[1][1] - dotSize2:item[1][1] + dotSize2,
                item[1][0] - dotSize2:item[1][0] + dotSize2, 2] = 0
                frame[item[1][1] - dotSize2:item[1][1] + dotSize2,
                item[1][0] - dotSize2:item[1][0] + dotSize2, 1] = 255
                frame[item[1][1] - dotSize2:item[1][1] + dotSize2,
                item[1][0] - dotSize2:item[1][0] + dotSize2, 0] = 0

        if not ret:
            print("failed to recive frame")
            exit()

        cnt = cnt + 1
        frameQueue.put(frame)

    ##################################### Main Loop ##############################
    print("ending thread...")
    capDev.release()


if __name__ == '__main__':
    main()
