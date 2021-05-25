import os
import queue
import numpy as np
from multiprocessing import Process, Queue
from queue import Empty
from enum import Enum
import time
import tkinter as tk

import PIL.Image
import PIL.ImageTk
import cv2 as cv

FRAME_TIME = 1.0 / 30


class FramePreProcessing(Enum):
    Nothing=0
    CenterDot=1
    CrossLine=2
    CursorPos=3



def main():
    frameQueue = Queue()
    directiveQueue = Queue()
    preProcessingQueue = Queue()

    camera_p = Process(target=CameraStreamerThread, args=(frameQueue, directiveQueue, preProcessingQueue))
    camera_p.start()

    App = CalibrationApp(tk.Tk(), frameQueue, directiveQueue, preProcessingQueue)

    # time.sleep(20)
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


        self.vidWidth = self.frameQueue.get(block=True)
        self.vidHeight = self.frameQueue.get(block=True)
        print("w: %d, h: %d" % (self.vidWidth, self.vidHeight))

        videoFrame = tk.Frame(self.masterWindow)
        videoFrame.pack(side=tk.LEFT)

        self.videoCanvas = tk.Canvas(videoFrame, width=self.vidWidth, height=self.vidHeight)
        # self.videoCanvas = tk.Canvas(videoFrame, width=640, height=480)
        self.videoCanvas.pack(side=tk.TOP)
        self.videoCanvas.bind("<B1-Motion>", self.process_mouse_event)
        self.showCursorOnCanvas=False

        self.captureBtn = tk.Button(videoFrame, text="Capture Frame", command=self.freeze_frame)
        self.freezeVidFrame=False
        self.captureBtn.pack(side=tk.TOP)



        ###starting main loop
        self.directiveQueue.put('next')
        self.loop()

        self.masterWindow.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.masterWindow.mainloop()

    def process_mouse_event(self, event):
        self.showCursorOnCanvas = True
        self.curX = event.x
        self.curY = event.y

    def freeze_frame(self):
        self.freezeVidFrame = not self.freezeVidFrame

    def loop(self):
        try:
            self.frame = self.frameQueue.get(block=True, timeout=1)
        except Empty:
            print("queue Empty error")
            self.on_closing()

        # print("display %d"%self.cnt)
        self.cnt = self.cnt+1

        self.photo = PIL.ImageTk.PhotoImage(image=PIL.Image.fromarray(self.frame))
        self.videoCanvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
        self.videoCanvas.update()

        # print(np.min(self.frame))

        self.masterWindow.after(int(FRAME_TIME*1000), self.loop)


        self.preProcessingQueue.put([FramePreProcessing.CrossLine, None])
        if self.showCursorOnCanvas:
            self.preProcessingQueue.put([FramePreProcessing.CursorPos, [self.curX, self.curY]])

        if self.freezeVidFrame:
            self.directiveQueue.put('hold')
        else:
            self.directiveQueue.put('next')


    def on_closing(self):
        self.masterWindow.destroy()


def CameraStreamerThread(frameQueue, directiveQueue, preProcessingQueue):
    if os.name == 'nt':
        print("Windows env")
        capDev = cv.VideoCapture(0)
    elif os.name == 'posix':
        print("Unix env")
        capDev = cv.VideoCapture(0)
    else:
        print("os mismatch: %s"%os.name)
        exit(1)

    if not capDev.isOpened():
        print("cannot open camera")
        exit()

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
        if directive == 'exit':
            break
        elif directive == 'next':
            usePreviousFrame = False

        elif directive == 'hold':
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
            dotSize = 3
            lineSize = 2
            if item[0] == FramePreProcessing.CenterDot:
                frame[int(vidHeight / 2) - dotSize:int(vidHeight / 2) + dotSize,
                      int(vidWidth / 2) - dotSize:int(vidWidth / 2) + dotSize, 0] = 255
            if item[0] == FramePreProcessing.CrossLine:
                frame[int(vidHeight / 2) - lineSize:int(vidHeight / 2) + lineSize,
                      :, 0] = 255
                frame[:,
                      int(vidWidth / 2) - lineSize:int(vidWidth / 2) + lineSize, 0] = 255
            if item[0] == FramePreProcessing.CursorPos:
                frame[item[1][1] - dotSize:item[1][1] + dotSize,
                      item[1][0] - dotSize:item[1][0] + dotSize, 2] = 255
                frame[item[1][1] - dotSize:item[1][1] + dotSize,
                      item[1][0] - dotSize:item[1][0] + dotSize, 1] = 0
                frame[item[1][1] - dotSize:item[1][1] + dotSize,
                      item[1][0] - dotSize:item[1][0] + dotSize, 0] = 0


        if not ret:
            print("failed to recive frame")
            exit()

        cnt=cnt+1
        frameQueue.put(frame)


    ##################################### Main Loop ##############################
    print("ending thread...")
    capDev.release()


if __name__ == '__main__':
    main()
