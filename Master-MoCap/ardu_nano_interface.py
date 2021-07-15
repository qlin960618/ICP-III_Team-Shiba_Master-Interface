import serial.tools.list_ports as serialPortList
import ctypes
import multiprocessing as mp
from ctypes import Structure, c_int
from multiprocessing.sharedctypes import Array
import time
import serial

import numpy as np

SERIAL_PORT = "COM5"
SERIAL_PID = 29987


MIN_INTERVAL=50
VERBOSE_LEVEL=0 #debugger
INITIAL_SLEEP=5	#time fro connection to arduino ready



def main():

    master = SerialMasterInterface()
    master.list_all_port()
    path = master.get_path_from_pid(SERIAL_PID)
    print("making connection to %s"%path)
    master.connect_master(path)

    for i in range(1000):
        print(master.get_data())
        time.sleep(0.1)

    #Exiting Program
    master.reset()

class DataStruc(Structure):
    _fields_ = [('pot1', c_int), ('pot2', c_int), ('pot3', c_int),
                ('pot4', c_int), ('pot5', c_int), ('sw', c_int)]

class SerialMasterInterface():
    ePortConnected = False
    serialLink = None

    # stored list of existing ports
    portsList = None
    n_portsList = 0

    """
    function: Constructor
    """
    def __init__(self, eError=None):

        self.ePortConnected = mp.Event()
        self.ePortConnected.clear()
        self.lockData = mp.Lock()
        if eError is None:
            eError=mp.Event()
            eError.clear()
        self.eError = eError
        self.Data=[0,0,0,0,0,0]
        self.sharedDataArray = mp.sharedctypes.Array(DataStruc, [tuple(self.Data)],
                                                     lock=self.lockData)
        self.serialLinkThreadHandler = None

        self.portsList = None
        self.n_portsList = 0
        # self.lastTimeCalled=time.time()
        self.update_port_list()

    """
    function: Update List of ports
    return: list of port objects
    """

    def update_port_list(self):

        self.portsList = list(serialPortList.comports())
        self.n_portsList = len(self.portsList)

        return self.portsList

    """
    function: return device by HWID search
    return: None or device path
    """

    def get_path_from_pid(self, PIDin):
        #get updated port list
        self.update_port_list()
        if self.portsList == None:
            print("Master: Port list not yet updated.")
            self.update_port_list()

        i_stored = -1
        for i in range(self.n_portsList):
            if self.portsList[i].pid == PIDin:
                if i_stored != -1:
                    print("Master: Multiple match found")
                    return None
                i_stored = i

        # match found
        if i_stored >= 0:
            return self.portsList[i_stored].device

        print("Master: PID Not found.")
        return None

    """
    function: print out all existing port on this computer
    return: all port object
    """

    def list_all_port(self):
        if self.portsList == None:
            print("Master: Port list not yet updated.")
            self.update_port_list()

        print("Master: Listing ports...")
        print("\tPath\tVID\tPID")
        for i in range(self.n_portsList):
            print("\t%s\t%s\t%s" % (self.portsList[i].device.ljust(31), self.portsList[i].vid, self.portsList[i].pid))

        return self.portsList

    """
    Function: Initiate connection
    return: true for successful connection, false for failed
    """

    def connect_master(self, devPath):
        if self.ePortConnected.is_set():
            print("Master: port already open, clearing now")
            self.reset()

        self.serialLinkThreadHandler = SerialDeviceHandler(devPath, self.ePortConnected, self.sharedDataArray,
                                                           self.lockData, self.eError)
        self.serialLinkThreadHandler.start()

        # short sleep for arduino to ready
        time.sleep(INITIAL_SLEEP)

        if not self.ePortConnected.is_set():
            print("Master: Failed to grab connected signal")
            self.reset()
            return False

        return True

    def get_data(self):
        values=[self.sharedDataArray[0].pot1, self.sharedDataArray[0].pot2,
                self.sharedDataArray[0].pot3, self.sharedDataArray[0].pot4,
                self.sharedDataArray[0].pot5, self.sharedDataArray[0].sw]
        return values

    def get_status(self):
        return self.ePortConnected.is_set()

    # reset only serial connection
    def reset(self):
        self.ePortConnected.clear()
        if self.serialLinkThreadHandler is None:
            return
        self.serialLinkThreadHandler.join(10)

        if not self.serialLinkThreadHandler.is_alive():
            print("Master: Serial Handler Thread Joined")
        else:
            self.serialLinkThreadHandler.terminate()
            print("Master: Serial Handler Thread Failed to Join, Force Terminate")
        self.serialLinkThreadHandler=None

    # deep reset
    def reset_all(self):
        self.portsList = None
        self.n_portsList = 0
        self.reset()


class SerialDeviceHandler(mp.Process):
    def __init__(self, devPath, ePortConnected, sharedDataArray, lockData, eError):

        #save device path
        self.devPath=devPath
        # Storing Information
        self.ePortConnected = ePortConnected
        self.eError = eError
        self.sharedDataArray = sharedDataArray
        self.lockData = lockData
        # ##Launch Process
        super(SerialDeviceHandler, self).__init__()

    def run(self):
        try:
            serialDev = serial.Serial(self.devPath, 9600, timeout=0.1, parity=serial.PARITY_NONE,
                                           stopbits=serial.STOPBITS_ONE,
                                           bytesize=serial.EIGHTBITS)
        except (OSError, serial.SerialException, ValueError) as err:
            print("Serial: Connection Failed")
            self.eError.set()
            self.ePortConnected.clear()
            return

        ############ internal function
        def test_connection():
            try:
                if not serialDev.is_open:
                    return False
            except AttributeError:
                return False
            return True

        def close():
            try:
                serialDev.close()
            except:
                pass
            self.eError.set()
            self.ePortConnected.clear()

        ############ start of run
        ret = True
        for i in range(20):
            ret = test_connection()
            if ret:
                break
            else:
                time.sleep(0.1)
        if not ret:
            print("Serial: Connection Dropped")
            close()
            return
        ##start of loop
        self.ePortConnected.set()
        while self.ePortConnected.is_set():
            #test if connection is still alive
            if not test_connection():
                break
            #test if there is error
            if self.eError.is_set():
                print("Serial: Encountered error somethere else: quitting")
                break
            #check buffer length
            if serialDev.in_waiting>=100:
                serialDev.flush()

            RawData=serialDev.readline(50)
            try:
                RawData=RawData.decode("utf-8")
            except UnicodeDecodeError:
                continue

            RawData=RawData.strip().split(" ")
            if len(RawData)!=8:
                continue
            if RawData[0]!= 'ss' and RawData[7]!= 'ee':
                continue
            data = [0 for i in range(6)]
            for i in range(len(RawData[1:-1])):
                data[i]=int(RawData[i+1], base=10)

            self.sharedDataArray[0]=tuple(data)

        #existing while loop
        close()
        return

if __name__ == '__main__':
    main()
