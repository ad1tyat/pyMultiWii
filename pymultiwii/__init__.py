#!/usr/bin/env python

"""Handles Multiwii Serial Protocol."""

__author__ = "Aldo Vargas"
__author__ = "William Koch"
__copyright__ = "Copyright 2017 Altax.net"

__license__ = "GPL"
__version__ = "1.6"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"


import serial, time, struct
import socket
from urllib.parse import urlparse
import abc
import asyncio
import logging
logger = logging.getLogger("pymultiwii")

class MultiwiiCommChannel(object, metaclass=abc.ABCMeta):
    @abc.abstractmethod
    def connect(self):
        """ Open connection """
        return 

    @abc.abstractmethod
    def close(self):
        """ Close connection """
        return 

    @abc.abstractmethod
    def write(self, message):
        """ Write the message to the channel """
        return 

    @abc.abstractmethod
    def read(self):
        """ Read data from channel """
        return 

    def is_checksum_valid(self, data, checksum):
        computed = 0
        for i in data :
            computed = computed ^ i
        return computed == checksum

class MultiWiiSerialChannel(MultiwiiCommChannel):

    def __init__(self, dstAddress, baudrate = 115200):
        self.ser = serial.Serial()
        self.ser.port = dstAddress
        self.ser.baudrate = baudrate
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = 0
        self.ser.xonxoff = False
        self.ser.rtscts = False
        self.ser.dsrdtr = False
        self.ser.writeTimeout = 2

    def connect(self):
        """Time to wait until the board becomes operational"""
        wakeup = 2
        #TODO Need to replace this with proper logging
        self.PRINT = 1
        try:
            self.ser.open()
            if self.PRINT:
                print("Waking up board on "+self.ser.port+"...")
            for i in range(1,wakeup):
                if self.PRINT:
                    print(wakeup-i)
                    time.sleep(1)
                else:
                    time.sleep(1)
        except Exception as error:
            print("\n\nError opening "+self.ser.port+" port.\n"+str(error)+"\n\n")

    def close(self):
        pass

    def write(self, message):
        return self.ser.write(message)

    def read(self):
        while True:
            header = self.ser.read()
            if header == b'$':
                header = header+self.ser.read(2)
                break
        datalength = struct.unpack('<b', self.ser.read())[0]
        code = struct.unpack('<b', self.ser.read())
        data = self.ser.read(datalength)
        temp = struct.unpack('<'+'h'*(int(datalength/2)),data)
        self.ser.flushInput()
        self.ser.flushOutput()
        return temp

class MultiwiiAsyncIOTCPChannel(MultiwiiCommChannel):
    def __init__(self, ipaddr, port):
        self.ipaddr = ipaddr
        self.port = port

        self.reader = None
        self.writter = None


    def connect(self):
        loop = asyncio.get_event_loop()
        loop.run_until_complete(self._connect(loop))
        loop.close()

    async def _connect(self, loop):
        self.reader, self.writer = await asyncio.open_connection(self.ipaddr, self.port, loop=loop)

    def close(self):
        pass

    def write(self, message):
        self.writer.write(message)

    def _parse_payload(self, payload):
        """ parse packet payload and return fields throw exception is parsing error """

        # Check for preamble
        logger.debug (payload[:2])
        if payload[:2] != b'$M':
            raise Exception("Preable not found")

        # Check direction, indexing bytesarray returns int
        direction = payload[2]
        if not (direction == ord('>') or direction == ord('<')):
            raise Exception("Direction not found")

        datalength = payload[3]
        logger.debug ("len={}".format(datalength))
        command = payload[4]
        logger.debug ("CMD={}".format(command))
        data = payload[5 : 5 + datalength] 
        logger.debug ("Data={}".format(data))
        crc = payload[5 + datalength]
        logger.debug ("CRC={}".format(crc))

        return (direction, datalength, command, data, crc)

    async def read(self):
        packet = await self.reader.read(1024)
        payload = bytes(packet)
        (direction, datalength, command, data, crc) = self._parse_payload(payload)

        # Check if any data has been corrupted
        logger.debug ("Size+CMD+DATA={}".format(payload[3: 3 + 1 + 1 + datalength]))
        if not self.is_checksum_valid(payload[3 : 3 + 1 + 1 + datalength], crc):
            raise Exception("Checksum not valid!")
        return data



class MultiwiiTCPChannel(MultiwiiCommChannel):
    # TODO Whats the max size we can expect to receive from the FC?
    BUFFER_SIZE = 1024

    def __init__(self, ipaddr, port):
        self.ipaddr = ipaddr
        self.port = port

        self.data_recv = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.connect((self.ipaddr, self.port))
        #self.sock.setblocking(0)

    def close(self):
        if not self.sock:
            raise Exception("Cannot close, socket never created")
        self.s.close()

    def write(self, message):
        if not self.sock:
            raise Exception("Cannot write, socket never created")
        self.sock.send(message)


        # The interface is the same as serial we will require this in 
        # in two steps for now. If there is a response, it will be cached 
        # and a read will have to be made
        # self.data_recv = self.s.recv(MultiwiiTCPChannel.BUFFER_SIZE)
    
    def _parse_payload(self, payload):
        """ parse packet payload and return fields throw exception is parsing error """

        # Check for preamble
        logger.debug (payload[:2])
        if payload[:2] != b'$M':
            raise Exception("Preable not found")

        # Check direction, indexing bytesarray returns int
        direction = payload[2]
        if not (direction == ord('>') or direction == ord('<')):
            raise Exception("Direction not found")

        datalength = payload[3]
        logger.debug ("len={}".format(datalength))
        command = payload[4]
        logger.debug ("CMD={}".format(command))
        data = payload[5 : 5 + datalength] 
        logger.debug ("Data={}".format(data))
        crc = payload[5 + datalength]
        logger.debug ("CRC={}".format(crc))

        return (direction, datalength, command, data, crc)

    def read(self):
        """ Read and return data in bytes 
        General format:
            <preamble>,<direction>,<size>,<command>,data,<crc>
        """


        # Look for preamble
        start = time.time()
        """
        header = None
        while True:
            try:
                header = self.sock.recv(1)
            except:
                pass
            print("header=", header)
            if header and header == b'$':
                break

        packet = None
        #try:
        packet = header + self.sock.recv(MultiwiiTCPChannel.BUFFER_SIZE)
        """
        packet =  self.sock.recv(MultiwiiTCPChannel.BUFFER_SIZE)
        #except:
        #    return
        payload = bytes(packet)
        logger.debug("Lapse = {}".format(time.time() - start))
        logger.debug ("Payload= {}".format(payload))


        (direction, datalength, command, data, crc) = self._parse_payload(payload)

        # Check if any data has been corrupted
        logger.debug ("Size+CMD+DATA={}".format(payload[3: 3 + 1 + 1 + datalength]))
        if not self.is_checksum_valid(payload[3 : 3 + 1 + 1 + datalength], crc):
            raise Exception("Checksum not valid!")

        return data

         
class MultiWii:

    """Multiwii Serial Protocol message ID"""
    """ notice: just attitude, rc channels and raw imu, set raw rc are implemented at the moment """
    IDENT = 100
    STATUS = 101
    RAW_IMU = 102
    SERVO = 103
    MOTOR = 104
    RC = 105
    RAW_GPS = 106
    COMP_GPS = 107
    ATTITUDE = 108
    ALTITUDE = 109
    ANALOG = 110
    RC_TUNING = 111
    PID = 112
    BOX = 113
    MISC = 114
    MOTOR_PINS = 115
    BOXNAMES = 116
    PIDNAMES = 117
    WP = 118
    BOXIDS = 119
    RC_RAW_IMU = 121
    SET_RAW_RC = 200
    SET_RAW_GPS = 201
    SET_PID = 202
    SET_BOX = 203
    SET_RC_TUNING = 204
    ACC_CALIBRATION = 205
    MAG_CALIBRATION = 206
    SET_MISC = 207
    RESET_CONF = 208
    SET_WP = 209
    SWITCH_RC_SERIAL = 210
    IS_SERIAL = 211
    DEBUG = 254


    """Class initialization"""
    def __init__(self, fc_address):

        """Global variables of data"""
        self.PIDcoef = {'rp':0,'ri':0,'rd':0,'pp':0,'pi':0,'pd':0,'yp':0,'yi':0,'yd':0}
        self.rcChannels = {'roll':0,'pitch':0,'yaw':0,'throttle':0,'elapsed':0,'timestamp':0}
        self.rawIMU = {'ax':0,'ay':0,'az':0,'gx':0,'gy':0,'gz':0,'mx':0,'my':0,'mz':0,'elapsed':0,'timestamp':0}
        self.motor = {'m1':0,'m2':0,'m3':0,'m4':0,'elapsed':0,'timestamp':0}
        self.attitude = {'angx':0,'angy':0,'heading':0,'elapsed':0,'timestamp':0}
        self.altitude = {'estalt':0,'vario':0,'elapsed':0,'timestamp':0}
        self.message = {'angx':0,'angy':0,'heading':0,'roll':0,'pitch':0,'yaw':0,'throttle':0,'elapsed':0,'timestamp':0}
        self.temp = ();
        self.temp2 = ();
        self.elapsed = 0
        self.PRINT = 1

        self.channel = None
        parsed_address = urlparse(fc_address)
        if parsed_address.scheme == "tcp":
            self.channel = MultiwiiTCPChannel(parsed_address.hostname, parsed_address.port)
            #self.channel = MultiwiiAsyncIOTCPChannel(parsed_address.hostname, parsed_address.port)

        else:
            self.channel = MultiWiiSerialChannel(fc_address)
        
        #self.connect()

    def connect(self):
        """ Connect to the flight controller through the defined communication channel """
        self.channel.connect()

    def close(self):
        """ Close the connection to the flight controller """
        self.channel.close()

    def sendCMD(self, data_length, code, data):
        """Function for sending a command to the board"""

        checksum = 0
        total_data = [b'$', b'M', b'<', data_length, code] + data
        for i in struct.pack('<2B%dH' % len(data), *total_data[3:len(total_data)]):
            checksum = checksum ^ i
        total_data.append(checksum)
        #try:
        # Pack as little endian, 3 character, 2 unsigned char, length of data double float,
        self.channel.write(struct.pack('<3c2B%dHB' % len(data), *total_data))

        #self.channel.read()
        #except Exception as error:
        #    print("sendCMD error ", error)
            #print "("+str(error)+")\n\n"

    """Function for sending a command to the board and receive attitude"""
    """
    Modification required on Multiwii firmware to Protocol.cpp in evaluateCommand:

    case MSP_SET_RAW_RC:
      s_struct_w((uint8_t*)&rcSerial,16);
      rcSerialCount = 50; // 1s transition 
      s_struct((uint8_t*)&att,6);
      break;

    """

    def setRawRC(self, rcData):
        self.sendCMD(16, MultiWii.SET_RAW_RC, rcData)
        self.channel.read()

    def sendCMDreceiveATT(self, data_length, code, data):
        checksum = 0
        total_data = [b'$', b'M', b'<', data_length, code] + data
        for i in struct.pack('<2B%dH' % len(data), *total_data[3:len(total_data)]):
            checksum = checksum ^ i # ord(i)
        total_data.append(checksum)
        try:
            start = time.time()
            b = None
            b = self.channel.write(struct.pack('<3c2B%dHB' % len(data), *total_data))
            temp = self.channel.read(cmd)

            elapsed = time.time() - start
            self.attitude['angx']=float(temp[0]/10.0)
            self.attitude['angy']=float(temp[1]/10.0)
            self.attitude['heading']=float(temp[2])
            self.attitude['elapsed']=round(elapsed,3)
            self.attitude['timestamp']="%0.2f" % (time.time(),) 
            return self.attitude
        except Exception as error:
            #print "\n\nError in sendCMDreceiveATT."
            #print "("+str(error)+")\n\n"
            pass

    """Function to arm / disarm """
    """
    Modification required on Multiwii firmware to Protocol.cpp in evaluateCommand:

    case MSP_SET_RAW_RC:
      s_struct_w((uint8_t*)&rcSerial,16);
      rcSerialCount = 50; // 1s transition 
      s_struct((uint8_t*)&att,6);
      break;

    """
    
    def setPID(self,pd):
        nd=[]
        for i in np.arange(1,len(pd),2):
            nd.append(pd[i]+pd[i+1]*256)
        data = pd
        print("PID sending:",data)
        self.sendCMD(30,MultiWii.SET_PID,data)
        self.sendCMD(0,MultiWii.EEPROM_WRITE,[])

    async def _getData(self, cmd):
        print("sending ", cmd)
        await self.sendCMD(0, cmd, [])
        print("reading ")
        temp = await self.channel.read()
        print("Data received = ", temp)


    """Function to receive a data packet from the board"""
    def getData(self, cmd):
        #try:
        start = time.time()
        self.sendCMD(0, cmd, [])

        temp = self.channel.read()

        elapsed = time.time() - start
        if cmd == MultiWii.ATTITUDE:
            self.attitude['angx']=float(temp[0]/10.0)
            self.attitude['angy']=float(temp[1]/10.0)
            self.attitude['heading']=float(temp[2])
            self.attitude['elapsed']=round(elapsed,3)
            self.attitude['timestamp']="%0.2f" % (time.time(),) 
            return self.attitude
        elif cmd == MultiWii.ALTITUDE:
            self.altitude['estalt']=float(temp[0])
            self.altitude['vario']=float(temp[1])
            self.attitude['elapsed']=round(elapsed,3)
            self.attitude['timestamp']="%0.2f" % (time.time(),) 
            return self.rcChannels
        elif cmd == MultiWii.RC:
            self.rcChannels['roll']=temp[0]
            self.rcChannels['pitch']=temp[1]
            self.rcChannels['yaw']=temp[2]
            self.rcChannels['throttle']=temp[3]
            self.rcChannels['aux1']=temp[4]
            self.rcChannels['aux2']=temp[5]
            self.rcChannels['aux3']=temp[6]
            self.rcChannels['aux4']=temp[7]
            self.rcChannels['elapsed']=round(elapsed,3)
            self.rcChannels['timestamp']="%0.2f" % (time.time(),)
            return self.rcChannels
        elif cmd == MultiWii.RAW_IMU:
            self.rawIMU['ax']=float(temp[0])
            self.rawIMU['ay']=float(temp[1])
            self.rawIMU['az']=float(temp[2])
            self.rawIMU['gx']=float(temp[3])
            self.rawIMU['gy']=float(temp[4])
            self.rawIMU['gz']=float(temp[5])
            self.rawIMU['mx']=float(temp[6])
            self.rawIMU['my']=float(temp[7])
            self.rawIMU['mz']=float(temp[8])
            self.rawIMU['elapsed']=round(elapsed,3)
            self.rawIMU['timestamp']="%0.2f" % (time.time(),)
            return self.rawIMU
        elif cmd == MultiWii.MOTOR:
            self.motor['m1']=float(temp[0])
            self.motor['m2']=float(temp[1])
            self.motor['m3']=float(temp[2])
            self.motor['m4']=float(temp[3])
            self.motor['elapsed']="%0.3f" % (elapsed,)
            self.motor['timestamp']="%0.2f" % (time.time(),)
            return self.motor
        elif cmd == MultiWii.PID:
            dataPID=[]
            if len(temp)>1:
                d=0
                for t in temp:
                    dataPID.append(t%256)
                    dataPID.append(t/256)
                for p in [0,3,6,9]:
                    dataPID[p]=dataPID[p]/10.0
                    dataPID[p+1]=dataPID[p+1]/1000.0
                self.PIDcoef['rp']= dataPID=[0]
                self.PIDcoef['ri']= dataPID=[1]
                self.PIDcoef['rd']= dataPID=[2]
                self.PIDcoef['pp']= dataPID=[3]
                self.PIDcoef['pi']= dataPID=[4]
                self.PIDcoef['pd']= dataPID=[5]
                self.PIDcoef['yp']= dataPID=[6]
                self.PIDcoef['yi']= dataPID=[7]
                self.PIDcoef['yd']= dataPID=[8]
            return self.PIDcoef
        elif cmd == MultiWii.BOXIDS:
            return temp
        elif cmd == MultiWii.BOXNAMES:
            return temp
        elif cmd == MultiWii.STATUS:
            """ The MultiWii API is out of date with what is implemented by betaflight,
            https://github.com/betaflight/betaflight/blob/master/src/main/interface/msp.c#L696
            """
            

            # unpack up until flight modes becaues
            # this is dynamic
            # B=1
            # H=2
            # I=4
            unpacked = struct.unpack("<3HIB2HB", temp[:16])
            dt = unpacked[0]
            ic2_error_count = unpacked[1]
            sensors = unpacked[2]
            flight_mode_flags = unpacked[3]
            pid_profile_index = unpacked[4]
            system_load = unpacked[5]
            gyro_cycle_time = unpacked[6]
            size_conditional_flight_mode_flags = unpacked[7]

            unpacked2 = None
            if size_conditional_flight_mode_flags > 0:
                unpacked2 = struct.unpack("<{}BBI".format(size_conditional_flight_mode_flags), temp[16:])
# FIXME This is going to break, flags will be byte array
                conditional_flight_mode_flags = unpacked2[:size_conditional_flight_mode_flags]
                num_disarming_flags = unpacked2[size_conditional_flight_mode_flags ]
                arming_disabled_flags = unpacked2[size_conditional_flight_mode_flags + 1]
            else:
                unpacked2 = struct.unpack("<BI", temp[16:])

                num_disarming_flags = unpacked2[0]
                arming_disabled_flags = unpacked2[1]

            print ("Flight modes =", flight_mode_flags)
            print ("Arm disabled = ", arming_disabled_flags)

            return temp
        else:
            return "No return error!"
        #except Exception as error:
        #    print("getData erorr", error)

    """Function to receive a data packet from the board. Note: easier to use on threads"""
    def getDataInf(self, cmd):
        while True:
            self.getData(cmd)

    """Function to ask for 2 fixed cmds, attitude and rc channels, and receive them. Note: is a bit slower than others"""
    def getData2cmd(self, cmd):
        try:
            start = time.time()
            self.sendCMD(0,self.ATTITUDE,[])
            temp = self.channel.read(cmd)

            self.sendCMD(0,self.RC,[])
            temp2 = self.channel.read(cmd)

            elapsed = time.time() - start

            if cmd == MultiWii.ATTITUDE:
                self.message['angx']=float(temp[0]/10.0)
                self.message['angy']=float(temp[1]/10.0)
                self.message['heading']=float(temp[2])
                self.message['roll']=temp2[0]
                self.message['pitch']=temp2[1]
                self.message['yaw']=temp2[2]
                self.message['throttle']=temp2[3]
                self.message['elapsed']=round(elapsed,3)
                self.message['timestamp']="%0.2f" % (time.time(),) 
                return self.message
            else:
                return "No return error!"
        except Exception as error:
            print(error)
