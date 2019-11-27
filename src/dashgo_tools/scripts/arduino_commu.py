#coding:utf8

# arduino的底盘通信类
import os, time
import thread

from math import pi as PI, degrees, radians, sin, cos
import os
import time
import sys, traceback
from serial.serialutil import SerialException
from serial import Serial

class Arduino:
    ''' Configuration Parameters
    '''    
    N_ANALOG_PORTS = 6
    N_DIGITAL_PORTS = 12
    
    def __init__(self, port="/dev/dashgo", baudrate=115200, timeout=0.1):
        
        self.PID_RATE = 30 # Do not change this!  It is a fixed property of the Arduino PID controller.
        self.PID_INTERVAL = 1000 / 30
        
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.encoder_count = 0
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 30.
    
        # Keep things thread safe
        self.mutex = thread.allocate_lock()
            
        # An array to cache analog sensor readings
        self.analog_sensor_cache = [None] * self.N_ANALOG_PORTS
        
        # An array to cache digital sensor readings
        self.digital_sensor_cache = [None] * self.N_DIGITAL_PORTS
    
    def connect(self):
        try:
            print "Connecting to Arduino on port", self.port, "..."
            self.port = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.writeTimeout)
            # The next line is necessary to give the firmware time to wake up.
            time.sleep(1)
            test = self.get_baud()
            if test != self.baudrate:
                time.sleep(1)
                test = self.get_baud()   
                if test != self.baudrate:
                    raise SerialException
            print "Connected at", self.baudrate
            print "Arduino is ready."

        except SerialException:
            print "Serial Exception:"
            print sys.exc_info()
            print "Traceback follows:"
            traceback.print_exc(file=sys.stdout)
            print "Cannot connect to Arduino!"
            os._exit(1)

    def open(self): 
        ''' Open the serial port.
        '''
        self.port.open()

    def close(self): 
        ''' Close the serial port.
        '''
        self.port.close() 
    
    def send(self, cmd):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        self.port.write(cmd + '\r')

    def recv(self, timeout=0.5):
        timeout = min(timeout, self.timeout)
        ''' This command should not be used on its own: it is called by the execute commands   
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Arduino
        '''
        c = ''
        value = ''
        attempts = 0
        while c != '\r':
            c = self.port.read(1)
            value += c
            attempts += 1
            if attempts * self.interCharTimeout > timeout:
                return None

        value = value.strip('\r')

        return value
            
    def recv_ack(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        ack = self.recv(self.timeout)
        return ack == 'OK'

    def recv_int(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        value = self.recv(self.timeout)
        try:
            return int(value)
        except:
            return None

    def recv_array(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        try:
            values = self.recv(self.timeout * self.N_ANALOG_PORTS).split()
            return map(int, values)
        except:
            return []

    def execute(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning a single integer value.
        '''
        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd + '\r')
            value = self.recv(self.timeout)
            while attempts < ntries and (value == '' or value == 'Invalid Command' or value == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    value = self.recv(self.timeout)
                except:
                    print "Exception executing command: " + cmd
                attempts += 1
        except:
            self.mutex.release()
            print "Exception executing command: " + cmd
            value = None
        
        self.mutex.release()
        return int(value)

    def execute_array(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning an array.
        '''
        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd + '\r')
            values = self.recv_array()
            while attempts < ntries and (values == '' or values == 'Invalid Command' or values == [] or values == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    values = self.recv_array()
                except:
                    print("Exception executing command: " + cmd)
                attempts += 1
        except:
            self.mutex.release()
            print "Exception executing command: " + cmd
            raise SerialException
            return []
        
        try:
            values = map(int, values)
        except:
            values = []

        self.mutex.release()
        return values
        
    def execute_ack(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning True if response is ACK.
        '''
        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd + '\r')
            ack = self.recv(self.timeout)
            while attempts < ntries and (ack == '' or ack == 'Invalid Command' or ack == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    ack = self.recv(self.timeout)
                except:
                    print "Exception executing command: " + cmd
            attempts += 1
        except:
            self.mutex.release()
            print "execute_ack exception when executing", cmd
            print sys.exc_info()
            return 0
        
        self.mutex.release()
        return ack == 'OK'   
    
    def update_pid(self, Kp, Kd, Ki, Ko):
        ''' Set the PID parameters on the Arduino
        '''
        print "Updating PID parameters"
        cmd = 'u ' + str(Kp) + ':' + str(Kd) + ':' + str(Ki) + ':' + str(Ko)
        self.execute_ack(cmd)                          

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        return int(self.execute('b'));

    def get_encoder_counts(self):
        values = self.execute_array('e')
        if len(values) != 2:
            print "Encoder count was not 2"
            raise SerialException
            return None
        else:
            return values

    def reset_encoders(self):
        ''' Reset the encoder counts to 0
        '''
        return self.execute_ack('r')
    
    def drive(self, right, left):
        ''' Speeds are given in encoder ticks per PID interval
        '''
        return self.execute_ack('m %d %d' %(right, left))
    
    def drive_m_per_s(self, right, left):
        ''' Set the motor speeds in meters per second.
        '''
        left_revs_per_second = float(left) / (self.wheel_diameter * PI)
        right_revs_per_second = float(right) / (self.wheel_diameter * PI)

        left_ticks_per_loop = int(left_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)
        right_ticks_per_loop  = int(right_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)

        self.drive(right_ticks_per_loop , left_ticks_per_loop )
        
    def stop(self):
        ''' Stop both motors.
        '''
        self.drive(0, 0)

    def ping(self):
        values = self.execute_array('p')
        if len(values) != 5:
            print "ping count was not 5"
            raise SerialException
            return None
        else:
            return values

    def get_voltage(self):
        return self.execute('v');

    def get_emergency_button(self):
        return self.execute('j');

    def get_pidin(self):
        values = self.execute_array('i')
        if len(values) != 2:
            print "get_pidin count was not 2"
            raise SerialException
            return None
        else:
            return values

    def get_pidout(self):
        values = self.execute_array('f')
        if len(values) != 2:
            print "get_pidout count was not 2"
            raise SerialException
            return None
        else:
            return values

