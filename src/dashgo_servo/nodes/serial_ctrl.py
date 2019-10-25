#coding:utf8
# 这个代码封装了和舵机控制板交互的接口，可以直接调用，也可以使用对应的ros接口
# 详细的通信协议可以参考《二次开发串口通信协议》
# 简单来说，我们只需要set position和read position指令，但是因为并没有提供读取舵机当前位置的接口，所以只能默认舵机的实际运动是和指令一致的QAQ

# 版本：v1
# 功能：可以控制舵机的转动
from __future__ import print_function
import serial
import time
import numpy as np
from struct import pack
def messageGen(command,param):
    def int2hex(num):
        # example
        # num=55 return '38'
        # python2
        return pack('B',num).encode('hex')
        # python3
        # return pack('B',num).hex()

    if command == 'move':
        if len(param)%2 != 0:
            print('error length of move command')
            return b''
        servo_id = param[::2]
        angle = param[1::2]
        # 需要将angle转换为pwm形式
        # [0,180]->[500,1000]
        pwm_angle = np.array(angle)/180.0*2000.0+500
        # 根据通信协议生成hex格式的message
        message = ['55','55']
        cmd = '03'
        length = len(servo_id)*3+5
        message.append(int2hex(length))
        message.append(cmd)
        message.append(int2hex(len(servo_id)))
        # 因为时间参数是bound所有舵机的，所有取时间最长的舵机
        # 但是店家的理论速度也太tm快了，所以我老实点给666ms吧
        # servo_vel = 60/0.16
        # servo_time = max(angle))/servo_vel
        servo_time = 666
        servo_time = int(servo_time)
        # 所有的数据都应该转换为hex形式
        # 使用struct.pack('B',num)来将unsigned char(uint8)转换为hex形式
        servo_time_l = servo_time%256
        servo_time_h = servo_time//256
        message.extend([int2hex(servo_time_l),int2hex(servo_time_h)])
        for i in range(len(servo_id)):
            package = [int2hex(int(servo_id[i]))]
            pwm_angle_l = int(pwm_angle[i]%256)
            pwm_angle_h = int(pwm_angle[i]//256)
            package.extend([int2hex(pwm_angle_l),int2hex(pwm_angle_h)])
            message.extend(package)
        message = ''.join(message)
        message = message.decode('hex')
        return message
    elif command == 'voltage':
        print('will be added in v2 distribution')
        return b''
    elif command == 'move_group':
        print('I dont think we need it')
        return b''
    else:
        print('error command')
        return b''
    

class servoCtrl():
    def __init__(self,port='/dev/ttyTHS2',baud=9600):
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(self.port,self.baud)
    
    # 关于serial.Serial 类的write方法：
    # 对比python2和python3的代码
    # 可以发现write的参数应该是bytes类型的
    # python3 bytes和string类型是严格区分的，但是python2 bytes类型就是string类型的
    # 最直接的就是使用b'\x55\x55'这样开头的数据
    # 但是还有一个更简单的办法：
    # python2: '5555'.decode('hex')='\x55\x55'='UU'
    # python3: bytes.fromhex('5555')=b'\x55\x55'='UU'
    def setPosition(self,position):
        message = messageGen('move',position)
        if self.ser.isOpen() == False:
            self.ser.open()
        self.ser.write(message)

    def stop(self):
        if self.ser.isOpen == True:
            self.ser.close()
        print('serial port is closed')

if __name__ == "__main__":
    servo_ctrl = servoCtrl()
    angle = [1,60]
    servo_ctrl.setPosition(angle)
    # 等待舵机运动结束
    time.sleep(0.666)
    print('servo ',angle[::2],'rotate ',angle[1::2],'degree')
    
            
