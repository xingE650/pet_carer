#coding:utf8

from arduino_commu import Arduino
import time

# 简单模拟了一下ros节点和arduino的通信过程
def test_serial():
    controller = Arduino(port='/dev/dashgo',baudrate=115200,timeout=0.1)

    controller.connect()
    # 首先清除旧的编码器数据
    controller.reset_encoders()

    # 设置pid参数
    controller.update_pid(50,20,0,50)

    count = 100
    while count>0 :
        count -= 1
        # 得到超声波数据
        r0,r1,r2,r3,r4 = controller.ping()
        print("r0: " + str(r0)+"r1: " + str(r1) + "r2: " + str(r2) + "r3: " + str(r3)+ "r4: " + str(r4))

        # 读取当前的电压数值
        voltage_val = controller.get_voltage()*10
        print('voltage is: ',voltage_val)

        # 读取当前应急开关的状态
        emergencybt_val = controller.get_emergency_button()
        print("emergencybt_val=",emergencybt_val)

        # 获取当前编码器的数据
        left_enc,right_enc = controller.get_encoder_counts()
        print('left_enc: ',left_enc,' right_enc: ',right_enc)

        # 驱动底盘前进
        controller.drive(1,1)

        time.sleep(0.5)

    print('test finish')

if __name__ == "__main__":
    test_serial()
