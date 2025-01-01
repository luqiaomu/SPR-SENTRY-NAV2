import cv2 as cv
import cv2
import serial
import serial.tools.list_ports
import numpy as np

import time

class SerialPort:

    

    # 串口打开函数
    def open_port(self):
        self.ports_list = list(serial.tools.list_ports.comports())
        port = '/dev/ttyUSB0'  # 串口号
        baudrate = 115200  # 波特率
        try:
            self.ser = serial.Serial(port, baudrate, timeout=2)
            if self.ser.isOpen() == True:
                print("串口打开成功")
        except Exception as exc:
            print("串口打开异常", exc)


    def port_clean(self):
        self.ser.flushInput()#清理缓冲区数据
        
 
    # 关闭串口
    def close_port(self):
    
        try:
            self.ser.close()
            if self.ser.isOpen():
                print("串口未关闭")
            else:
                print("串口已关闭")
        except Exception as exc:
            print("串口关闭异常", exc)


    #发送角度
    def sendAngle(self,angle):#目前采用的弧度制

        angle1=angle[0]#yaw弧度
        angle2=angle[1]#pitch弧度
        angle_1=int((angle[0]+3.14)*10000)#yaw角度放大后的数据，保留4位小数,再精细的就没必要了
        angle_2=int((angle[1]+3.14)*10000)#pitch角度放大

        #存放高八位和低八位数据的字符数组
        data_unsigned =[0XFF,0,0,0,0,0,0,0XFE]

        #第一组数据的高八位和低八位
        data_unsigned[1]=(angle_1>>8)#取高八位,将高八位的数据对应的字符发出去
        data_unsigned[2]=(angle_1 & 0x00ff)#取低八位
        
        print((angle_1>>8))
        
        # #第二组数据的高八位和低八位
        # data_unsigned[3]=(angle_2>>8)
        # data_unsigned[4]=(angle_2 & 0x00ff)

        print(data_unsigned)


        try:
            send_datas = data_unsigned#需要发送的数据，此处直接发送列表数据，不需要设置编码格式，如果发送字符串需要制定编码格式
            # ser.write(str(send_datas+'\r\n').encode("utf-8"))
            self.ser.write(send_datas)
            print('-' * 80)
            print("已发送数据:")
            print(send_datas)
            print('-' * 80)

        except Exception as exc:
            print("发送异常", exc)

    def msg_read(self):
        # while True:
            # if self.serial_isopen:
                try:
                    self.ser.flushInput()
                    # 读取串口数据
                    data = []
                    data = self.ser.read(7)
                    print('#####')
                    if data[0] == 0xff and data[6] == 0xfe:
                        print(f'send goal:{data[1]}')

                except Exception as exc:
                    print("receive error", exc)

    def sendTwist(self, linear_x, angular_z):#目前采用的弧度制
        linear_x = int(linear_x)
        angular_z = int(angular_z)
       
        data =[0XFF,0,43,0,0,0,0,0XFE]

        
        data[1]=linear_x
        data[2]=angular_z
        

        
        # #第二组数据的高八位和低八位
        # data[3]=(angular_z>>8)
        # data[4]=(angular_z & 0x00ff)

        print(data)


        try:
            send_datas = data#需要发送的数据，此处直接发送列表数据，不需要设置编码格式，如果发送字符串需要制定编码格式
            # ser.write(str(send_datas+'\r\n').encode("utf-8"))
            self.ser.write(data)
            print('-' * 80)
            print("已发送数据:")
            print(data)
            print('-' * 80)

        except Exception as exc:
            print("发送异常", exc)
    

if __name__ == "__main__":
    #实例化串口类
    port=SerialPort()
    port.open_port()

    angle=[0.1234,0.5678]
    while True:
        port.sendTwist(40, 43)
        time.sleep(0.05)

    port.close_port()
