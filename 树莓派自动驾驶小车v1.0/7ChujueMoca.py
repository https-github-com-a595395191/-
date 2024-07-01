# coding:utf-8

# 加入摄像头模块，让小车实现自动循迹行驶
# 思路为：摄像头读取图像，进行二值化，将白色的赛道凸显出来
# 选择下方的一行像素，黑色为0，白色为255
# 找到白色值的中点
# 目标中点与标准中点（320）进行比较得出偏移量
# 根据偏移量来控制小车左右轮的转速
# 考虑了偏移过多失控->停止;偏移量在一定范围内->高速直行(这样会速度不稳定，已删)

import RPi.GPIO as GPIO
import time
# import cv2
import numpy as np
# from picamera import PiCamera, Color
from time import sleep
import serial
import smbus

#ADC操作
address = 0x48
A0 = 0x40
A1 = 0x41
A2 = 0x42
A3 = 0x43
bus = smbus.SMBus(1)

#串口操作
port = '/dev/ttyAMA0'
ser = serial.Serial(port,9600)
ser.flushInput()
ser.close()    #关闭serial 表示的串口
ser.open()    #打开串口
#ser.open()
print serial    #可查看当前串口的状态信息
ser.isOpen()    #当前串口是否已经打开
# 定义引脚
IN1 = 35
IN2 = 36
IN3 = 37
IN4 = 38
en1 = 31
en2 = 32

# 设置GPIO口为BOARD编号规范
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# 设置GPIO口为输出
GPIO.setup(IN1,GPIO.OUT)
GPIO.setup(IN2,GPIO.OUT)
GPIO.setup(IN3,GPIO.OUT)
GPIO.setup(IN4,GPIO.OUT)
GPIO.setup(en1,GPIO.OUT)
GPIO.setup(en2,GPIO.OUT)

# 设置PWM波,频率为500Hz
pwm1 = GPIO.PWM(en1, 50) 
pwm2 = GPIO.PWM(en2, 50)

# pwm波控制初始化
pwm1.start(0)
pwm2.start(0)

# center定义
center = 320

# 打开摄像头，图像尺寸640*480（长*高），opencv存储值为480*640（行*列）
#demoCamera = PiCamera()
#demoCamera.start_preview()    #打开摄像头预览
#demoCamera.annotate_background = Color('white')
#demoCamera.annotate_foreground = Color('red') 
#demoCamera.resolution = (640, 480)      #设置摄像头的分辨率
#demoCamera.framate = 60                 #设定摄像头的帧率
# cap = cv2.VideoCapture(0)
while (1):
#ADC操作
    bus.write_byte(address,A0)
    value = bus.read_byte(address)
    print(value)
    print("AOUT0:%1.3f  " %(value*3.3/255))
    adc0 = value*3.3/255
    
#    bus.write_byte(address,A1)  
#    value1 = bus.read_byte(address)
#    print(value1)
#    print("AOUT1:%1.3f  " %(value1*3.3/255))
#    adc1 = value1*3.3/255
    
#前进
    print ("前进")
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.HIGH)
    GPIO.output(IN3,GPIO.HIGH)
    GPIO.output(IN4,GPIO.LOW)
    if (adc0 > 1) :
        print ("右转")
        print(value)
        pwm1.ChangeDutyCycle(40)
        pwm2.ChangeDutyCycle(40)
        GPIO.output(IN1,GPIO.LOW)
        GPIO.output(IN2,GPIO.HIGH)
        GPIO.output(IN3,GPIO.LOW)
        GPIO.output(IN4,GPIO.LOW)
        sleep(1)
    if (adc1  > 1 ) :
        print ("左转")
        pwm1.ChangeDutyCycle(30)
        pwm2.ChangeDutyCycle(30)
        GPIO.output(IN1,GPIO.LOW)
        GPIO.output(IN2,GPIO.LOW)
        GPIO.output(IN3,GPIO.HIGH)
        GPIO.output(IN4,GPIO.LOW)
        sleep(1)
    
#    pwm1.ChangeDutyCycle(20)
#    pwm2.ChangeDutyCycle(30)
#    GPIO.output(IN1,GPIO.LOW)
#    GPIO.output(IN2,GPIO.HIGH)
#   GPIO.output(IN3,GPIO.HIGH)
#    GPIO.output(IN4,GPIO.LOW)
 
#     ret, frame = cap.read()
    # 转化为灰度图
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     cv2.imshow("img_gray",gray)
    # 大津法二值化
#     retval, dst = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
    # 膨胀，白区域变大
#     dst = cv2.dilate(dst, None, iterations=2)
    # # 腐蚀，白区域变小
    # dst = cv2.erode(dst, None, iterations=6)

    # 单看第400行的像素值
#     color = dst[400]
    # 找到白色的像素点个数
#     white_count = np.sum(color == 255)
    # 找到白色的像素点索引
#     white_index = np.where(color == 255)

    # 防止white_count=0的报错
#     if white_count == 0:
#         white_count = 1

    # 找到白色像素的中心点位置
#     center = (white_index[0][white_count - 1] + white_index[0][0]) / 2

    # 计算出center与标准中心点的偏移量
    direction = center - 320

#不打印它了    print(direction)
    
    #蓝牙串口
    n = ser.inWaiting()
 #   breaking()
    if(n):
        v=ser.read(n)
        print(v)
        if(v=="q"):
            pwm1.ChangeDutyCycle(50)
            pwm2.ChangeDutyCycle(50 - direction)
            GPIO.output(IN1,GPIO.LOW)
            GPIO.output(IN2,GPIO.HIGH)
            GPIO.output(IN3,GPIO.LOW)
            GPIO.output(IN4,GPIO.HIGH) 
        if(v=="w"):
            pwm1.ChangeDutyCycle(50)
            pwm2.ChangeDutyCycle(50 - direction)
            GPIO.output(IN1,GPIO.HIGH)
            GPIO.output(IN2,GPIO.LOW)
            GPIO.output(IN3,GPIO.HIGH)
            GPIO.output(IN4,GPIO.LOW) 
        if(v=="e"):
            pwm1.ChangeDutyCycle(50)
            pwm2.ChangeDutyCycle(50 - direction)
            GPIO.output(IN1,GPIO.LOW)
            GPIO.output(IN2,GPIO.LOW)
            GPIO.output(IN3,GPIO.HIGH)
            GPIO.output(IN4,GPIO.LOW) 
        if(v=="r"):
            pwm1.ChangeDutyCycle(50 + direction)
            pwm2.ChangeDutyCycle(50)
            GPIO.output(IN1,GPIO.LOW)
            GPIO.output(IN2,GPIO.HIGH)
            GPIO.output(IN3,GPIO.LOW)
            GPIO.output(IN4,GPIO.LOW)
        if(v=="d"):
            pwm1.ChangeDutyCycle(0)
            pwm2.ChangeDutyCycle(0)
    sleep(0)

     # 停止
#    if abs(direction) > 250:
#         pwm1.ChangeDutyCycle(0)
#         pwm2.ChangeDutyCycle(0)
  #      pwm3.ChangeDutyCycle(0)
 #       pwm4.ChangeDutyCycle(0)

    # 右转
#     elif direction >= 0:
        # 限制在70以内
#         if direction > 70:
#             direction = 70
#         pwm1.ChangeDutyCycle(15 + direction)
#         pwm2.ChangeDutyCycle(25)
#         GPIO.output(IN1,GPIO.LOW)
#         GPIO.output(IN2,GPIO.HIGH)
#         GPIO.output(IN3,GPIO.LOW)
#         GPIO.output(IN4,GPIO.LOW)
 #       pwm3.ChangeDutyCycle(320)
#      pwm4.ChangeDutyCycle(0)

    # 左转
#     elif direction < 0:
#         if direction < -70:
#             direction = -70
#         pwm1.ChangeDutyCycle(25)
#         pwm2.ChangeDutyCycle(20 - direction)
#         GPIO.output(IN1,GPIO.LOW)
#         GPIO.output(IN2,GPIO.LOW)
#         GPIO.output(IN3,GPIO.HIGH)
#         GPIO.output(IN4,GPIO.LOW)       
  #      pwm3.ChangeDutyCycle(30 - direction)
  #      pwm4.ChangeDutyCycle(0)

#      if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
# 释放清理
# cap.release()
# cv2.destroyAllWindows()
pwm1.stop()
pwm2.stop() 
gpio.cleanup()

