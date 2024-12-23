#!/usr/bin/env python
import PCF8591 as ADC
import RPi.GPIO as GPIO
import time
import math
import LCD1602
import smbus

power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

#引脚资源配置
DO = 17
DO2 = 13
TRIG = 20
ECHO = 21
Buzzer = 12    # pin11
RoAPin = 4    # CLK Pin
RoBPin = 5    # DT Pin
BtnPin = 23    # Button Pin
BuzzerPin = Buzzer

# 设置引脚编号模式为BCM编号
GPIO.setmode(GPIO.BCM)# Numbers GPIOs by physical location

BUS = smbus.SMBus(1)

pin_R = 18 # pins is a dict

# 引脚模式设置
GPIO.setup(pin_R, GPIO.OUT)   # Set pins' mode is output
GPIO.output(pin_R, GPIO.HIGH) # Set pins to high(+3.3V) to off led
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(BuzzerPin, GPIO.OUT)
GPIO.output(BuzzerPin, GPIO.HIGH)
GPIO.setup(RoAPin, GPIO.IN)    # input mode
GPIO.setup(RoBPin, GPIO.IN)
GPIO.setup(BtnPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

globalCounter = 0

flag = 0
Last_RoB_Status = 0
Current_RoB_Status = 0



def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

 
bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
address = 0x68       # This is the address value read via the i2cdetect command

# Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)

def rotaryDeal():
    global flag
    global Last_RoB_Status
    global Current_RoB_Status
    global globalCounter
    Last_RoB_Status = GPIO.input(RoBPin)
    while(not GPIO.input(RoAPin)):
        Current_RoB_Status = GPIO.input(RoBPin)
        flag = 1
    if flag == 1:
        flag = 0
        if (Last_RoB_Status == 0) and (Current_RoB_Status == 1):
            globalCounter = globalCounter + 1
        if (Last_RoB_Status == 1) and (Current_RoB_Status == 0):
            globalCounter = globalCounter - 1

def btnISR(channel):
    global globalCounter
    globalCounter = 0


def on():
    GPIO.output(BuzzerPin, GPIO.LOW)

def off():
    GPIO.output(BuzzerPin, GPIO.HIGH)
    
def beep(x):
    on()
    time.sleep(x)
    off()
    time.sleep(x)

def distance():
    GPIO.output(TRIG, 0)
    time.sleep(0.000002)

    GPIO.output(TRIG, 1)
    time.sleep(0.00001)
    GPIO.output(TRIG, 0)

    
    while GPIO.input(ECHO) == 0:
        a = 0
    time1 = time.time()
    while GPIO.input(ECHO) == 1:
        a = 1
    time2 = time.time()

    during = time2 - time1
    return during * 340 / 2 * 100

def write_word(addr, data):
    global BLEN
    temp = data
    if BLEN == 1:
        temp |= 0x08
    else:
        temp &= 0xF7
    BUS.write_byte(addr ,temp)

def send_command(comm):
    # Send bit7-4 firstly
    buf = comm & 0xF0
    buf |= 0x04               # RS = 0, RW = 0, EN = 1
    write_word(LCD_ADDR ,buf)
    time.sleep(0.002)
    buf &= 0xFB               # Make EN = 0
    write_word(LCD_ADDR ,buf)

    # Send bit3-0 secondly
    buf = (comm & 0x0F) << 4
    buf |= 0x04               # RS = 0, RW = 0, EN = 1
    write_word(LCD_ADDR ,buf)
    time.sleep(0.002)
    buf &= 0xFB               # Make EN = 0
    write_word(LCD_ADDR ,buf)

def send_data(data):
    # Send bit7-4 firstly
    buf = data & 0xF0
    buf |= 0x05               # RS = 1, RW = 0, EN = 1
    write_word(LCD_ADDR ,buf)
    time.sleep(0.002)
    buf &= 0xFB               # Make EN = 0
    write_word(LCD_ADDR ,buf)

    # Send bit3-0 secondly
    buf = (data & 0x0F) << 4
    buf |= 0x05               # RS = 1, RW = 0, EN = 1
    write_word(LCD_ADDR ,buf)
    time.sleep(0.002)
    buf &= 0xFB               # Make EN = 0
    write_word(LCD_ADDR ,buf)

def init(addr, bl):
#   global BUS
#   BUS = smbus.SMBus(1)
    global LCD_ADDR
    global BLEN
    LCD_ADDR = addr
    BLEN = bl
    try:
        send_command(0x33) # Must initialize to 8-line mode at first
        time.sleep(0.005)
        send_command(0x32) # Then initialize to 4-line mode
        time.sleep(0.005)
        send_command(0x28) # 2 Lines & 5*7 dots
        time.sleep(0.005)
        send_command(0x0C) # Enable display without cursor
        time.sleep(0.005)
        send_command(0x01) # Clear Screen
        BUS.write_byte(LCD_ADDR, 0x08)
    except:
        return False
    else:
        return True

def clear():
    send_command(0x01) # Clear Screen

def openlight():  # Enable the backlight
    BUS.write_byte(0x27,0x08)
    BUS.close()

def write(x, y, str):
    if x < 0:
        x = 0
    if x > 15:
        x = 15
    if y <0:
        y = 0
    if y > 1:
        y = 1

    # Move cursor
    addr = 0x80 + 0x40 * y + x
    send_command(addr)

    for chr in str:
        send_data(ord(chr))

def Print(x):
    init(0x27, 1)
    if x == 1:
        write(4, 0, 'Not')
        write(7, 1, 'Raining!')
    if x == 0:
        write(4, 0, 'It is')
        write(7, 1, 'Raining!')

def setup():
    ADC.setup(0x48)
    GPIO.setup(DO, GPIO.IN)
    GPIO.setup(DO2, GPIO.IN)



def loop():
    status = 1
    global globalCounter
    tmp2 = 0 # Rotary Temperary

    # 中断事件监测
    GPIO.add_event_detect(BtnPin, GPIO.FALLING, callback=rotaryDeal)
    
    # 主循环
    while True:
        # 判断光敏传感器灰度值，当其大于210时将LED打开
        print ('Value: ', ADC.read(0))
        if ADC.read(0) > 210:
            GPIO.output(pin_R, GPIO.HIGH)
        else:
            GPIO.output(pin_R, GPIO.LOW)
            
       # 读取雨滴传感器数据，当检测到湿度信息且与当前状态不同时，在LCD上显示“It is raining”或者“Not raining”
        print (ADC.read(1))
        tmp = GPIO.input(DO2);
        if tmp != status:
            Print(tmp)
            status = tmp

        # 读取超声波传感器数据，当距离数据小于10时蜂鸣器报警
        dis = distance()
        if dis<10:
            beep(0.0001)
            
        # 旋转编码传感器获取旋转角度，并在LCD上显示总里程数
        if tmp2 != globalCounter:
            init(0x27, 1)
            write(4, 0, 'Total is')
            write(7, 1, str(globalCounter))
            tmp2 = globalCounter
            
        time.sleep(0.5)
        
        # 陀螺仪加速度传感器获得z,y,z三个方向的角加速度以及姿态信息，并在工作区打印
        gyro_xout = read_word_2c(0x43)
        gyro_yout = read_word_2c(0x45)
        gyro_zout = read_word_2c(0x47)

        print ("gyro_xout : ", gyro_xout, " scaled: ", (gyro_xout / 131))
        print ("gyro_yout : ", gyro_yout, " scaled: ", (gyro_yout / 131))
        print ("gyro_zout : ", gyro_zout, " scaled: ", (gyro_zout / 131))

        accel_xout = read_word_2c(0x3b)
        accel_yout = read_word_2c(0x3d)
        accel_zout = read_word_2c(0x3f)

        accel_xout_scaled = accel_xout / 16384.0
        accel_yout_scaled = accel_yout / 16384.0
        accel_zout_scaled = accel_zout / 16384.0

        print ("accel_xout: ", accel_xout, " scaled: ", accel_xout_scaled)
        print ("accel_yout: ", accel_yout, " scaled: ", accel_yout_scaled)
        print ("accel_zout: ", accel_zout, " scaled: ", accel_zout_scaled)

        print ("x rotation: " , get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))
        print ("y rotation: " , get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))
        
        # 若检测到角加速度突然增加，认为车辆发生晃动，蜂鸣器报警
        if abs(accel_xout_scaled)>1 or abs(accel_yout_scaled)>1 or abs(accel_zout_scaled)>1:
            beep(0.001)
        
        # 若检测到姿态角发生突然改变，认为此时发生侧翻，蜂鸣器报警
        if get_x_rotation(accel_xout_scaled,accel_yout_scaled,accel_zout_scaled)>60 or get_y_rotation(accel_xout_scaled,accel_yout_scaled,accel_zout_scaled)>60
            beep(0.001)
            
        time.sleep(0.2)

if __name__ == '__main__':
    try:
        setup()
        loop()
    except KeyboardInterrupt: 
        pass    
