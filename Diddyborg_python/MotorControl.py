import RPi.GPIO as GPIO
from time import sleep
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
import time
import datetime
import numpy as np
import ThunderBorg3 as ThunderBorg
# import keyboard

TB = ThunderBorg.ThunderBorg()
TB.Init()


GPIO.setup(11, GPIO.IN)    #SO1
GPIO.setup(9, GPIO.IN)     #SO2
GPIO.setup(10, GPIO.IN)    #S03
GPIO.setup(22, GPIO.IN)    #SO4
GPIO.setup(27, GPIO.IN)    #SO5
sleep(1)


speed=0.3
dt = str(datetime.datetime.now())
#on = black, off = white
timestamped_motor_command = np.zeros(3)
input_1 = 0
input_2 = 0
# try:
while True:
  s1=GPIO.input(11)
  s2=GPIO.input(9)
  s3=GPIO.input(10)
  s4=GPIO.input(22)
  s5=GPIO.input(27)
  
#10000
  if (s1==1)& (s2==0)&(s3==0)&(s4==0)&(s5==0):
    #print ("left1")
    input_2 = 1*speed
    input_1 = 0*speed
    # TB.SetMotor2(input_1)
    # TB.SetMotor1(input_2)
    #sleep(0.5)
#11000
  elif (s1==1)&(s2==1)&(s3==0)&(s4==0)&(s5==0):
    #print ("left2")
    input_2 = 1*speed
    input_1 = 0.125*speed
    # TB.SetMotor2(1*speed)
    # TB.SetMotor1(0.125*speed)
    #sleep(0.5)
#11100
  elif (s1==1)&(s2==1)&(s3==1)&(s4==0)&(s5==0):
    #print ("left3")
    input_2 = 1*speed
    input_1 = 0.375*speed
    # TB.SetMotor2(1*speed)
    # TB.SetMotor1(0.375*speed)
    #sleep(0.5)
#01000    
  elif (s1==0)&(s2==1)&(s3==0)&(s4==0)&(s5==0):
    #print ("left4")
    input_2 = 1*speed
    input_1 = 0.45*speed
    # TB.SetMotor2(1*speed)
    # TB.SetMotor1(0.45*speed)
    #sleep(0.5)
#01100
  elif (s1==0)&(s2==1)&(s3==1)&(s4==0)&(s5==0):
    #print ("left5")
    input_2 = 1*speed
    input_1 = 0.6*speed
    # TB.SetMotor2(1*speed)
    # TB.SetMotor1(0.6*speed)
    #sleep(0.5)
#00001
  elif (s1==0)&(s2==0)&(s3==0)&(s4==0)&(s5==1):
    #print ("right1")
    input_2 = 0*speed
    input_1 = 1*speed
    # TB.SetMotor2(0*speed)
    # TB.SetMotor1(1*speed)
    #sleep(0.5)
#00011
  elif (s1==0)&(s2==0)&(s3==0)&(s4==1)&(s5==1):
    #print ("right2")
    input_2 = 0.125*speed
    input_1 = 1*speed
    # TB.SetMotor2(0.125*speed)
    # TB.SetMotor1(1*speed)
    #sleep(0.5)
#00111
  elif (s1==0)&(s2==0)&(s3==1)&(s4==1)&(s5==1):
    #print ("right3")
    input_2 = 0.375*speed
    input_1 = 1*speed
    # TB.SetMotor2(0.375*speed)
    # TB.SetMotor1(1*speed)
    #sleep(0.5)
#00010
  elif (s1==0)&(s2==0)&(s3==0)&(s4==1)&(s5==0):
    #print ("right4")
    input_2 = 0.48*speed
    input_1 = 1*speed
    # TB.SetMotor2(0.48*speed)
    # TB.SetMotor1(1*speed)
    #sleep(0.5)
#00110
  elif (s1==0)&(s2==0)&(s3==1)&(s4==1)&(s5==0):
    #print ("right5")
    input_2 = 0.6*speed
    input_1 = 1*speed
    # TB.SetMotor2(0.6*speed)
    # TB.SetMotor1(1*speed)
    #sleep(0.5)
#00100
  elif (s1==0)&(s2==0)&(s3==1)&(s4==0)&(s5==0):
    #print ("straight1")
    input_2 = 1*speed
    input_1 = 1*speed
    # TB.SetMotor2(1*sp eed)
    # TB.SetMotor1(1*speed)
    #sleep(0.5)
#01110
  elif (s1==0)&(s2==1)&(s3==1)&(s4==1)&(s5==0):
    #print ("straight2")
    input_2 = 1*speed
    input_1 = 1*speed
    # TB.SetMotor1(1*speed)
    # TB.SetMotor2(1*speed)
    #sleep(0.5)

#11111
  elif (s1==1)&(s2==1)&(s3==1)&(s4==1)&(s5==1):
    #print ("STOP")
    input_2 = 0*speed
    input_1 = 0*speed
    # TB.SetMotor1(0)
    # TB.SetMotor2(0)
    #sleep(0.5)
  
  TB.SetMotor2(input_2)
  TB.SetMotor1(input_1)
  sleep(0.5)
  timestamp= time.time()#
  # input_1 =  TB.GetMotor1()
  # input_2 = TB.GetMotor2()
  # if input_1 is None:
  #   input_1 = -99999.
  # if input_2 is None:
  #   input_2 = -99999.
  timestamped_motor_command = np.array([timestamp,input_1,input_2])
  #print(timestamped_motor_command)
  with open("Motor_Input{0}.csv".format(dt), "ab") as f:
        # np.savetxt(f, np.expand_dims(timestamped_motor_command, axis=0),fmt='%4.8f' , delimiter=",")
        np.savetxt(f, timestamped_motor_command,fmt='%4.8f' , delimiter=",")

  # if keyboard.is_pressed('q'):  # if key 'q' is pressed 
  #       print('Q is pressed. Now exit!')
  #       break  # finishing the loop
  
# except:
#   TB.SetMotor1(0)
#   TB.SetMotor2(0)
      
  
 