#!/usr/bin/env python3
import serial
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String, Float32
from initialize_position import Initialize_Pos

#
ser = serial.Serial ("/dev/ttyUSB_DEVICE2", 115200) #Open port with baud rate 
class STM_Connect():
    def __init__(self): 
        rospy.init_node('STM_Pub', anonymous=True)
        self.left_wheel_speed_ = 0
        self.right_wheel_speed_ = 0
        self.center_wheel_speed_ = 0     
        rospy.loginfo("Publish data to STM")
        rospy.Subscriber('wheel_vtarget',String,self.Update_Speed)
    #     rospy.Subscriber('/joystick', Vector3, self.joystickCallback)

    # def joystickCallback(self, msg):
    #     r = rospy.Rate(20)
    #     #M0 = power command 
    #     #M1 = position command
    #     #M2 = velocity command
    #     self._left_wheel_power = -int(msg.z) 
    #     self._center_wheel_power = -int(msg.x)
    #     self._right_wheel_power= -int(msg.y)
    #     #power command   
    #     power_message = "M0"+"A"+str(self._left_wheel_power)+"B"+str(self._right_wheel_power)+"C"+str(self._center_wheel_power)+"\r\n"
    #     ser.write(bytes(power_message, 'utf-8'))
    #     rospy.loginfo(power_message)
    #     r.sleep()
        
    def Update_Speed(self, msg):
        r = rospy.Rate(10)
        self.wheel = msg.data.split(',')
        self.left_wheel_power = -int(float(self.wheel[0])*100)
        self.center_wheel_power = -int(float(self.wheel[1])*100)
        self.right_wheel_power = -int(float(self.wheel[2])*100)
       
        if self.left_wheel_power==0 and self.right_wheel_power==0 and self.center_wheel_power==0:
            print("no motion")
        else: #dead zone
            if 0<=abs(self.left_wheel_power)<9 and 0<=abs(self.right_wheel_power)<9 and 0<=abs(self.center_wheel_power)<9:
                if self.left_wheel_power<0:
                    self.left_wheel_power = self.left_wheel_power - 10
                if self.right_wheel_power<0:
                    self.right_wheel_power = self.right_wheel_power -10 
                if self.center_wheel_power<0:
                    self.center_wheel_power = self.center_wheel_power -10 
                if self.left_wheel_power>0:
                    self.left_wheel_power = self.left_wheel_power + 10
                if self.right_wheel_power>0:
                    self.right_wheel_power = self.right_wheel_power +10 
                if self.center_wheel_power>0:
                    self.center_wheel_power = self.center_wheel_power +10 
            if abs(self.left_wheel_power)+abs(self.right_wheel_power)+abs(self.center_wheel_power)<30:
                self.center_wheel_power = float(self.center_wheel_power) * 1.2
                self.left_wheel_power = float(self.left_wheel_power) * 1.2
                self.right_wheel_power = float(self.right_wheel_power) * 1.2           
       #power command   
        power_message = "M0"+"A"+str(self.left_wheel_power)+"B"+str(self.right_wheel_power)+"C"+str(self.center_wheel_power)+"\r\n"
        ser.write(bytes(power_message, 'utf-8'))
        rospy.loginfo(power_message)
        r.sleep()

    def sendSerial(self,string):
        packet = bytes(string+"\r\n",'ascii')
        ser.write(packet)     

    # def selflocalize(self):
    #     self.sendSerial("M0A20B20C20")
    #     rospy.Rate(0.1).sleep()
    #     self.sendSerial("M0A0B0C0")

class stop():
    def sendSerial(self,string):
        packet = bytes(string+"\r\n",'ascii')
        ser.write(packet)   

    def stop(self):
        rospy.loginfo("stop")
        self.sendSerial("M0A0B0C0")

if __name__ =='__main__':
    try:
        agent=STM_Connect() 	
        rospy.spin()
        rospy.on_shutdown(stop().stop())
    except rospy.ROSInterruptException:
        rospy.logwarn("Connection Failed")