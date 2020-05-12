#!/usr/bin/env python
import rospy
# importing "array" for array creations 
import array as arr 
from random import randint

from geometry_msgs.msg import Twist


class MoveRosBots(object):

    def __init__(self):
    
        self.cmd_vel_pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)
        self.last_cmdvel_command = Twist()
        self._cmdvel_pub_rate = rospy.Rate(10)
        self.shutdown_detected = False

                                    
    def clean_class(self):
        # Stop Robot
        twist_object = Twist()
        twist_object.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_object)
        print("You have arrived at your destination")
        self.shutdown_detected = True

    def stoplight(self):  
        #this is where the color detection message goes
        '''i = 0
        # array with int type 
        a = arr.array('i', [1, 2, 3]) 
        color = randint(0,2)
        print(color)'''
        if(a[color] == 1):
            return self.green_handler()
        elif(a[color] == 2):
            return self.yellow_handler()
        elif(a[color] == 3):
            return self.red_handler()
    
    def green_handler(self):
        return arr.array('f', [1.0, 0.0, 2.0, 0.0])
    def yellow_handler(self):
        return arr.array('f', [0.5, 0.0, 4.0, 0.0])
    def red_handler(self):
        return arr.array('f', [0.5, 0.0, 4.0, 1.0])

def main():
    rospy.init_node('move_robot_node', anonymous=True)
    
    
    moverosbots_object = MoveRosBots()
    twist_object = Twist()
    # store the linear vel, angular vel, time, stop
    vel = arr.array('f', [0.0, 0.0, 0.0, 0])
    twist_object.linear.x = vel[0]
    twist_object.angular.z = vel[1]
    moverosbots_object.cmd_vel_pub.publish(twist_object)
    
    done = 0
    rate = rospy.Rate(5)
    
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        moverosbots_object.clean_class()
        rospy.loginfo("See you next time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:

        twist_object.linear.x = 0
        twist_object.angular.z = 0
        moverosbots_object.cmd_vel_pub.publish(twist_object)
        rate.sleep()

        #Trajectory for First Segment
        if(done == 1):
            vel[0] = 1
            vel[1] = 0
            vel[2] = 6.2
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()-t0
            moverosbots_object.cmd_vel_pub.publish(twist_object)
            while(t1 < vel[2]):
                twist_object.linear.x = vel[0]  
                twist_object.angular.z = vel[1]
                moverosbots_object.cmd_vel_pub.publish(twist_object)
                t1 = rospy.Time.now().to_sec()-t0
                rate.sleep()

            print("At the light, make a sharp left.")
            #Stoplight Handling   
            vel = moverosbots_object.green_handler()
            #print(vel)
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()-t0
            while(t1 < vel[2]):
                twist_object.linear.x = vel[0]  
                twist_object.angular.z = vel[1]
                moverosbots_object.cmd_vel_pub.publish(twist_object)
                t1 = rospy.Time.now().to_sec()-t0
                rate.sleep()
            if(vel[3] == 1):
                while(t1 < (3+vel[2])):
                    twist_object.linear.x = 0
                    twist_object.angular.z = 0
                    moverosbots_object.cmd_vel_pub.publish(twist_object)
                    t1 = rospy.Time.now().to_sec()-t0
                    rate.sleep()
        
        #Trajectory for Second Segment
        if (done == 2):
            vel[0] = 1
            vel[1] = 0.3
            vel[2] = 9.5
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()-t0
            while(t1 < vel[2]):
                twist_object.linear.x = vel[0]  
                twist_object.angular.z = vel[1]
                moverosbots_object.cmd_vel_pub.publish(twist_object)
                t1 = rospy.Time.now().to_sec()-t0
                rate.sleep()     
            vel[0] = 1
            vel[1] = 0
            vel[2] = 4.0
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()-t0
            moverosbots_object.cmd_vel_pub.publish(twist_object)
            while(t1 < vel[2]):
                twist_object.linear.x = vel[0]  
                twist_object.angular.z = vel[1]
                moverosbots_object.cmd_vel_pub.publish(twist_object)
                t1 = rospy.Time.now().to_sec()-t0
                rate.sleep()
            
            print("At the light, turn right, then left.")
            #Stoplight Handling  
            vel = moverosbots_object.yellow_handler()
            #print(vel)
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()-t0
            while(t1 < vel[2]):
                twist_object.linear.x = vel[0]  
                twist_object.angular.z = vel[1]
                moverosbots_object.cmd_vel_pub.publish(twist_object)
                t1 = rospy.Time.now().to_sec()-t0
                rate.sleep()
            if(vel[3] == 1):
                while(t1 < (3+vel[2])):
                    twist_object.linear.x = 0
                    twist_object.angular.z = 0
                    moverosbots_object.cmd_vel_pub.publish(twist_object)
                    t1 = rospy.Time.now().to_sec()-t0
                    rate.sleep()

        #Trajectory for Third Segment
        if (done == 3):
            vel[0] = 1
            vel[1] = -0.3
            vel[2] = 9.8/2
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()-t0
            while(t1 < vel[2]):
                twist_object.linear.x = vel[0]  
                twist_object.angular.z = vel[1]
                moverosbots_object.cmd_vel_pub.publish(twist_object)
                t1 = rospy.Time.now().to_sec()-t0
                rate.sleep()
            vel[0] = 1
            vel[1] = 0.3
            vel[2] = 9.35/2
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()-t0
            while(t1 < vel[2]):
                twist_object.linear.x = vel[0]  
                twist_object.angular.z = vel[1]
                moverosbots_object.cmd_vel_pub.publish(twist_object)
                t1 = rospy.Time.now().to_sec()-t0
                rate.sleep()    
            vel[0] = 1
            vel[1] = 0
            vel[2] = 5.2
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()-t0
            moverosbots_object.cmd_vel_pub.publish(twist_object)
            while(t1 < vel[2]):
                twist_object.linear.x = vel[0]  
                twist_object.angular.z = vel[1]
                moverosbots_object.cmd_vel_pub.publish(twist_object)
                t1 = rospy.Time.now().to_sec()-t0
                rate.sleep()
            
            print("At the light, continue straight")
            #Stoplight Handling
            vel = moverosbots_object.red_handler()
            #print(vel)
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()-t0
            while(t1 < vel[2]):
                twist_object.linear.x = vel[0]  
                twist_object.angular.z = vel[1]
                moverosbots_object.cmd_vel_pub.publish(twist_object)
                t1 = rospy.Time.now().to_sec()-t0
                rate.sleep()
            if(vel[3] == 1):
                while(t1 < (3+vel[2])):
                    twist_object.linear.x = 0
                    twist_object.angular.z = 0
                    moverosbots_object.cmd_vel_pub.publish(twist_object)
                    t1 = rospy.Time.now().to_sec()-t0
                    rate.sleep()
        
        #Trajectory for Fourth Segment
        if (done == 4):
            #if we were turning left instead of going straight
            '''vel[0] = 1
            vel[1] = 0.33
            vel[2] = 8.65/2
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()-t0
            while(t1 < vel[2]):
                twist_object.linear.x = vel[0]  
                twist_object.angular.z = vel[1]
                moverosbots_object.cmd_vel_pub.publish(twist_object)
                t1 = rospy.Time.now().to_sec()-t0
                rate.sleep()  '''  
            vel[0] = 1
            vel[1] = 0
            vel[2] = 10
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()-t0
            moverosbots_object.cmd_vel_pub.publish(twist_object)
            while(t1 < vel[2]):
                twist_object.linear.x = vel[0]  
                twist_object.angular.z = vel[1]
                moverosbots_object.cmd_vel_pub.publish(twist_object)
                t1 = rospy.Time.now().to_sec()-t0
                rate.sleep()
            
            #Stoplight Handling
            vel = moverosbots_object.red_handler()
            #print(vel)
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()-t0
            while(t1 < vel[2]):
                twist_object.linear.x = vel[0]  
                twist_object.angular.z = vel[1]
                moverosbots_object.cmd_vel_pub.publish(twist_object)
                t1 = rospy.Time.now().to_sec()-t0
                rate.sleep()
            if(vel[3] == 1):
                while(t1 < (3+vel[2])):
                    twist_object.linear.x = 0
                    twist_object.angular.z = 0
                    moverosbots_object.cmd_vel_pub.publish(twist_object)
                    t1 = rospy.Time.now().to_sec()-t0
                    rate.sleep()
        if (done == 5):
            ctrl_c=True
        '''
        #Trajectory for Fifth Segment
        if (done == 5):
            vel[0] = 1
            vel[1] = -0.33
            vel[2] = 8.65/2
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()-t0
            while(t1 < vel[2]):
                twist_object.linear.x = vel[0]  
                twist_object.angular.z = vel[1]
                moverosbots_object.cmd_vel_pub.publish(twist_object)
                t1 = rospy.Time.now().to_sec()-t0
                rate.sleep()    
            vel[0] = 1
            vel[1] = 0
            vel[2] = 7.2
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()-t0
            moverosbots_object.cmd_vel_pub.publish(twist_object)
            while(t1 < vel[2]):
                twist_object.linear.x = vel[0]  
                twist_object.angular.z = vel[1]
                moverosbots_object.cmd_vel_pub.publish(twist_object)
                t1 = rospy.Time.now().to_sec()-t0
                rate.sleep()
                vel[0] = 1
            vel[1] = -0.33
            vel[2] = 8.65/2
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()-t0
            while(t1 < vel[2]):
                twist_object.linear.x = vel[0]  
                twist_object.angular.z = vel[1]
                moverosbots_object.cmd_vel_pub.publish(twist_object)
                t1 = rospy.Time.now().to_sec()-t0
                rate.sleep()    
            vel[0] = 1
            vel[1] = 0
            vel[2] = 6.2
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()-t0
            moverosbots_object.cmd_vel_pub.publish(twist_object)
            while(t1 < vel[2]):
                twist_object.linear.x = vel[0]  
                twist_object.angular.z = vel[1]
                moverosbots_object.cmd_vel_pub.publish(twist_object)
                t1 = rospy.Time.now().to_sec()-t0
                rate.sleep()
                vel[0] = 1
            vel[1] = -0.33
            vel[2] = 8.65/2
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()-t0
            while(t1 < vel[2]):
                twist_object.linear.x = vel[0]  
                twist_object.angular.z = vel[1]
                moverosbots_object.cmd_vel_pub.publish(twist_object)
                t1 = rospy.Time.now().to_sec()-t0
                rate.sleep()    
            vel[0] = 1
            vel[1] = 0
            vel[2] = 6.2
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()-t0
            moverosbots_object.cmd_vel_pub.publish(twist_object)
            while(t1 < vel[2]):
                twist_object.linear.x = vel[0]  
                twist_object.angular.z = vel[1]
                moverosbots_object.cmd_vel_pub.publish(twist_object)
                t1 = rospy.Time.now().to_sec()-t0
                rate.sleep()
            
            #Stoplight Handling
            vel = moverosbots_object.stoplight()
            print(vel)
            t0 = rospy.Time.now().to_sec()
            t1 = rospy.Time.now().to_sec()-t0
            while(t1 < vel[2]):
                twist_object.linear.x = vel[0]  
                twist_object.angular.z = vel[1]
                moverosbots_object.cmd_vel_pub.publish(twist_object)
                t1 = rospy.Time.now().to_sec()-t0
                rate.sleep()
            if(vel[3] == 1):
                while(t1 < (3+vel[2])):
                    twist_object.linear.x = 0
                    twist_object.angular.z = 0
                    moverosbots_object.cmd_vel_pub.publish(twist_object)
                    t1 = rospy.Time.now().to_sec()-t0
                    rate.sleep()'''

        done+= 1
        
    
if __name__ == '__main__':
    main()