#!/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin, asin
from turtle import position
import numpy as np
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os

m = 27*(10**-3)
l = 46*(10**-3)
Ix = 16.57171*(10**-6)
Iy = 16.57171*(10**-6)
Iz = 29.261652*(10**-6)
Ip = 12.65625 *(10**-8)
kf=1.28192*(10**-8)
km=5.964552*(10**-3)
g=9.8


####### Tuning Parameters ########
Kp = 119.205
Kd = 10

lamda1=0.52 	#for z
lamda2=2.85	#for phi
lamda3=11	#for theta
lamda4=12.8	#for psi

k1=2.52	#for z
k2=156.725	#for phi
k3=199		#for theta
k4=7		#for psi
##################################

class Quadrotor():
  def __init__(self):
     # publisher for rotor speeds 
     self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=10)
     # subscribe to Odometry topic
     self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry",Odometry, self.odom_callback)
     self.t0 = None
     self.t = None
     self.t_series = []
     self.x_series = []
     self.y_series = []
     self.z_series = []
     self.mutex_lock_on = False
     
     #Initial Rotor speeds
     self.w1=0
     self.w2=0
     self.w3=0
     self.w4=0
     
     #Allocation Matrix
     self.alloc_mat=np.array([[1/(4*kf),(2**0.5)*-1/(4*kf*l),(2**0.5)*-1/(4*kf*l),-1/(4*km*kf)],[1/(4*kf),(2**0.5)*-1/(4*kf*l),(2**0.5)*1/(4*kf*l), 1/(4*km*kf)],[1/(4*kf),(2**0.5)*1/(4*kf*l), (2**0.5)*1/(4*kf*l), -1/(4*km*kf)],[1/(4*kf),(2**0.5)*1/(4*kf*l), (2**0.5)*-1/(4*kf*l), 1/(4*km*kf)]])
     
     self.x_des     = 0 
     self.xdot_des  = 0
     self.xddot_des = 0

     self.y_des     = 0 
     self.ydot_des  = 0
     self.yddot_des = 0

     self.z_des     = 0 
     self.zdot_des  = 0
     self.zddot_des = 0
 
     
     rospy.on_shutdown(self.save_data)
     
  def traj_evaluate(self):
    # TODO: evaluating the corresponding trajectories designed in Part 1 to return the desired positions, velocities and accelerations
    
    T=np.array([1,self.t,self.t**2,self.t**3,self.t**4,self.t**5])
    Tdash=np.array([0,1,2*self.t,3*self.t**2,4*self.t**3,5*self.t**4])
    Tddash=np.array([0,0,2,6*self.t,12*self.t**2,20*self.t**3])
    
    # Desired trajectory coefficients from MATLAB
    ax=np.array([0,-0.0478,0.0116,-0.0004,0.0000,-0.0000])
    ay=np.array([0,0.0309,-0.0087,0.0006,-0.0000,0.0000])
    az=np.array([0,0.3140,-0.0273,0.0010,-0.0000,0.0000])
    
    self.x_des     = np.dot(ax,T) 
    self.xdot_des  = np.dot(ax,Tdash)
    self.xddot_des = np.dot(ax,Tddash)
    
    print(self.x_des)

    self.y_des     = np.dot(ay,T)
    self.ydot_des  = np.dot(ay,Tdash)
    self.yddot_des = np.dot(ay,Tddash)

    self.z_des     = np.dot(az,T)
    self.zdot_des  = np.dot(az,Tdash)
    self.zddot_des = np.dot(az,Tddash)
    
    
    self.x_des     = (26569384468291*self.t**4)/4611686018427387904 - (216892934435*self.t**5)/9223372036854775808 - (1020752372685139*self.t**3)/2305843009213693952 + (3350826389417567*self.t**2)/288230376151711744 - (6883808954239807*self.t)/144115188075855872 
    self.xdot_des  = (26569384468291*self.t**3)/1152921504606846976 - (1084464672175*self.t**4)/9223372036854775808 - (3062257118055417*self.t**2)/2305843009213693952 + (3350826389417567*self.t)/144115188075855872 - 6883808954239807/144115188075855872
    self.xddot_des = (79708153404873*self.t**2)/1152921504606846976 - (1084464672175*self.t**3)/2305843009213693952 - (3062257118055417*self.t)/1152921504606846976 + 3350826389417567/144115188075855872
 
    
    print(self.x_des)

    self.y_des     = (11104918243077*self.t**5)/147573952589676412928 - (212555075746405*self.t**4)/18446744073709551616 + (642545318264037*self.t**3)/1152921504606846976 - (5005482252510573*self.t**2)/576460752303423488 + (2224847054010301*self.t)/72057594037927936
    self.ydot_des  = (55524591215385*self.t**4)/147573952589676412928 - (212555075746405*self.t**3)/4611686018427387904 + (1927635954792111*self.t**2)/1152921504606846976 - (5005482252510573*self.t)/288230376151711744 + 2224847054010301/72057594037927936
    self.yddot_des = (55524591215385*self.t**3)/36893488147419103232 - (637665227239215*self.t**2)/4611686018427387904 + (1927635954792111*self.t)/576460752303423488 - 5005482252510573/288230376151711744

    self.z_des     = (810845893349*self.t**5)/9223372036854775808 - (17737253917029*self.t**4)/1152921504606846976 + (281895642609995*self.t**3)/288230376151711744 - (491931651605185*self.t**2)/18014398509481984 + (5655729290284997*self.t)/18014398509481984
 
    self.zdot_des  = (4054229466745*self.t**4)/9223372036854775808 - (17737253917029*self.t**3)/288230376151711744 + (845686927829985*self.t**2)/288230376151711744 - (491931651605185*self.t)/9007199254740992 + 5655729290284997/18014398509481984
    self.zddot_des = (4054229466745*self.t**3)/2305843009213693952 - (53211761751087*self.t**2)/288230376151711744 + (845686927829985*self.t)/144115188075855872 - 491931651605185/9007199254740992
    
    
    
  def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
     # obtain the desired values by evaluating the corresponding trajectories
     
               # obtain the desired values by evaluating the corresponding trajectories
        if self.t<=65:
            self.traj_evaluate()
            omega = self.w1-self.w2+self.w3-self.w4
            # Current Position vector
            x=xyz[0,0]
            y=xyz[1,0]
            z=xyz[2,0]
            # Current Velocity vector
            xdot=xyz_dot[0,0]
            ydot=xyz_dot[1,0]
            zdot=xyz_dot[2,0]
            # Read current Roll ,Pitch, Yaw angle from Odometry
            roll_ang = rpy[0,0]
            pitch_ang = rpy[1,0]
            yaw_ang = rpy[2,0]
            # Current Roll ,Pitch, Yaw angular velocities
            roll_vel = rpy_dot[0,0]
            pitch_vel = rpy_dot[1,0]
            yaw_vel = rpy_dot[2,0]

            print(x,y,z)

            ############## U1 Equations ################
            ############## Sliding Equation
            s1 = (zdot - self.zdot_des) + lamda1 * (z - self.z_des)
            # Signum of S1
            if s1>0:
                sgns1=+1
            elif s1<0:
                sgns1=-1
            else:
                sgns1=0
            # U1 equation
            u1 = m * (g - lamda1 * (zdot - self.zdot_des) + self.zddot_des - k1 * sgns1) * (1/(np.cos(roll_ang)*np.cos(pitch_ang)))
            ####### Convert desired Postions to Desired Roll,Pitch angles #######
            Fx = m*(-Kp*(x-self.x_des)-Kd*(xdot-self.xdot_des)+ self.xddot_des)
            Fy = m*(-Kp*(y-self.y_des)-Kd*(ydot-self.ydot_des)+ self.yddot_des)
            fxu1 = Fx / u1  
            fyu1 = Fy / u1 
            fxu1 = min(fxu1,1)
            fxu1 = max(fxu1,-1)
            fyu1 = min(fyu1,1)
            fyu1 = max(fyu1,-1)
            theta_des = asin(fxu1)
            phi_des   = asin(-fyu1)
            ############## U2 Equations #############
            ############## Sliding Equation
            s2 = roll_vel + lamda2*np.arctan2(np.sin(roll_ang-phi_des),np.cos(roll_ang-phi_des))
            # Signum of S2
            if s2>0:
                sgns2=+1
            elif s2<0:
                sgns2=-1
            else:
                sgns2=0
            # U2 equation
            u2 = Ix*((-pitch_vel*yaw_vel*((Iy-Ix)/Ix))+Ip*omega*pitch_vel/Ix-lamda2*(roll_vel)-k2*sgns2)
            ############## U3 Equations ##############
            ############## Sliding Equation
            s3 = pitch_vel + lamda3 * np.arctan2(np.sin(pitch_ang - theta_des),np.cos(pitch_ang - theta_des))
            # Signum of S3
            if s3>0:
                sgns3=+1
            elif s3<0:
                sgns3=-1
            else: 
                sgns3=0
            # U3 equation
            u3 = Iy*((-roll_vel*yaw_vel*((Iz-Ix)/Iy))+Ip*omega*roll_vel/Ix-lamda3*(pitch_vel)-k3*sgns3)
            ############## U4 Equations #############
            ############# Sliding Equation
            s4 = yaw_vel + lamda4 * np.arctan2(np.sin(yaw_ang),np.cos(yaw_ang))
            # Signum of S4
            if s4>0:
                sgns4=+1
            elif s4<0:
                sgns4=-1
            else: 
                sgns4=0
            ############### U4 equation
            u4 = Iz*(((-roll_vel*pitch_vel*((Ix-Iy)/Iz))-lamda4*(yaw_vel)-k4*sgns4))
            # U = [u1,u2,u3,u4]
            U = np.array([u1,u2,u3,u4])
            # convert U matrix to Angular velocites using Allocation matrix
            ang_vel_sq = self.alloc_mat @ U
            ang_vel_sq = np.abs(ang_vel_sq)
            ang_vel = ang_vel_sq**(1/2)
            # Limit the angular velocity to 2618
            for i in range(0,len(ang_vel)):
                ang_vel[i]=min(ang_vel[i],2618)

            motor_vel = ang_vel
            motor_speed = Actuators()
            motor_speed.angular_velocities = [motor_vel[0], motor_vel[1], motor_vel[2], motor_vel[3]]
            # Previous time sample - Angular velocities
            self.w1=motor_vel[0]
            self.w2=motor_vel[1]
            self.w3=motor_vel[2]
            self.w4=motor_vel[3]

            self.motor_speed_pub.publish(motor_speed)

        else:
            motor_speed1=Actuators()
            motor_speed1.angular_velocities=[0,0,0,0]


  
# odometry callback function (DO NOT MODIFY)
  def odom_callback(self, msg):
        if self.t0 == None:
            self.t0 = msg.header.stamp.to_sec()
        self.t = msg.header.stamp.to_sec() - self.t0
        # convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
        w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])
        v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])
        xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position.y], [msg.pose.pose.position.z]])
        q = msg.pose.pose.orientation
        T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = xyz[0:3, 0]
        R = T[0:3, 0:3]
        xyz_dot = np.dot(R, v_b)
        rpy = tf.transformations.euler_from_matrix(R, 'sxyz')
        rpy_dot = np.dot(np.asarray([[1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])],[0, np.cos(rpy[0]), -np.sin(rpy[0])], [0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]]), w_b)
        rpy = np.expand_dims(rpy, axis=1)
        # store the actual trajectory to be visualized later
        if (self.mutex_lock_on is not True):
            self.t_series.append(self.t)
            self.x_series.append(xyz[0, 0])
            self.y_series.append(xyz[1, 0])
            self.z_series.append(xyz[2, 0])


        # call the controller with the current states
        self.smc_control(xyz, xyz_dot, rpy, rpy_dot)
        
      # save the actual trajectory data
  def save_data(self):
        # TODO: update the path below with the correct path
        with open("/home/girivaasan/rbe502_project/src/control/scripts/log.pkl", "wb") as fp:
            self.mutex_lock_on = True
            pickle.dump([self.t_series,self.x_series,self.y_series,self. z_series], fp)
            
if __name__ == '__main__':
    rospy.init_node("quadrotor_control")
    rospy.loginfo("Press Ctrl + C to terminate")
    whatever = Quadrotor()
    try:
       rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
            
            
     
