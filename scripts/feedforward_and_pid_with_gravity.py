#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import numpy as np

class DesiredTrajectoryNode:
    def __init__(self):
        rospy.init_node('desired_trajectory_node', anonymous=True)
        self.torque1_pub = rospy.Publisher('/torque1', Float64, queue_size=10)
        self.torque2_pub = rospy.Publisher('/torque2', Float64, queue_size=10)
        self.torque3_pub = rospy.Publisher('/torque3', Float64, queue_size=10)
        self.pwm1_pub = rospy.Publisher('/pwm1', Float64, queue_size=10)
        self.pwm2_pub = rospy.Publisher('/pwm2', Float64, queue_size=10)
        self.pwm3_pub = rospy.Publisher('/pwm3', Float64, queue_size=10)
        self.e1_pub = rospy.Publisher('/e1', Float64, queue_size=10)
        self.e2_pub = rospy.Publisher('/e2', Float64, queue_size=10)
        self.e3_pub = rospy.Publisher('/e3', Float64, queue_size=10)
        self.edot1_pub = rospy.Publisher('/edot1', Float64, queue_size=10)
        self.edot2_pub = rospy.Publisher('/edot2', Float64, queue_size=10)
        self.edot3_pub = rospy.Publisher('/edot3', Float64, queue_size=10)
        self.q1_desired_pub = rospy.Publisher('/q1_desired', Float64, queue_size=10)
        self.q2_desired_pub = rospy.Publisher('/q2_desired', Float64, queue_size=10)
        self.q3_desired_pub = rospy.Publisher('/q3_desired', Float64, queue_size=10)
        self.qdot1_desired_pub = rospy.Publisher('/qdot1_desired', Float64, queue_size=10)
        self.qdot2_desired_pub = rospy.Publisher('/qdot2_desired', Float64, queue_size=10)
        self.qdot3_desired_pub = rospy.Publisher('/qdot3_desired', Float64, queue_size=10)
        self.qddot1_desired_pub = rospy.Publisher('/qddot1_desired', Float64, queue_size=10)
        self.qddot2_desired_pub = rospy.Publisher('/qddot2_desired', Float64, queue_size=10)
        self.qddot3_desired_pub = rospy.Publisher('/qddot3_desired', Float64, queue_size=10)

        self.q1_pub = rospy.Publisher('/robot_arm/joint1_position_controller/command', Float64, queue_size=10)
        self.q2_pub = rospy.Publisher('/robot_arm/joint2_position_controller/command', Float64, queue_size=10)
        self.q3_pub = rospy.Publisher('/robot_arm/joint3_position_controller/command', Float64, queue_size=10)

        self.q1, self.q2, self.q3 = 0.0, 0.0, 0.0
        self.qdot1, self.qdot2, self.qdot3 = 0.0, 0.0, 0.0
        self.q1_desired, self.q2_desired, self.q3_desired = 0.0, 0.0, 0.0
        self.qdot1_desired, self.qdot2_desired, self.qdot3_desired = 0.0, 0.0, 0.0
        self.qddot1_desired, self.qddot2_desired, self.qddot3_desired = 0.0, 0.0, 0.0

        self.kv = 0.5
        self.kp = 2.0
        self.ki = 0.001
        self.e1_sum, self.e2_sum, self.e3_sum = 0.0, 0.0, 0.0
        self.q1_sub = rospy.Subscriber('/q1', Float64, self.q1_callback)
        self.q2_sub = rospy.Subscriber('/q2', Float64, self.q2_callback)
        self.q3_sub = rospy.Subscriber('/q3', Float64, self.q3_callback)
        self.qdot1_sub = rospy.Subscriber('/qdot1', Float64, self.qdot1_callback)
        self.qdot2_sub = rospy.Subscriber('/qdot2', Float64, self.qdot2_callback)
        self.qdot3_sub = rospy.Subscriber('/qdot3', Float64, self.qdot3_callback)
        self.start_time = rospy.Time.now()  

    def generate_trajectory(self):
        t0 = 0  
        tf = 10
        p0 = np.array([self.q1, self.q2, self.q3])
        pf = np.array([np.pi/2, np.pi/4, -np.pi/2])
        v0 = np.array([0, 0, 0]) 
        vf = np.array([0, 0, 0]) 

        current_time = (rospy.Time.now() - self.start_time).to_sec()
        t = current_time

        if t >= tf:
            self.q_desired = pf
            self.qdot_desired = 0.0
            self.qddot_desired = 0.0
        else:
            M = np.array([[t0**3, t0**2, t0, 1],
                          [tf**3, tf**2, tf, 1],
                          [3*t0**2, 2*t0, 1, 0],
                          [3*tf**2, 2*tf, 1, 0]])
            b = np.array([p0, pf, v0, vf])
            coefficients = np.linalg.solve(M, b)
            a, b, c, d = coefficients

            self.q_desired = a * t**3 + b * t**2 + c * t + d
            self.qdot_desired = 3 * a * t**2 + 2 * b * t + c
            self.qddot_desired = 6 * a * t + 2 * b

            self.q1_desired = self.q_desired[0]
            self.q2_desired = self.q_desired[1]
            self.q3_desired = self.q_desired[2]

            self.qdot1_desired = self.qdot_desired[0]
            self.qdot2_desired = self.qdot_desired[1]
            self.qdot3_desired = self.qdot_desired[2]

            self.qddot1_desired = self.qddot_desired[0]
            self.qddot2_desired = self.qddot_desired[1]
            self.qddot3_desired = self.qddot_desired[2]

        return tf    

    def q1_callback(self, msg):
        self.q1 = msg.data 

    def q2_callback(self, msg):
        self.q2 = msg.data 

    def q3_callback(self, msg):
        self.q3 = msg.data 
    
    def qdot1_callback(self, msg):
        self.qdot1 = msg.data 

    def qdot2_callback(self, msg):
        self.qdot2 = msg.data 

    def qdot3_callback(self, msg):
        self.qdot3 = msg.data  

    def generate_torque(self):
        self.generate_trajectory()
        e1 = self.q1_desired - self.q1
        e2 = self.q2_desired - self.q2
        e3 = self.q3_desired - self.q3
        edot1 = self.qdot1_desired - self.qdot1
        edot2 = self.qdot2_desired - self.qdot2
        edot3 = self.qdot3_desired - self.qdot3
        m2 = 0.12084
        m3 = 0.02485
        g_force = 9.81
        a2 = 0.115
        a3 = 0.02276
        x = 79.4*np.pi/180
        """
        G_matrix = np.array([[0],
                             [g_force*(m3*(a3*np.cos(self.q2 + self.q3) + a2*np.cos(self.q2 + x)) + a2*m2*np.cos(self.q2 + x))],
                             [a3*g_force*m3*np.cos(self.q2 + self.q3)]
                            ])"""
        g1 = 0
        g2 = g_force*(m3*(a3*np.cos(self.q2 + self.q3) + a2*np.cos(self.q2 + x)) + a2*m2*np.cos(self.q2 + x))
        g3 = a3*g_force*m3*np.cos(self.q2 + self.q3)

        tf = self.generate_trajectory()
        current_time = (rospy.Time.now() - self.start_time).to_sec()

        if current_time <= tf:
            self.e1_sum += e1
            self.e2_sum += e2
            self.e3_sum += e3
        
        torque1 = (self.qddot1_desired + self.kp * e1  + self.ki * self.e1_sum + self.kv * edot1 + g1)
        torque2 = (self.qddot2_desired + self.kp * e2  + self.ki * self.e2_sum + self.kv * edot2 + g2)
        torque3 = (self.qddot3_desired + self.kp * e3  + self.ki * self.e3_sum + self.kv * edot3 + g3)
        pwm1 = (self.qddot1_desired + self.kp * e1  + self.ki * self.e1_sum + self.kv * edot1 + g1) * (885 / 4.1)
        pwm2 = (self.qddot2_desired + self.kp * e2  + self.ki * self.e2_sum + self.kv * edot2 + g2) * (885 / 4.1)
        pwm3 = (self.qddot3_desired + self.kp * e3  + self.ki * self.e3_sum + self.kv * edot3 + g3) * (885 / 4.1)

        return torque1, torque2, torque3, e1, e2, e3, edot1, edot2, edot3, pwm1, pwm2, pwm3

    def publish_torque(self):
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():

            torque1, torque2, torque3, e1, e2, e3, edot1, edot2, edot3, pwm1, pwm2, pwm3 = self.generate_torque()
            self.torque1_pub.publish(torque1)
            self.torque2_pub.publish(torque2)
            self.torque3_pub.publish(torque3)
            self.pwm1_pub.publish(pwm1)
            self.pwm2_pub.publish(pwm2)
            self.pwm3_pub.publish(pwm3)
            self.e1_pub.publish(e1)
            self.e2_pub.publish(e2)
            self.e3_pub.publish(e3)
            self.edot1_pub.publish(edot1)
            self.edot2_pub.publish(edot2)
            self.edot3_pub.publish(edot3)
            self.q1_desired_pub.publish(self.q1_desired)
            self.q2_desired_pub.publish(self.q2_desired)
            self.q3_desired_pub.publish(self.q3_desired)
            self.qdot1_desired_pub.publish(self.qdot1_desired)
            self.qdot2_desired_pub.publish(self.qdot2_desired)
            self.qdot3_desired_pub.publish(self.qdot3_desired)
            self.qddot1_desired_pub.publish(self.qddot1_desired)
            self.qddot2_desired_pub.publish(self.qddot2_desired)
            self.qddot3_desired_pub.publish(self.qddot3_desired)

            self.q1_pub.publish(self.q1)
            self.q2_pub.publish(self.q2)
            self.q3_pub.publish(self.q3)
            rate.sleep()

if __name__ == '__main__':
    try:
        desired_trajectory_node = DesiredTrajectoryNode()
        desired_trajectory_node.publish_torque()
    except rospy.ROSInterruptException:
        pass
