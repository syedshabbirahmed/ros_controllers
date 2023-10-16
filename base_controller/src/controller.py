#! /usr/bin/env python3

import rospy, tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import numpy as np
from base_controller.srv import MoveRobot


class BaseController:

    def __init__(self):
        self.x             = rospy.get_param('/base_controller/target/x')
        self.y             = rospy.get_param('/base_controller/target/y')
        self.phi           = rospy.get_param('/base_controller/target/phi')
        self.world         = rospy.get_param('/base_controller/world')
        self.robot         = rospy.get_param('/base_controller/model_name')
        self.spd_lm        = rospy.get_param('/base_controller/limit/speed')
        self.omg_lm        = rospy.get_param('/base_controller/limit/omega')
        lsr_topic_name     = rospy.get_param('/base_controller/lsr_topic_name')
        fbk_topic_name     = rospy.get_param('/base_controller/fbk_topic_name')
        cmd_topic_name     = rospy.get_param('/base_controller/cmd_topic_name')
        self.m             = rospy.get_param('/base_controller/min_distance_to_obstacles')
        self.eps           = rospy.get_param('/base_controller/epsilon')
        self.target_1      = np.array([rospy.get_param('/base_controller/target_1/x'), 
                                       rospy.get_param('/base_controller/target_1/y'), 0])
        self.target_2      = np.array([rospy.get_param('/base_controller/target_2/x'), 
                                       rospy.get_param('/base_controller/target_2/y'), 0])


        self.regions       = {}
        self.linear_x      = 0
        self.angular_z     = 0
        self.goal          = np.array([self.x, self.y, self.phi])
        self.target_3      = np.array([self.x, self.y, self.phi])
        self.angle_range   = np.linspace(np.pi/2, -np.pi/2, 480)

        self.current_target = 1
        self.final_distance = 100

        self.msg           = Twist()
        self.pose_feedback = ModelStates()

        self._received_first_lmsg = False
        self._received_first_pmsg = False
        self._received_first_rmsg = False

        self.laser_sub = rospy.Subscriber(lsr_topic_name, LaserScan, self.laser_callback)
        self.pose_sub  = rospy.Subscriber(fbk_topic_name, ModelStates, self.pose_callback)
        self.twist_pub = rospy.Publisher(cmd_topic_name,  Twist, queue_size=1)
        self.service   = rospy.Service('base_controller/start', MoveRobot, self.start_callback)

    def start_callback(self, res):
        self._received_first_rmsg = True
        res = True
        return res

    def laser_callback(self, lmsg):
        self._received_first_lmsg = True
        self.range = np.array(lmsg.ranges[119:599])
        self.regions = {
            'right':  min(min(lmsg.ranges[0:143]), 10),
            'fright': min(min(lmsg.ranges[144:287]), 10),
            'front':  min(min(lmsg.ranges[288:431]), 10),
            'fleft':  min(min(lmsg.ranges[432:575]), 10),
            'left':   min(min(lmsg.ranges[576:719]), 10),
        }
        # 0:143, 144:287, 288:431, 432:575, 576:719
    def pose_callback(self, msg):
        self._received_first_pmsg = True
        self.pose_feedback = msg

    
    def take_action(self):
        
        if self._received_first_lmsg and self._received_first_pmsg:
            linear_x = 0
            angular_z = 0

            if self.regions['front'] > self.m and self.regions['fleft'] > self.m and self.regions['fright'] > self.m:
                linear_x = self.linear_x
                angular_z = self.angular_z
            elif self.regions['front'] < self.m and self.regions['fleft'] > self.m and self.regions['fright'] > self.m:
                linear_x = 0
                angular_z = -0.3
            elif self.regions['front'] > self.m and self.regions['fleft'] > self.m and self.regions['fright'] < self.m:
                linear_x = 0
                angular_z = -0.3
            elif self.regions['front'] > self.m and self.regions['fleft'] < self.m and self.regions['fright'] > self.m:
                linear_x = 0
                angular_z = 0.3
            elif self.regions['front'] < self.m and self.regions['fleft'] > self.m and self.regions['fright'] < self.m:
                linear_x = 0
                angular_z = -0.3
            elif self.regions['front'] < self.m and self.regions['fleft'] < self.m and self.regions['fright'] > self.m:
                linear_x = 0
                angular_z = 0.3
            elif self.regions['front'] < self.m and self.regions['fleft'] < self.m and self.regions['fright'] < self.m:
                linear_x = 0
                angular_z = -0.3
            elif self.regions['front'] > self.m and self.regions['fleft'] < self.m and self.regions['fright'] < self.m:
                linear_x = 0
                angular_z = -0.3
            else:
                pass
            if self.final_distance < 0.5:
                linear_x = self.linear_x
                angular_z = self.angular_z

            self.msg.linear.x  = linear_x
            self.msg.angular.z = angular_z
            self.twist_pub.publish(self.msg)

    def potential_field_control(self):
        
        k1 = 1
        k2 = 1
        k3 = 0.5
        k4 = 1.0
        k5 = 0.1
        vel_repul = np.array([0,0])
        if self._received_first_pmsg:
            index = self.pose_feedback.name.index(self.robot)
            pose_ = self.pose_feedback.pose[index].position
            quat_ = self.pose_feedback.pose[index].orientation
            angles = tf.transformations.euler_from_quaternion((quat_.x,quat_.y,
                                                               quat_.z,quat_.w))
            
            theta = angles[2]
            current_pose = [pose_.x, pose_.y, theta]  # x, y, theta 

            # repulsive velocity
            # obstacles
            obst_index   = np.where(self.range < self.m)
            obst_angles  = self.angle_range[obst_index]
            dist_to_obst = self.range[obst_index]
            obst_dir = np.array([dist_to_obst*-np.sin(obst_angles), dist_to_obst*np.cos(obst_angles)])
            try:
                vel_repul = ((1/dist_to_obst - 1/self.m) * (1/dist_to_obst**3) * obst_dir)
                vel_repul = k3 * np.sum(vel_repul, axis=1)
            except:
                vel_repul = np.array([0,0])
            # check if the velocity is nan
            if np.isnan(vel_repul.any()):
                vel_repul = np.array([0,0])

            # Additional potential field for the robot to avoid obstacles
            distance_1 = np.sqrt((self.target_1[0] - current_pose[0])**2 + (self.target_1[1] - current_pose[1])**2)
            distance_2 = np.sqrt((self.target_2[0] - current_pose[0])**2 + (self.target_2[1] - current_pose[1])**2)
            distance_3 = np.sqrt((self.target_3[0] - current_pose[0])**2 + (self.target_3[1] - current_pose[1])**2)

            if self.current_target == 1:
                if distance_1 > 0.6:
                    distance = distance_1
                    self.goal = self.target_1
                else:
                    distance = distance_2
                    self.current_target = 2
            if self.current_target == 2:
                if distance_2 > 0.6:
                    distance = distance_2
                    self.goal = self.target_2
                else:
                    distance = distance_3
                    self.current_target = 3
            if self.current_target == 3:
                self.final_distance = distance_3
                distance = distance_3
                self.goal = self.target_3
            # Implementation of potential field control    
            vel_atrr = k1 * (self.goal[:-1] - current_pose[:-1])

            # Limit the linear velocity
            if np.linalg.norm(vel_atrr,2) > self.spd_lm:
                vel_atrr = vel_atrr / np.linalg.norm(vel_atrr) * self.spd_lm
            velocity = k4 * vel_atrr + k5 * vel_repul

            
            if distance > self.eps:

                omega = k2 * (np.arctan2(velocity[1],velocity[0]) - theta)

                # Limit the angular velocity
                if np.abs(omega) > self.omg_lm:
                    omega = self.omg_lm * np.sign(omega)

                self.linear_x  = np.sqrt(velocity[0]**2 + velocity[1]**2)
                self.angular_z = omega
            else:
                omega  = k2 * (self.goal[2] - theta)
                
                # Limit the angular velocity
                if np.abs(omega) > self.omg_lm:
                    omega = self.omg_lm * np.sign(omega)

                self.linear_x  = 0
                self.angular_z = omega