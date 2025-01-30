# optitrack_subscriber2.py for second drone 
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt16
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

import matplotlib.pyplot as plt
import time

import numpy as np
from scipy.spatial.transform import Rotation as R


class OptiTrackSubscriber2(Node):
    def __init__(self):
        super().__init__('opti_track_subscriber2')

        # Customize QoS settings to match the publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # This needs to be BEST_EFFORT to read messages
            history=HistoryPolicy.KEEP_LAST,         # Keep the last message
            depth=10                                 # Queue up to 10 messages
        )

        self.subscription = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/cf2/pose',
            self.listener_callback,
            qos_profile
        
        )
        #message to send flight commands to crazyflie program
        self.pub_commands = self.create_publisher(Float32MultiArray, '/cf2/commands', 10)

        #drone to drone communication for waypoint synchronization
        self.pub_threshold_met = self.create_publisher(Bool, '/threshold_met_cf2', 10)
        self.sub_threshold_met = self.create_subscription(Bool, '/threshold_met_cf1', self.cf1_threshold_met_callback, 10)  
        self.threshold_met = False
        self.cf1_threshold_met = False

        #INITIAL SET UP 
        self.position = [0.0, 0.0, 0.0] #current position of drone, automatically updated
        #self.target_positions = [[1.5, 1.0, 1.5], [0.5,1.0,0.5],[1.0,0.5,1.0]] #set multiple the points 
        self.target_positions = [[2.0, 0.5, 1.0],[2.0, 0.5, 2.0]] #set single position (x,y,z)
        # self.target_positions = [[2.0, 1.0, 1.0]] #set single position (x,y,z)

        
        #THIS IS FUTURE CODE FOR MULTIPLE DRONES POTENTIALLY 
        #self.multiple_drones = {'gary': [[1.5, 1.0, 1.5], [0.5,1.0,0.5]], 'patrick': [[], []]}
        
        self.current_target_index = 0 
        self.target_position = self.target_positions[self.current_target_index]
        self.threshold = 0.15  # [m] Threshold for reaching the target
        # Controls variables
        self.t = 0.01 #average time between signals in seconds

        # Values for rotational (yaw) PID
        self.orientation_quat = [0.0, 0.0, 0.0, 0.0] #current orientation in quaternions
        self.current_orientation = 0.0
        self.target_orientation_quat = [0.0, 0.7, 0.0, 0.7]
        #temp fix: need to rotate drone to face right for rigid body then reorient
        self.target_orientation = 90 #position the drone in desired orientation and this value should be the yaw from optitrack
        self.k_p_rot = 0.25
        self.k_p_rot_sign = 1
        self.max_yawrate = 15
        self.min_yawrate = -15

        # values for vertical Y (thrust) PID
        self.hover = 44000 #originally 46500     
        self.max_thrust = 55000 #origionall 50000
        self.min_thrust = 42000
        self.k_p_y = 17000 #previously 15000 on jan 22
        self.k_i_y = 1500 #extra amount of thrust wanted (originally 2000)
        self.k_d_y = 10000
        self.threshold_met = False

        self.cur_y_error = 0.0
        self.prev_y_error = 0.0
        self.int_y_error = 0.0
        self.int_y_max = 5000 # maximum added thrust from integral component

        # values for horizontal X (pitch) PID
        self.k_p_x = 2
        self.k_i_x = 0.6
        self.k_d_x = 4.0
        self.max_pitch = 3.0
        self.min_pitch = -3.0

        self.cur_x_error = 0.0
        self.prev_x_error = 0.0
        self.int_x_error = 0.0
        self.int_x_max = 3.0 # maximum added pitch from integral component

        # values for horizontal Z (roll) PID
        self.k_p_z = 2
        self.k_i_z = 0.6
        self.k_d_z = 4.0 #previously 3.5
        self.max_roll = 3.0
        self.min_roll = -3.0

        self.cur_z_error = 0.0
        self.prev_z_error = 0.0
        self.int_z_error = 0.0
        self.int_z_max = 3.0

        self.startTimer = False
        self.startTime = time.time()

        

    def listener_callback(self, msg):
        # need this conditional to avoid QoS error
        if msg.header.frame_id == "world":
            # store the current x,y,x position of the drone (in meters)
            self.position[0] = msg.pose.position.x
            self.position[1] = msg.pose.position.y
            self.position[2] = msg.pose.position.z

            # store the current x, y, z, w orientation of the drone (in Quaternions)
            self.orientation_quat[0] = msg.pose.orientation.x
            self.orientation_quat[1] = msg.pose.orientation.y
            self.orientation_quat[2] = msg.pose.orientation.z
            self.orientation_quat[3] = msg.pose.orientation.w

            # calls rotational PID function (yawrate)
            yawrate = self.calculate_yawrate()

            # calls X axis PID function (pitch)
            pitch = self.calculate_pitch()

            # calls Y axis PID function (thrust)
            thrust = self.calculate_thrust()

            # Calls Z axis PID function (roll)
            roll = self.calculate_roll()

            # create message of type Float Array (all values need to be floats)
            msg = Float32MultiArray()
            msg.data = [float(roll), float(pitch), float(yawrate), float(thrust), 
                        float(self.position[0]), float(self.position[1]), float(self.position[2])]
            self.pub_commands.publish(msg) #publish commands for drone controller
            
            # #This is a timer so the drones stay in one location for a few seconds 
            # if self.is_within_threshold(self.position, self.target_position) and not self.startTimer:
            #     self.startTimer = True
            #     self.startTime = time.time()
            #     #print("Threshold met")
            # else:
            #     self.startTimer = False
            
            # # Check if the drone has reached the target position NEW, stay for 3 seconds 
            # if self.is_within_threshold(self.position, self.target_position) and self.startTimer and time.time() - self.startTime < 3:
            #     self.get_logger().info(f"Reached target position {self.target_position}")
            #     print(f"cf2: Reached target position {self.target_position}")
            #     # Move to the next target if available
            #     self.current_target_index += 1
            #     self.startTime = time.time()
            #     self.startTimer = False
            #     if self.current_target_index < len(self.target_positions):
            #         self.target_position = self.target_positions[self.current_target_index]
            #         self.get_logger().info(f"Moving to next target position {self.target_position}")
            #         print(f"cf2: Moving to next target position {self.target_position}")
            #     else:
            #         self.get_logger().info("All target positions reached. Hovering.")
            #         print("cf2: All target positions reached. Hovering.")
            #         # Optionally, set hover or stop commands

            #new threshold logic
            if self.is_within_threshold(self.position, self.target_position): #if drone is at desired position
                if not self.startTimer: #if the timer for hovering has not started, start it
                    print("cf2 timer started")
                    self.startTimer = True
                    self.startTime = time.time()
                elif time.time() - self.startTime >= 3: #if the drone has been at the desired position for 3 seconds
                    if not self.threshold_met:
                        self.threshold_met = True
                        self.publish_threshold_met()
                        print("cf2: Threshold met")
                    if self.cf1_threshold_met and self.cf1_threshold_met:
                        self.current_target_index += 1
                        if self.current_target_index < len(self.target_positions):  #if these is another target position, move to it
                            self.target_position = self.target_positions[self.current_target_index]
                            self.get_logger().info(f"cf2:Moving to next target position {self.target_position}")
                            print(f"cf2: moving to next position: {self.target_position}")
                    else:
                        print("Waiting for cf1 to reach threshold.")

                    # else:
                    #     self.get_logger().info("All target positions reached. Hovering.")

        # error handling for unexpected pose message
        else:
            self.get_logger().warn(f"Received pose in unexpected frame: {msg.header.frame_id}")
            pass
    

    #helps drone move from one position to the other 
    def is_within_threshold(self, position, target_position):
        # Calculate Euclidean distance between the current position and target position
        dist = np.linalg.norm(np.array(position) - np.array(target_position))
        return dist <= self.threshold
    
    #callback function that gets the threshold_met value from cf1
    def cf1_threshold_met_callback(self, msg):
        self.cf1_threshold_met = msg.data #set the boolean to the value recieved from cf2

    #publishes the threshold_met value to communicate with cf1
    def publish_threshold_met(self):
        threshold_met_msg = Bool()
        threshold_met_msg.data = self.threshold_met
        self.pub_threshold_met.publish(threshold_met_msg)

    # rotational control
    def calculate_yawrate(self):
        # (P term)
        r = R.from_quat(self.orientation_quat)
        pitch, yaw, roll = r.as_euler('xyz', degrees = True)
        self.current_orientation = yaw
        rot_error = self.target_orientation - self.current_orientation

        # workaround logic to determine direction of rotation (bc of quaternions)
        if abs(self.orientation_quat[1]) > 0.7: # this value is specific to a certain set up orientation
            self.k_p_rot_sign = 1
        else:
            self.k_p_rot_sign = -1

        yawrate = self.k_p_rot_sign * self.k_p_rot * rot_error
        yawrate = max(self.min_yawrate, min(yawrate, self.max_yawrate))
        return yawrate
    
    # X Axis control
    def calculate_pitch(self):
        # (P term)
        self.cur_x_error = self.target_position[0] - self.position[0]
        if -0.01 <= self.cur_x_error <= 0.01: #if error is within margin, set to 0 (in meters; 0.01 = 1cm)
            self.cur_x_error = 0
        x_fp = self.k_p_x * self.cur_x_error
        # print(f"x_fp: {x_fp}")
        
        # I term
        future_int_x_error = self.int_x_error + 0.5 * (self.prev_x_error + self.cur_x_error) * self.t
        if abs(future_int_x_error * self.k_i_x) < self.int_x_max:
            self.int_x_error = future_int_x_error
        x_fi = self.k_i_x * self.int_x_error
        # print(f"x_fi: {x_fi}")

        # D term
        x_error_dif = self.cur_x_error - self.prev_x_error
        x_fd = self.k_d_x * (x_error_dif) / self.t
        # print(f"x_fd: {x_fd}")
        self.prev_x_error = self.cur_x_error

        pitch = x_fp + x_fi + x_fd
        pitch = max(self.min_pitch, min(pitch, self.max_pitch))
        return pitch
    
    # Y axis control 
    def calculate_thrust(self):
        # P term:
        self.cur_y_error = self.target_position[1]- self.position[1]

        # I term:
        if -0.01 <= self.cur_y_error <= 0.01: #if error is within margin, set to 0 (in meters; 0.01 = 1cm)
            self.cur_y_error = 0

        y_fp = self.k_p_y * self.cur_y_error
        #print(f"y_fp: {y_fp}")

        #calculate what k_i would be
        future_int_y_error = self.int_y_error + 0.5 * (self.prev_y_error + self.cur_y_error) * self.t
        #if the calculated value is within range, update int_y_error
        if abs(future_int_y_error * self.k_i_y) < self.int_y_max:
            self.int_y_error = future_int_y_error
        #otherwise keep self.int_y_error the same
        y_fi = self.k_i_y * self.int_y_error
        #print(f"y_fi: {y_fi}")

        # D term
        error_dif = self.cur_y_error - self.prev_y_error
        y_fd = self.k_d_y * (error_dif) / self.t
        #print(f"y_fd: {y_fd}")
        self.prev_y_error = self.cur_y_error

        thrust = self.hover + y_fp + y_fi + y_fd
        # Clamp thrust to valid range 
        thrust = int(max(self.min_thrust, min(thrust, self.max_thrust)))
        return thrust
    
    def calculate_roll(self):
        # P term
        self.cur_z_error = self.target_position[2] - self.position[2]
        z_fp = self.k_p_z * self.cur_z_error
        #print(f"z_fp: {z_fp}")

        # I term
        future_int_z_error = self.int_z_error + 0.5 * (self.prev_z_error + self.cur_z_error) * self.t
        if abs(future_int_z_error * self.k_i_z) < self.int_z_max:
            self.int_z_error = future_int_z_error
        z_fi = self.k_i_z * self.int_z_error
        #print(f"z_fi: {z_fi}")

        # D Term
        z_error_dif = self.cur_z_error - self.prev_z_error
        z_fd = self.k_d_z * (z_error_dif) / self.t
        #print(f"z_fd: {z_fd}")
        self.prev_z_error = self.cur_z_error

        roll = z_fp + z_fi + z_fd
        roll = max(self.min_roll, min(roll, self.max_roll))
        return roll
   
    def get_position(self):
        return self.position
    
def main(args=None):
    rclpy.init(args=args)

    optitrack_subscriber2 = OptiTrackSubscriber2()

    rclpy.spin(optitrack_subscriber2)

    optitrack_subscriber2.destroy_node()
    rclpy.shutdown()



      

if __name__ == '__main__':
    main()  
