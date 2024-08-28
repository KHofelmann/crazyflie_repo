# opti_track_subscriber.py
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt16
from std_msgs.msg import Float32MultiArray

import matplotlib.pyplot as plt
import time

import numpy as np
from scipy.spatial.transform import Rotation as R


class OptiTrackSubscriber(Node):
    def __init__(self):
        super().__init__('opti_track_subscriber')

        # Customize QoS settings to match the publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # This needs to be BEST_EFFORT to read messages
            history=HistoryPolicy.KEEP_LAST,         # Keep the last message
            depth=10                                 # Queue up to 10 messages
        )

        self.subscription = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/cf/pose',
            self.listener_callback,
            qos_profile
        
        )
        #message to send flight commands to crazyflie program
        self.pub_commands = self.create_publisher(Float32MultiArray, '/cf1/commands', 10)

        self.position = [0.0, 0.0, 0.0] #current position of drone, automatically updated
        self.target_position = [0.5, 0.5, 0.5]

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
        self.hover = 48000 #originally 46500     
        self.max_thrust = 50000
        self.min_thrust = 42000
        self.k_p_y = 200 #4000 seemed ideal value at the time
        self.k_i_y = 2000 #extra amount of thrust wanted (originally 2000)
        self.k_d_y = 5000

        self.cur_y_error = 0.0
        self.prev_y_error = 0.0
        self.int_error = 0.0
        self.int_max = 2000 # maximum added thrust from integral component

        # values for horizontal X (pitch) PID
        self.k_p_x = 2.0
        self.max_pitch = 3.0
        self.min_pitch = -3.0

        # values for horizontal Z (roll) PID
        self.k_p_z = 2.0
        self.max_roll = 3.0
        self.min_roll = -3.0

        # # Graphing and data collection
        # self.timestamp_data = []
        # self.cur_x_data = []
        # self.cur_y_data = []
        # self.cur_z_data = []

        # self.thrust_data = []
        # self.y_fp_data = []
        # self.y_fi_data = []
        # self.start_time = time.time()

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

            # Yaw control (P term)
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

            # X control (P term)
            x_error = self.target_position[0] - self.position[0]
            pitch = self.k_p_x * x_error
            pitch = max(self.min_pitch, min(pitch, self.max_pitch))
            #print(f"Pitch: {pitch}")

            # Y control (with P and I terms)
            # P term:
            self.cur_y_error = self.target_position[1]- self.position[1]

            # I term:
            if -0.01 <= self.cur_y_error <= 0.01: #if error is within margin, set to 0 (in meters; 0.01 = 1cm)
                self.cur_y_error = 0
            # if the y error is 0, reset the integral error to 0
            # if self.cur_y_error == 0:
            #     self.int_error = 0

            y_fp = self.k_p_y * self.cur_y_error

            # clamp for integral error
            # *NEED TO FIX LOGIC*

            #calculate what k_i would be
            future_int_error = self.int_error + 0.5 * (self.prev_y_error + self.cur_y_error) * self.t
            #if the calculated value is within range, update int_error
            if abs(future_int_error * self.k_i_y) < self.int_max:
                self.int_error = future_int_error
            #otherwise keep self.int_error the same

            # if self.k_i_y * self.int_error < self.int_max:
            #     self.int_error = self.int_error + 0.5 * (self.prev_y_error + self.cur_y_error) * self.t
            y_fi = self.k_i_y * self.int_error
            print(f"y_fi: {y_fi}")


            error_dif = self.cur_y_error - self.prev_y_error
            y_fd = self.k_d_y * (error_dif) / self.t
            #print(f"y_kd: {y_fd}")
            self.prev_y_error = self.cur_y_error

            thrust = self.hover + y_fp + y_fi + y_fd

            # Clamp thrust to valid range 
            thrust = int(max(self.min_thrust, min(thrust, self.max_thrust)))
            print(f"thrust: {thrust}")

             # Z control
            z_error = self.target_position[2] - self.position[2]
            roll = self.k_p_z * z_error
            roll = max(self.min_roll, min(roll, self.max_roll))
            #print(f"Roll: {roll}")

            # # Record data for graphing
            # current_time = time.time()
            # elapsed_time = current_time - self.start_time
            # self.timestamp_data.append(elapsed_time)
            # self.cur_x_data.append(self.position[0])
            # self.cur_y_data.append(self.position[1])
            # self.cur_z_data.append(self.position[2])
            # self.y_fp_data.append(y_fp)
            # self.y_fp_data.append(y_fi)

            # create message of type Float Array (all values need to be floats)
            msg = Float32MultiArray()
            msg.data = [float(roll), float(pitch), float(yawrate), float(thrust), 
                        float(self.position[0]), float(self.position[1]), float(self.position[2])]
            self.pub_commands.publish(msg) #publish commands for drone controller
        # error handling for unexpected pose message
        else:
            self.get_logger().warn(f"Received pose in unexpected frame: {msg.header.frame_id}")
            pass
   
    def get_position(self):
        return self.position
    
def main(args=None):
    rclpy.init(args=args)

    optitrack_subscriber = OptiTrackSubscriber()

    rclpy.spin(optitrack_subscriber)

    optitrack_subscriber.destroy_node()
    rclpy.shutdown()

    # try:
    #     rclpy.spin(optitrack_subscriber)
    # except KeyboardInterrupt:
    #     optitrack_subscriber.get_logger().info('Node stopped by user')
    # finally:
    #     # Destroy the node explicitly
    #     # (optional - otherwise it will be done automatically
    #     # when the garbage collector destroys the node object)
    #     optitrack_subscriber.destroy_node()
    #     rclpy.shutdown()

        # # Plotting Graphs
        # # Currently plotting cur_y vs time
        # plt.figure()
        # plt.plot(optitrack_subscriber.timestamp_data, optitrack_subscriber.cur_y_data, 'r-', label='Y Position')
        # plt.xlabel('Time(s)')
        # plt.ylabel('Y Position (m)')
        # plt.title('Y Position Over Time')
        # plt.legend()
        # plt.show()
        # print("Plotted")

if __name__ == '__main__':
    main()  
