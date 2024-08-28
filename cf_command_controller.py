import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import logging
import time
import cflib
from threading import Thread, Timer
from cflib.crazyflie import Crazyflie
from cflib.utils import uri_helper

import matplotlib.pyplot as plt
import time

# the last digit of the radio address specifies which drone its connected (currently either 7 or 8)
link_uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E8')

logging.basicConfig(level=logging.ERROR)

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('cf_driver')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/cf1/commands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        cflib.crtp.init_drivers()

        self._cf = Crazyflie(rw_cache='./cache')
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        # variables to store flight commands
        self.pitch = 0.0
        self.roll = 0.0
        self.yawrate = 0.0
        self.thrust = 0.0

        # variables for current position
        self.x_position = 0.0
        self.y_position = 0.0
        self.z_position = 0.0

        # limit flight time for testing
        self.flight_duration = 40.0 #in seconds

        # constant command values for testing
        self.const_thrust = 44000 
        self.const_roll = 0 #-3,3 range
        self.const_pitch = 0 #-3,3 range
        self.const_yawrate = 0 #-15,15 range

        self._cf.open_link(link_uri)

        #send initial command of zeros (needed for crazyflie protocol)
        self._cf.commander.send_setpoint(0, 0, 0, 0) 

        print('Connecting to %s' % link_uri)

        # Graphing and data collection
        self.timestamp_data = []
        self.cur_x_data = []
        self.cur_y_data = []
        self.cur_z_data = []

        self.thrust_data = []
        self.y_fp_data = []
        self.y_fi_data = []
        self.start_time = time.time()

    # this function is called each time a 'command' message is received
    def listener_callback(self, msg):
        # ensure commands for all axis are recieved
        if len(msg.data) >= 7:
            self.roll = msg.data[0]
            self.pitch = msg.data[1]
            self.yawrate = msg.data[2]
            self.thrust = int(msg.data[3]) #thrust needs to be an int

            self.x_position = msg.data[4]
            self.y_position = msg.data[5]
            self.z_position = msg.data[6]
            
            #print(f"Received: Roll = {self.roll}, Pitch = {self.pitch}, Yawrate = {self.yawrate}, Thrust = {self.thrust}")
            #print(f"x: {self.x_position}, y: {self.y_position}, z: {self.z_position}")

            # record data for graphing
            current_time = time.time()
            elapsed_time = current_time - self.start_time
            self.timestamp_data.append(elapsed_time)
            self.cur_x_data.append(self.x_position)
            self.cur_y_data.append(self.y_position)
            self.cur_z_data.append(self.z_position)
            self.thrust_data.append(self.thrust)
            # self.y_fp_data.append(y_fp)
            # self.y_fp_data.append(y_fi)
        else:
            print("Error: incorrect msg length")
        self.run_motors() #call function to send commands to crazyflie

    # function to run motors called each time data is recieved
    def run_motors(self):
        #comment these out to run REAL values
        self.roll = self.const_roll
        self.pitch = self.const_pitch
        self.yawrate = self.const_yawrate
        # self.thrust = self.const_thrust

        self._cf.commander.send_setpoint(self.roll, self.pitch, self.yawrate, self.thrust)
        print(f"Running: Roll = {self.roll}, Pitch = {self.pitch}, Yawrate = {self.yawrate}, Thrust = {self.thrust}")

        #self._cf.commander.send_setpoint(self.const_roll, self.const_pitch, self.const_yawrate, self.const_thrust)

    # Unused function that uses Threading
    def _send_thrust_command(self):
        # Unlock startup thrust protectionc
        # self._cf.commander.send_setpoint(0, 0, 0, 0)
        # time.sleep(0.1)  # Give it a brief moment to register the unlock command

        while True:
            self._cf.commander.send_setpoint(self.roll, self.pitch, self.yawrate, self.thrust)
            
            time.sleep(0.05)  # Send commands at 20Hz

    def _connected(self, link_uri):
        """ This callback is called from the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to continuously send thrust commands.
        #temporatily commented out for debugging (not working)
        #Thread(target=self._send_thrust_command).start()

    # add a ramp down for safety
    def _stop_program(self):
        """Stops the Crazyflie and exits the program."""
        print("Timeout reached, stopping the Crazyflie.")

        thrust_mult = 1 #origionally 1 
        thrust_step = 500 #origional 500 
        thrust = 34000 #origional 30000

        while thrust >= 30000: #note, set to 40000 it flew to the roof 
            self._cf.commander.send_setpoint(self.roll, self.pitch, self.yawrate, thrust)
            time.sleep(0.1)
            if thrust >= 35000:
                thrust_mult = -1
            thrust += thrust_step * thrust_mult
        self._cf.commander.send_setpoint(0, 0, 0, 0)  # Stop the motors
        self._cf.close_link()  # Disconnect from the Crazyflie
        rclpy.shutdown()  # Shutdown ROS 2

    def _connection_failed(self, link_uri, msg):
        """Callback when initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    # Set up a timer to stop the program after set amount of time (in seconds)
    timeout_timer = Timer(minimal_subscriber.flight_duration, minimal_subscriber._stop_program)
    timeout_timer.start()

    rclpy.spin(minimal_subscriber)

    # Plotting Graphs
    # Currently plotting cur_y vs time
    plt.figure()
    plt.subplot(1,2,1)
    plt.plot(minimal_subscriber.timestamp_data, minimal_subscriber.cur_y_data, 'r-', label='Y Position')
    plt.xlabel('Time(s)')
    plt.ylabel('Y Position (m)')
    plt.title('Y Position Over Time')
    plt.legend()

    # graphing thrust over time
    plt.subplot(1,2,2)
    plt.plot(minimal_subscriber.timestamp_data, minimal_subscriber.thrust_data, 'b-', label='Thrust')
    plt.xlabel('Time(s)')
    plt.ylabel('Thrust')
    plt.title('Thrust Over Time')
    plt.legend()
    plt.show()
    print("Plotted")

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

   

if __name__ == '__main__':
    main()

