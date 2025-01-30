import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import csv
import os
from datetime import datetime



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
link_uri2 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')


logging.basicConfig(level=logging.ERROR)


class MinimalSubscriber(Node):


    def __init__(self):
        super().__init__('cf_driver')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/cf1/commands',
            self.listener_callback,
            10)
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/cf2/commands',
            self.listener_callback2,
            10)
        
        
        self.subscription  # prevent unused variable warning

        cflib.crtp.init_drivers()

        #Crazyflie drone 1 (number 8) connection
        self._cf1 = Crazyflie(rw_cache='./cache')
        self._cf1.connected.add_callback(self._connected)
        self._cf1.disconnected.add_callback(self._disconnected)
        self._cf1.connection_failed.add_callback(self._connection_failed)
        self._cf1.connection_lost.add_callback(self._connection_lost)


        #Crazyflie drone 2 (number 7) connection
        self._cf2 = Crazyflie(rw_cache='./cache')
        self._cf2.connected.add_callback(self._connected)
        self._cf2.disconnected.add_callback(self._disconnected)
        self._cf2.connection_failed.add_callback(self._connection_failed)
        self._cf2.connection_lost.add_callback(self._connection_lost)


        # Flight variables for both drones
        self.roll1, self.pitch1, self.yawrate1, self.thrust1 = 0.0, 0.0, 0.0, 0
        self.roll2, self.pitch2, self.yawrate2, self.thrust2 = 0.0, 0.0, 0.0, 0


        # Position variables
        self.x_position1, self.y_position1, self.z_position1 = 0.0, 0.0, 0.0
        self.x_position2, self.y_position2, self.z_position2 = 0.0, 0.0, 0.0  # 0.25m offset for drone 2




        # limit flight time for testing
        self.flight_duration = 20.0 #in seconds


        # constant command values for testing
        self.const_thrust = 44000
        self.const_roll = 0 #-3,3 range
        self.const_pitch = 0 #-3,3 range
        self.const_yawrate = 0 #-15,15 range


        self._cf1.open_link(link_uri)
        self._cf2.open_link(link_uri2)
        


        #send initial command of zeros (needed for crazyflie protocol)
        self._cf1.commander.send_setpoint(0, 0, 0, 0)
        self._cf2.commander.send_setpoint(0, 0, 0, 0)


        print(f'Connecting to {link_uri} and {link_uri2}')
        


        # Graphing and data collection (1st cf drone)
        self.timestamp_data = []
        self.cur_x_data = []
        self.cur_y_data = []
        self.cur_z_data = []

        self.thrust_data = []
        self.y_fp_data = []
        self.y_fi_data = []

        # Graphing and data collection 2 (2nd cf drone)
        self.timestamp_data2 = []
        self.cur_x_data2 = []
        self.cur_y_data2 = []
        self.cur_z_data2 = []

        self.thrust_data2 = []
        self.y_fp_data2 = []
        self.y_fi_data2 = []

        self.start_time = time.time()

    def listener_callback2(self, msg):
       #listener for the second drone to optitrack_2

        # ensure commands for all axis are recieved
        if len(msg.data) >= 7:
            self.roll2 = msg.data[0]
            self.pitch2 = msg.data[1]
            self.yawrate2 = msg.data[2]
            self.thrust2 = int(msg.data[3]) #thrust needs to be an int


            self.x_position2 = msg.data[4]
            self.y_position2 = msg.data[5]
            self.z_position2 = msg.data[6]


            # record data for graphing
            current_time = time.time()
            elapsed_time = current_time - self.start_time
            self.timestamp_data2.append(elapsed_time)
            self.cur_x_data2.append(self.x_position2)
            self.cur_y_data2.append(self.y_position2)
            self.cur_z_data2.append(self.z_position2)
            self.thrust_data2.append(self.thrust2)
            # self.y_fp_data.append(y_fp)
            # self.y_fp_data.append(y_fi)
        else:
            print("Error: incorrect msg length")

        

    # this function is called each time a 'command' message is received
    def listener_callback(self, msg):
        #listener for the first drone, optireack_subscriber 

        # ensure commands for all axis are recieved
        if len(msg.data) >= 7:
            self.roll1 = msg.data[0]
            self.pitch1 = msg.data[1]
            self.yawrate1 = msg.data[2]
            self.thrust1 = int(msg.data[3]) #thrust needs to be an int


            self.x_position1 = msg.data[4]
            self.y_position1 = msg.data[5]
            self.z_position1 = msg.data[6]

            #print(f"Received: Roll = {self.roll}, Pitch = {self.pitch}, Yawrate = {self.yawrate}, Thrust = {self.thrust}")
            #print(f"x: {self.x_position}, y: {self.y_position}, z: {self.z_position}")


            # record data for graphing
            current_time = time.time()
            elapsed_time = current_time - self.start_time
            self.timestamp_data.append(elapsed_time)
            self.cur_x_data.append(self.x_position1)
            self.cur_y_data.append(self.y_position1)
            self.cur_z_data.append(self.z_position1)
            self.thrust_data.append(self.thrust1)
            # self.y_fp_data.append(y_fp)
            # self.y_fp_data.append(y_fi)
        else:
            print("Error: incorrect msg length")

        print(f"thrust 1 = {self.thrust1}, thrust 2 = {self.thrust2}")
        self.run_motors() #call function to send commands to crazyflie
        
        
    # function to run motors called each time data is recieved
    def run_motors(self):
        #comment these out to run REAL values
        # self.roll = self.const_roll
        # self.pitch = self.const_pitch
        # self.yawrate = self.const_yawrate
        # self.thrust = self.const_thrust


        # Send commands to drone 1
        self._cf1.commander.send_setpoint(self.roll1, self.pitch1, self.yawrate1, self.thrust1)
        print(f"Drone 1: Roll = {self.roll1}, Pitch = {self.pitch1}, Yawrate = {self.yawrate1}, Thrust = {self.thrust1}")

        # Send commands to drone 2
        self._cf2.commander.send_setpoint(self.roll2, self.pitch2, self.yawrate2, self.thrust2)
        print(f"Drone 2: Roll = {self.roll2}, Pitch = {self.pitch2}, Yawrate = {self.yawrate2}, Thrust = {self.thrust2}")




        #self._cf.commander.send_setpoint(self.const_roll, self.const_pitch, self.const_yawrate, self.const_thrust)


    # Unused function that uses Threading
    def _send_thrust_command(self):
        
        while True:
            self._cf1.commander.send_setpoint(self.roll1, self.pitch1, self.yawrate1, self.thrust1)
            self._cf2.commander.send_setpoint(self.roll2, self.pitch2, self.yawrate2, self.thrust2)
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


        # set to the current thrust value
        rampdown_thrust1 = self.thrust1
        rampdown_thrust2 = self.thrust2
        rclpy.shutdown()  # Shutdown ROS 2 before ramping down to prevent override


        # ramp down thrust until reaching threshold to cut power, ideally on the ground
        while rampdown_thrust1 > 42500 and rampdown_thrust2 > 42500:
            if rampdown_thrust1 > 42500:
                rampdown_thrust1 -= 150
            if rampdown_thrust2 > 45000:
                rampdown_thrust2 -= 150

            print(f"running ramp down: {rampdown_thrust1}")
            print(f"running ramp down: {rampdown_thrust2}")
            self._cf1.commander.send_setpoint(self.roll1, self.pitch1, self.yawrate1, rampdown_thrust1)
            self._cf2.commander.send_setpoint(self.roll2, self.pitch2, self.yawrate2, rampdown_thrust2)
            time.sleep(0.1)
        self._cf1.commander.send_setpoint(0, 0, 0, 0)  # Stop the motors
        self._cf2.commander.send_setpoint(0, 0, 0, 0)  # Stop the motors
        self._cf1.close_link()  # Disconnect from the Crazyflie
        self._cf2.close_link()  # Disconnect from the Crazyflie




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

    # Ensure the directory exists
    os.makedirs('data', exist_ok=True)

        # Generate a timestamp for the filenames
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    # Save data for later use
    with open(f'data/flight_data1_{timestamp}.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'cur_x', 'cur_y', 'cur_z', 'thrust'])
        for i in range(len(minimal_subscriber.timestamp_data)):
            writer.writerow([
                minimal_subscriber.timestamp_data[i],
                minimal_subscriber.cur_x_data[i],
                minimal_subscriber.cur_y_data[i],
                minimal_subscriber.cur_z_data[i],
                minimal_subscriber.thrust_data[i]
            ])

    with open(f'data/flight_data2_{timestamp}.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'cur_x', 'cur_y', 'cur_z', 'thrust'])
        for i in range(len(minimal_subscriber.timestamp_data2)):
            writer.writerow([
                minimal_subscriber.timestamp_data2[i],
                minimal_subscriber.cur_x_data2[i],
                minimal_subscriber.cur_y_data2[i],
                minimal_subscriber.cur_z_data2[i],
                minimal_subscriber.thrust_data2[i]
            ])
    # minimal_subscriber._cf.close_link()  # Disconnect from the Crazyflie


    # Plotting Graph one
    # Currently plotting Y vs time, X vs time, Z vs time and Thrust vs time
    plt.figure()
    # plt.title('cf1 graphs')
    plt.subplot(2,2,1)
    plt.plot(minimal_subscriber.timestamp_data, minimal_subscriber.cur_y_data, 'r-', label='Y Position')
    plt.xlabel('Time(s)')
    plt.ylabel('Y Position (m)')
    plt.title('Y Position Over Time')
    plt.ylim(bottom=0)
    plt.legend()


    plt.subplot(2,2,2)
    plt.plot(minimal_subscriber.timestamp_data, minimal_subscriber.cur_x_data, 'r-', label='X Position')
    plt.xlabel('Time(s)')
    plt.ylabel('X Position (m)')
    plt.title('X Position Over Time')
    plt.ylim(bottom=0)
    plt.legend()


    plt.subplot(2,2,3)
    plt.plot(minimal_subscriber.timestamp_data, minimal_subscriber.cur_z_data, 'r-', label='Z Position')
    plt.xlabel('Time(s)')
    plt.ylabel('Z Position (m)')
    plt.title('Z Position Over Time')
    plt.ylim(bottom=0)
    plt.legend()


    # graphing thrust over time
    plt.subplot(2,2,4)
    plt.plot(minimal_subscriber.timestamp_data, minimal_subscriber.thrust_data, 'b-', label='Thrust')
    plt.xlabel('Time(s)')
    plt.ylabel('Thrust')
    plt.title('Thrust Over Time')
    plt.legend()
    plt.show()
    print("Plotted")

    # Plotting Graph two
    # Currently plotting Y vs time, X vs time, Z vs time and Thrust vs time
    plt.figure()
    # plt.title('cf2 graphs')
    plt.subplot(2,2,1)
    plt.plot(minimal_subscriber.timestamp_data2, minimal_subscriber.cur_y_data2, 'r-', label='Y Position')
    plt.xlabel('Time(s)')
    plt.ylabel('Y Position (m)')
    plt.title('Y Position Over Time')
    plt.ylim(bottom=0)
    plt.legend()


    plt.subplot(2,2,2)
    plt.plot(minimal_subscriber.timestamp_data2, minimal_subscriber.cur_x_data2, 'r-', label='X Position')
    plt.xlabel('Time(s)')
    plt.ylabel('X Position (m)')
    plt.title('X Position Over Time')
    plt.ylim(bottom=0)
    plt.legend()


    plt.subplot(2,2,3)
    plt.plot(minimal_subscriber.timestamp_data2, minimal_subscriber.cur_z_data2, 'r-', label='Z Position')
    plt.xlabel('Time(s)')
    plt.ylabel('Z Position (m)')
    plt.title('Z Position Over Time')
    plt.ylim(bottom=0)
    plt.legend()


    # graphing thrust over time
    plt.subplot(2,2,4)
    plt.plot(minimal_subscriber.timestamp_data2, minimal_subscriber.thrust_data2, 'b-', label='Thrust')
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
