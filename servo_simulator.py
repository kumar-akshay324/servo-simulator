from __future__ import print_function

import random
from motor import MotorModel
import matplotlib.pyplot as plt
import argparse
import sys, time
import warnings
warnings.filterwarnings("ignore")

# Command to Run in Terminal

# python3 servo_simulator.py [target angular position] -t [type of motor]  -s [initial angular position]
#
# target angular position: float value between 0 - 90 degrees -   Compulsory Input
#
# type of motor	:	  string value - Optional Input
#                     Separately Excited Motor Model		:	'sep'
#                     Armature Controlled Motor Model		:	'armc'	(Default)
#
# initial angular position	:	float value between 0- 90 degrees - Optional Input

class SimulateServo:

    def __init__(self):

        self.motor_object = MotorModel()

        # Make the robot joint spawn in a random angular position between 0-90 degrees (Range is said only for the sake of simplicity)
        self.current_position = random.random()*90

        self.angular_velocity = 0
        self.angle_min = 0
        self.angle_max = 90
        self.del_t = 0.001
        self.sum_pos_error = 0

        # self.PID_Control_Values = {'sep':[12, 0.7, 0.2], 'armc': [120, 5, 3]}
        # 5, 0.2, 0
        self.PID_Control_Values = {'sep':[3, 0.18, 20], 'armc': [60, 4, 3]}

    def PIDcontroller(self, Kp, Kd, Ki):
        ''' Implementing the PID controller '''

        pos_error = self.target_position - self.current_position
        pos_error_dot = self.angular_velocity
        self.sum_pos_error += pos_error*self.del_t

        computed_Torque = - Kp*pos_error - Kd*pos_error_dot - Ki*self.sum_pos_error

        return pos_error, computed_Torque

    def limit_angle(self):
        '''Clip the angular overshoot to bring it within limits'''
        self.current_position = min(max(self.current_position, self.angle_min), self.angle_max)

    def read_argument(self):
        '''Reads the user directions for target angular position and type of motor model to be used'''

        parser = argparse.ArgumentParser(description='Parse various common line arguments')

        parser.add_argument('p', type=float,help='Angular Position to converge to')
        parser.add_argument('-t', type=str, help='Type of Motor', default = 'armc')
        parser.add_argument('-s', type=float, help='Initial angular position', default = self.current_position)

        args = parser.parse_args()

        self.target_position = args.p
        self.motor_type = args.t.lower()

        if  not (self.angle_min < self.target_position < self.angle_max):
            print ("Shutting Down - Target Angule Position out of range")
            sys.exit()

        if self.motor_type not in self.PID_Control_Values.keys():
            self.motor_type = 'armc'
            print ("Running the simulation on default Armature Controlled Motor")

        self.current_position = args.s

        print ("Target Position: ", self.target_position, "\t\t", "Motor Type: ", self.motor_type)


    def main(self):
        '''Simulate the complete control - Simulation Step Size = 1 milli second '''
        time.sleep(10)
        # Reading the input from user
        self.read_argument()

        plt.title("PID based Servo Simulator")
        plt.xlabel("Time Steps")
        plt.ylabel("Angular Position in Degrees")
        plt.grid()
        motor_out = {'sep': "Separately Excited DC Motor\n", 'armc': "Armature Controlled DC Motor\n"}

        position_stream = []
        PID_parameters =  self.PID_Control_Values[self.motor_type]
        condition = True

        print ("PID Controller running for", motor_out[self.motor_type])

        while condition ==True:

            # Compute the PID controller torque output based on current configuration
            position_error, tau = self.PIDcontroller(*PID_parameters)
            # print ("Current position error is: ", position_error, "\t\t", "PID computed Torque is:", tau )

            # Obtain the angular velocity simulated by the Motor -  Converting RPM to degrees/second
            self.angular_velocity = self.motor_object.calc_omega(tau, self.motor_type)*6
            # print ("Generated Angular Velocity in degrees/second is:", self.angular_velocity)

            # Update the current angular position using the obtained angular position and time-step
            self.current_position += self.angular_velocity*self.del_t
            self.limit_angle()

            print ("Current angular position is:", self.current_position, "\n")

            # Append the obtained angular position to a list
            position_stream.append(self.current_position)

            # Plot the variation in current angular position over time - to show continuously updating plot
            plt.plot(position_stream)
            plt.pause(0.001)

            # Termination criteria - Angular velocity less than 2 degree/second and Angular Position error is less  than 0.2. degrees
            if abs(self.angular_velocity)<2 and abs(position_error)<0.2:
                condition = False

        print("\n\nConverged to Desired Position")
        print ("Target Angular Position: ", self.target_position, "Final Obtained Angular Position: ", self.current_position)
        plt.show(block="False")

if __name__ == '__main__':
    simulate_object = SimulateServo()
    simulate_object.main()





