import numpy as np

class MotorModel:

    def __init__(self):

        self.J_motor = 0.01                                         # Motor Rotational Inertia
        self.J_load = 0.02                                          # Load Rotational Inertia
        self.Kt = 1.1                                               # Torque Constant
        self.Ke = 1.2                                               # Electromotive Force Constant
        self.R = 1                                                  # Electrical Resistance in Ohms
        self.B_motor = 0.1                                          # Motor Viscous Friction Constant
        self.B_load = 0.11                                          # Load Viscous friction
        self.V = 12                                                 # Assuming that the motor runs at 12V DC supply

    def updated_J_B(self, n1=1, n2=1):
        ''' Updating the total inertia and total Damping
            Optional Input - Gear Ratio

            With no explicit details about the gear ratio, a 1:1 power transmission would be assumed
            '''
        self.J_total = self.J_motor + self.J_load*(n1/n2)**2
        self.B_total = self.B_motor + self.B_load*(n1/n2)**2


    def calc_omega(self, torque, motor_type= 'armc'):
        '''Calculate the simulated angular velocity as obtained from our hypothetical DC motor model

        Motor Model Transfer Function relates Torque with the output velocity

        Electical Motor model has inverse linear relationship between the Torque and output angular velocity for constant armature voltage V
        Dynamical Motor model has a Transfer Function that depends upon inertia J and friction B
        '''

        # This is an optional section where we could set the transmission gear ratio in case of a speed changer
        # n1, n2 = 1,2
        self.updated_J_B(n1=1, n2=1)

        if motor_type.lower() == 'sep':
            omega = (self.V - (torque*self.R)/self.Kt )/self.Ke
            # print ("PID Controller running for Electrical model of the Permannent Magnet DC Motor\n")

        elif motor_type.lower() == 'armc':
            omega = -(torque/self.J_total)*(np.exp(-self.B_total/self.J_total))
            # print ("PID Controller running for Dynamical model Permanent Magnet DC Motor\n")

        return omega




