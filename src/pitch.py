# pitch calculation
from cmath import sin, cos
from sympy import *

class Trajectory:
    # constants
    PROJECTILE_AREA = 0.00202682991                     # m^2
    DRAG_COEFFICIENT = 0.2
    RHO = 1.2                                           # density of air in kg/m^3
    C_D = RHO * DRAG_COEFFICIENT * (PROJECTILE_AREA/2)
    MASS = 0.003                                        # 3 grams
    V_LAUNCH = 70                                       # m/s
    GRAVITY = 9.81

    MAX_X_RANGE = 15                                    # 
    MAX_Y_RANGE = 3

    def __init__(self):
        self.pitch = 0                                  # pitch angle in radians
        self.yaw = 0                                    # yaw angle in radians
    
    def get_pitch(self, evo_range):
        # calculates the pitch angle to send back to the motor
        theta, t = symbols('theta, t')
        vx = self.V_LAUNCH * cos(theta)
        vy = self.V_LAUNCH * sin(theta)

        fx = -self.C_D * self.V_LAUNCH * vx
        fy = -self.C_D * self.V_LAUNCH * vy - self.MASS * self.GRAVITY

        ax = fx / self.MASS
        ay = fy / self.MASS

        

