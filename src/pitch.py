from sympy import *

class Trajectory:
    # constants
    PROJECTILE_AREA = 0.00202682991                  # m^2
    DRAG_COEFFICIENT = 0.2
    RHO = 1.2                                           # density of air in kg/m^3
    C_D = RHO * DRAG_COEFFICIENT * (PROJECTILE_AREA/2)
    MASS = 0.003                                       # 3 grams
    V_LAUNCH = 70                                       # m/s
    GRAVITY = 9.81

    MAX_X_RANGE = 15.0                                    # 
    MAX_Y_RANGE = 3.0

    def __init__(self):
        self.x_range = self.MAX_X_RANGE                 # TODO: replace value with value obtained from LiDAR
        self.y_range = self.MAX_Y_RANGE                 # TODO: where are we getting the value from? (RUTVIK)
        self.pitch = 0                                  # pitch angle in radians
        self.yaw = 0                                    # yaw angle in radians
        self.yaw_angle = 0

    def get_pitch(self):
        print("starting get_pitch")
        # calculates the pitch angle to send back to the motor
        theta, t = symbols('theta, t')

        vx = self.V_LAUNCH * cos(theta)
        vy = self.V_LAUNCH * sin(theta)

        fx = -self.C_D * self.V_LAUNCH * vx
        fy = -self.C_D * self.V_LAUNCH * vy - self.MASS * self.GRAVITY

        ax = fx / self.MASS
        ay = fy / self.MASS

        x_dist = vx * t + 0.5 * ax  * pow(t,2)
        y_dist = vy * t + 0.5 * ay * pow(t,2)
        print("solving for theta and t")
        solved = solve([(x_dist - self.x_range), (y_dist - self.y_range)], [theta, t], simplify=True, rational=True)
        print(solved)
        


