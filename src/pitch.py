from scipy.optimize import fsolve
import math

class Trajectory:
    # constants
    PROJECTILE_AREA = 4/3 * math.pi * pow((17.3 * pow(10,-3)),3)  # m^2
    DRAG_COEFFICIENT = 0.3
    RHO = 1.28  # density of air in kg/m^3
    C_D = RHO * DRAG_COEFFICIENT * (PROJECTILE_AREA / 2)
    MASS = 0.003  # 3 grams
    V_LAUNCH = 85  # m/s
    GRAVITY = 9.81

    MAX_X_RANGE = 30  #
    MAX_Y_RANGE = 0

    def __init__(self):
        self.x_range = (
            self.MAX_X_RANGE
        )  # TODO: replace value with value obtained from LiDAR
        self.y_range = (
            self.MAX_Y_RANGE
        )  # TODO: where are we getting the value from? (RUTVIK)
        self.pitch = 0  # pitch angle in radians
        self.yaw = 0  # yaw angle in radians
        self.yaw_angle = 0

    def get_pitch(self, p):
        # calculates the pitch angle to send back to the motor

        theta, t = p

        vx = self.V_LAUNCH * math.cos(theta)
        vy = self.V_LAUNCH * math.sin(theta)

        fx = -self.C_D * self.V_LAUNCH * vx
        fy = -self.C_D * self.V_LAUNCH * vy - self.MASS * self.GRAVITY

        ax = fx / self.MASS
        ay = fy / self.MASS

        x_dist = vx * t + 0.5 * ax * pow(t, 2) - self.x_range
        y_dist = vy * t + 0.5 * ay * pow(t, 2) - self.y_range

        return (x_dist, y_dist)

    def solve(self):
        theta, t = fsolve(self.get_pitch, (25, 1))
        print(theta, t)