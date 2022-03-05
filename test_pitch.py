from src.pitch import Trajectory

traj = Trajectory(20, 0)
print(traj.x_range, traj.y_range)
print(traj.solve())
