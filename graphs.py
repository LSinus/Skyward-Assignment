import pandas as pd
import matplotlib.pyplot as plt

import math

df = pd.read_csv('build/out.csv')

x_axis = df["time"]
altitude = df["altitude"]
acc_x = df["acc_x"]
vel_x = df["vel_x"]
roll = df["roll"]
pitch = df["pitch"]
yaw = df["yaw"]

graphs, axes = plt.subplots(2,3, figsize=(12, 6))

axes[0, 0].plot(x_axis, altitude, color='black', label='Altitude')
axes[0, 0].set_title('Altitude')
axes[0, 0].set_xlabel('seconds')
axes[0, 0].set_ylabel('meters')
axes[0, 0].legend()

axes[0, 1].plot(x_axis, vel_x, color='gold', label='Velocity')
axes[0, 1].set_title('Vertical velocity')
axes[0, 1].set_xlabel('seconds')
axes[0, 1].set_ylabel('m/s')
axes[0, 1].legend()

axes[0, 2].plot(x_axis, acc_x, color='brown', label='Acceleration')
axes[0, 2].set_title('Vertical Acceleration')
axes[0, 2].set_xlabel('seconds')
axes[0, 2].set_ylabel('m/s^2')
axes[0, 2].legend()

axes[1, 0].plot(x_axis, roll, color='red', label='Roll')
axes[1, 0].set_title('Roll')
axes[1, 0].set_xlabel('seconds')
axes[1, 0].set_ylabel('degrees')
axes[1, 0].legend()

axes[1, 1].plot(x_axis, pitch, color='green', label='Pitch')
axes[1, 1].set_title('Pitch')
axes[1, 1].set_xlabel('seconds')
axes[1, 1].set_ylabel('degrees')
axes[1, 1].legend()

axes[1, 2].plot(x_axis, yaw, color='blue', label='Yaw')
axes[1, 2].set_title('Yaw')
axes[1, 2].set_xlabel('seconds')
axes[1, 2].set_ylabel('degrees')
axes[1, 2].legend()

plt.tight_layout()
plt.show()

