import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Load data
data = pd.read_csv('mag_data4.csv')

# 3D plot to visualize the sphere
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(data['mag_x'], data['mag_y'], data['mag_z'], s=1)

mean_x = np.mean(data['mag_x'])
mean_y = np.mean(data['mag_y'])
mean_z = np.mean(data['mag_z'])

# Plot the mean point as a larger red dot
ax.scatter([mean_x], [mean_y], [mean_z], 
          s=100, c='red', marker='o', 
          edgecolors='black', linewidth=2,
          label=f'Mean ({mean_x:.0f}, {mean_y:.0f}, {mean_z:.0f})')

# Optional: Add lines from origin to mean point to visualize offset
ax.plot([0, mean_x], [0, mean_y], [0, mean_z], 
        'r--', alpha=0.7, linewidth=2, label='Offset from Origin')

# Plot origin point for reference
ax.scatter([0], [0], [0], 
          s=80, c='green', marker='s', 
          edgecolors='black', linewidth=1,
          label='Origin (0,0,0)')
ax.set_xlabel('Mag X')
ax.set_ylabel('Mag Y') 
ax.set_zlabel('Mag Z')
print(mean_x, mean_y, mean_z)
plt.show()

