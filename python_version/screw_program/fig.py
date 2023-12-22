import numpy as np
import math
import matplotlib.pyplot as plt

x = np.array(range(35))
y1 = [
    11.52, 10.52, 1.2, 0, 2.72, 0, 0.24, 3.8, 3.75, 4.44, 7.76, 6.92, 4.64,
    2.36, 1.64, 5.28, 4.96, 0, 7.12, 6.76, 2.2, 3.2, 1.68, 1.6, 11.6, 3.68,
    7.16, 13.2, 0, 6.64, 8.32, 0, 0, 0, 6.96
]
y2 = [
    11.04, 11.04, 7.2, 6.64, 10.12, 10.64, 6.84, 10.68, 9.275, 4.48, 7.88,
    11.12, 4.76, 3.28, 5.64, 5.92, 5.72, 5.04, 7.44, 6.6, 5.16, 3.6, 5.04,
    2.92, 10.84, 3.56, 7.52, 13.84, 6.88, 10.64, 8.24, 4.2, 5.12, 3.88, 7.64
]
y2, y1 = zip(*sorted(zip(y2, y1)))
penguin_means = {
    'Center of Mass':
    (11.52, 10.52, 1.2, 0, 2.72, 0, 0.24, 3.8, 3.75, 4.44, 7.76, 6.92, 4.64,
     2.36, 1.64, 5.28, 4.96, 0, 7.12, 6.76, 2.2, 3.2, 1.68, 1.6, 11.6, 3.68,
     7.16, 13.2, 0, 6.64, 8.32, 0, 0, 0, 6.96),
    'Our method':
    (11.04, 11.04, 7.2, 6.64, 10.12, 10.64, 6.84, 10.68, 9.275, 4.48, 7.88,
     11.12, 4.76, 3.28, 5.64, 5.92, 5.72, 5.04, 7.44, 6.6, 5.16, 3.6, 5.04,
     2.92, 10.84, 3.56, 7.52, 13.84, 6.88, 10.64, 8.24, 4.2, 5.12, 3.88, 7.64),
}
y1 = np.array(y1)
y2 = np.array(y2)
# x = np.arange(len(species))  # the label locations
width = 1  # the width of the bars
multiplier = 0
fig, ax = plt.subplots(layout='constrained')
plt.plot(x, y1[::-1], 'rs', linewidth=0.8, label='Center of Mass')
plt.plot(x, y2[::-1], 'go', linewidth=0.8, label='Our Implantation Position')
ax.fill_between(x, y1[::-1], 0, alpha=.3, linewidth=0, color='sandybrown')
ax.fill_between(x, y2[::-1], y1[::-1], alpha=.3, linewidth=0, color='skyblue')
plt.xticks(fontsize=24, weight='bold')
plt.yticks(fontsize=24, weight='bold')
# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Distance (mm)', fontsize=24, fontweight='bold')
ax.set_xlabel('Screw No.', fontsize=24, fontweight='bold')
ax.set_title('(a) Comparison of the Distance Margin', fontsize=24, fontweight='bold')
# ax.set_xticks(x + width, species)
ax.legend(loc='upper right', ncols=1, prop={"family": "Times New Roman", "size": 24, 'weight': "bold"})

plt.show()

x_depth = range(37)
y1_depth = [
    8.12, 20.63, 125.12, 110.76, 57.49, 95.64, 113.97, 63.76, 76.66, 23.75,
    40.48, 68.8, 9.2, 100.06, 138.14, 6.5, 93.81, 130.74, 68.29, 48.88, 96.46,
    95.46, 135.86, 13.76, 49.23, 22.93, 81.25, 65.34, 4.94, 51.02, 105.3,
    130.28, 66.73, 41.85, 22.58, 51.12, 25.55, 31.73
]
y2_depth = [
    71.44, 39.08, 135.51, 114.64, 138.46, 152.76, 118.17, 101.79, 75.73, 48.62,
    91.5, 111.87, 147.57, 141.61, 133.9, 139.9, 137.21, 69.76, 137.24, 200.54,
    126.3, 141.56, 50.82, 84.97, 60.02, 103, 117.07, 65.56, 52.06, 127.29,
    118.72, 72.72, 74.94, 56.7, 119.46, 47.79, 102.11
]
y2_depth, y1_depth = zip(*sorted(zip(y2_depth, y1_depth)))
y1_depth = np.array(y1_depth)
y2_depth = np.array(y2_depth)
fig, ax = plt.subplots(layout='constrained')
plt.plot(x_depth, y1_depth[::-1], 'rs', linewidth=0.8, label='$\it{v}_r$ by LSM and PCA')
plt.plot(x_depth, y2_depth[::-1], 'go', linewidth=0.8, label='Our Implantation Direction')
ax.fill_between(x_depth, y1_depth[::-1], 0, alpha=.3, linewidth=0, color='sandybrown')
ax.fill_between(x_depth, y2_depth[::-1], y1_depth[::-1], alpha=.3, linewidth=0, color='skyblue')
plt.xticks(fontsize=24, weight='bold')
plt.yticks(fontsize=24, weight='bold')
# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Depth (mm)', fontsize=24, fontweight='bold')
ax.set_xlabel('Screw No.', fontsize=24, fontweight='bold')
ax.set_title('(b) Comparison of the Entire Implantation Depths', fontsize=24, fontweight='bold')
# ax.set_xticks(x + width, species)
ax.legend(loc='upper right', ncols=1, prop={"family": "Times New Roman", "size": 24, 'weight': "bold"})
plt.show()


y1_f_depth_2 = [
    6.4, 20.63, 53.62, 57.47, 20.89, 25.6, 70.31, 16.46, 45.2, 3.72, 28.06,
    27.01
]
y1_b_depth_2 = [
    1.72, 0, 71.5, 53.29, 36.6, 70.04, 43.66, 47.3, 31.46, 20.03, 12.42, 41.79
]
y2_f_depth_2 = [
    44.07, 20.94, 77.6, 64.39, 35.44, 93.66, 72.82, 63.91, 44.27, 26.98, 49.85,
    99.23
]
y2_b_depth_2 = [
    27.37, 18.14, 57.91, 50.25, 103.02, 59.1, 45.35, 37.88, 31.46, 21.64,
    41.65, 58.03
]
y1_f_depth_3 = [
    4.4, 9.49, 35.04, 6.5, 18.29, 60.68, 11.66, 15.98, 45.61, 19.09, 60.9, 9.01
]
y1_b_depth_3 = [
    4.8, 90.57, 103.1, 0, 75.52, 70.06, 56.63, 32.9, 50.85, 76.37, 74.96, 4.75
]
y2_f_depth_3 = [
    74.84, 107.65, 116.04, 76.41, 73.1, 70.9, 45.36, 76.93, 67.96, 62.48,
    75.38, 24.27
]
y2_b_depth_3 = [
    37.03, 39.92, 25.57, 57.49, 66.8, 66.31, 24.4, 60.31, 132.58, 63.82, 66.18,
    26.55
]
y1_f_depth_4 = [
    5.99, 12.94, 15.42, 36.27, 4.94, 37.84, 31.76, 32.22, 28.43, 6.22, 18.01,
    43.41, 22.97, 19.48
]
y1_b_depth_4 = [
    43.24, 9.99, 65.83, 29.07, 0, 13.18, 73.54, 98.06, 38.3, 35.63, 4.57, 7.71,
    2.58, 12.25
]
y2_f_depth_4 = [
    38.63, 40.54, 50.63, 34.86, 24.49, 16.79, 73.14, 60.12, 36.59, 55.84,
    28.69, 57.3, 23.67, 74.06
]
y2_b_depth_4 = [
    46.34, 19.48, 52.37, 82.21, 41.07, 35.27, 54.15, 58.6, 36.13, 19.1, 28.01,
    62.16, 24.12, 28.05
]
# gs = gridspec.GridSpec(2, 6) # 创立2 * 6 网格
# gs.update(wspace=0.8)
x_depth_2 = np.array(range(len(y1_f_depth_2)))
fig, ax = plt.subplots(layout='constrained')
ax.bar(x_depth_2, y1_f_depth_2, 0.4, color='sandybrown', label='$\it{l}^f$ by LSM and PCA')
ax.bar(x_depth_2, y1_b_depth_2, 0.4, bottom=y1_f_depth_2, color='skyblue', label='$\it{l}^b$ by LSM and PCA')
ax.bar(x_depth_2 + 0.4, y2_f_depth_2, 0.4, color='lightpink', label='$\it{l}^f$ by Our Method')
ax.bar(x_depth_2 + 0.4, y2_b_depth_2, 0.4, bottom=y2_f_depth_2, color='darkgrey', label='$\it{l}^b$ by Our Method')
# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Depth (mm)', fontsize=24, fontweight='bold')
ax.set_xlabel('(a) Screws in 2-body fractures', fontsize=24, fontweight='bold')
# ax.set_title('Comparison of the Concrete Implantation Depths', fontsize=24)
plt.xticks(fontsize=24, weight='bold')
plt.yticks(fontsize=24, weight='bold')
plt.ylim((0, 220))
# ax.set_xticks(x + width, species)
ax.legend(loc='upper left', ncols=2, prop={"family": "Times New Roman", "size": 24, 'weight': "bold"})
plt.show()

x_depth_3 = np.array(range(len(y1_f_depth_3)))
fig, ax = plt.subplots(layout='constrained')
ax.bar(x_depth_3, y1_f_depth_3, 0.4, color='sandybrown', label='$\it{l}^f$ by LSM and PCA')
ax.bar(x_depth_3, y1_b_depth_3, 0.4, bottom=y1_f_depth_3, color='skyblue', label='$\it{l}^b$ by LSM and PCA')
ax.bar(x_depth_3 + 0.4, y2_f_depth_3, 0.4, color='lightpink', label='$\it{l}^f$ by Our Method')
ax.bar(x_depth_3 + 0.4, y2_b_depth_3, 0.4, bottom=y2_f_depth_3, color='darkgrey', label='$\it{l}^b$ by Our Method')
# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Depth (mm)', fontsize=24, fontweight='bold')
ax.set_xlabel('(b) Screws in 3-body fractures', fontsize=24, fontweight='bold')
# ax.set_title('Comparison of the Concrete Implantation Depths', fontsize=24)
plt.xticks(fontsize=24, weight='bold')
plt.yticks(fontsize=24, weight='bold')
plt.ylim((0, 220))
# ax.set_xticks(x + width, species)
ax.legend(loc='upper left', ncols=2, prop={"family": "Times New Roman", "size": 24, 'weight': "bold"})
plt.show()

x_depth_4 = np.array(range(len(y1_f_depth_4)))
fig, ax = plt.subplots(layout='constrained')
ax.bar(x_depth_4, y1_f_depth_4, 0.4, color='sandybrown', label='$\it{l}^f$ by LSM and PCA')
ax.bar(x_depth_4, y1_b_depth_4, 0.4, bottom=y1_f_depth_4, color='skyblue', label='$\it{l}^b$ by LSM and PCA')
ax.bar(x_depth_4 + 0.4, y2_f_depth_4, 0.4, color='lightpink', label='$\it{l}^f$ by Our Method')
ax.bar(x_depth_4 + 0.4, y2_b_depth_4, 0.4, bottom=y2_f_depth_4, color='darkgrey', label='$\it{l}^b$ by Our Method')
# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Depth (mm)', fontsize=24, fontweight='bold')
ax.set_xlabel('(c) Screws in 4/5-body fractures', fontsize=24, fontweight='bold')
# ax.set_title('Comparison of Concrete Implantation Depths', fontsize=24)
plt.xticks(fontsize=24, weight='bold')
plt.yticks(fontsize=24, weight='bold')
plt.ylim((0, 220))
# ax.set_xticks(x + width, species)
ax.legend(loc='upper left', ncols=2, prop={"family": "Times New Roman", "size": 24, 'weight': "bold"})
plt.show()