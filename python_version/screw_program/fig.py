import matplotlib.pyplot as plt
import numpy as np


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
# plt.style.use('bmh')
fig, ax = plt.subplots(layout='constrained')

# for attribute, measurement in penguin_means.items():
#     offset = width * multiplier
#     rects = ax.bar(x, measurement, width, alpha=0.6, label=attribute)
#     # ax.bar_label(rects, padding=3)
#     multiplier += 1
# for i in x:
#     if y1[i] < y2[i]:
#         ax.bar(x[i], y1[i], width, alpha=0.6, color='sandybrown', edgecolor='w')
#         ax.bar(x[i], y2[i]-y1[i], width, alpha=1, color='skyblue', bottom=y1[i], edgecolor='w')
#     else:
#         ax.bar(x[i], y1[i]-y2[i], width, alpha=0.6, color='sandybrown', bottom=y2[i], edgecolor='w')
#         ax.bar(x[i], y2[i], width, alpha=1, color='skyblue', edgecolor='w')
# ax.bar(x, y1, width, alpha=0.6, color='sandybrown', label='Center of Mass')
# ax.bar(x, y2, width, alpha=0.6, label='Our method')
plt.plot(x, y1[::-1], 'rs', alpha=0.5, linewidth=0.8, label='Center of Mass')
plt.plot(x, y2[::-1], 'go', alpha=0.5, linewidth=0.8, label='Our method')
ax.fill_between(x, y1[::-1], 0, alpha=.3, linewidth=0, color='sandybrown')
ax.fill_between(x, y2[::-1], y1[::-1], alpha=.3, linewidth=0, color='skyblue')

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Distance (mm)', fontsize=20)
ax.set_xlabel('Screws', fontsize=20)
ax.set_title('Distance Margin(mm) of the Implantation Positions from 2 Methods', fontsize=20)
# ax.set_xticks(x + width, species)
ax.legend(loc='upper right', ncols=1, prop={"family": "Times New Roman", "size": 20})

plt.show()
fig.savefig('./dist.png')
