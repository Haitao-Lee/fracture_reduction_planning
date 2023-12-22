import numpy as np
import math
import matplotlib.pyplot as plt


x = np.arange(0.001, 10, 0.1)
y = []
for t in x:
    y_1 = 1*math.exp(-0.5*(math.log(t))**2)/(math.sqrt(2*math.pi)*t)
    y.append(y_1*10000)
# plt.plot(x, y, label="my PDF")
# plt.xlabel("x")
# plt.ylabel("y")
# plt.ylim(0, 1)
# plt.legend()
# plt.show()

x_ = np.random.randn(10000)
y_ = np.exp(x_)
plt.hist(y_, bins=round(y_.max()), alpha=0.5, histtype='stepfilled',
         color='steelblue', edgecolor='none', label="validdation")
plt.plot(x, y, label="my PDF")
# plt.xlabel("x")
#plt.xlim(-1, 10)
plt.legend()
plt.show()