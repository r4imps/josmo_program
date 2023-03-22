import matplotlib.pyplot as plt
import csv
from odometry import ODOMEETRIA

obj=ODOMEETRIA()
val=obj.ODOMETRY_FUNC()
with open('SAVE.csv', 'r') as file:
    reader = csv.reader(file)
    print(reader)
plt.plot([val[6]], [val[7]], 'ro')
plt.axis([0,200, 0, 200])
plt.show()