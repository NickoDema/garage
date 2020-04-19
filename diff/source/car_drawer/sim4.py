import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation


def Gen_RandLine(length, dims=2):
    lineData = np.empty((dims, length))
    lineData[:, 0] = np.random.rand(dims)
    for index in range(1, length):
        step = ((np.random.rand(dims) - 0.5) * 0.1)
        lineData[:, index] = lineData[:, index - 1] + step

    return lineData


def update_lines(num, dataLines, lines):
    for line, data in zip(lines, dataLines):
        # NOTE: there is no .set_data() for 3 dim data...
        line.set_data(data[0:2, :num])
        # line.set_3d_properties(data[2, :num])
    return lines

# Attaching 3D axis to the figure
fig, ax = plt.subplots(1, 1, figsize=[14.0, 14.0], facecolor='w')

# Fifty lines of random 3-D lines
data = [Gen_RandLine(10, 2) for index in range(4)]

print(data)

# Creating fifty line objects.
# NOTE: Can't pass empty arrays into 3d version of plot()
lines = [ax.plot(dat[0, 0:1], dat[1, 0:1])[0] for dat in data]

print(lines)

# Setting the axes properties
ax.set_xlim([0.0, 1.0])
ax.set_xlabel('X')

ax.set_ylim([0.0, 1.0])
ax.set_ylabel('Y')

ax.set_title('3D Test')

# Creating the Animation object
line_ani = animation.FuncAnimation(fig, update_lines, 25, fargs=(data, lines),
                                   interval=50, blit=False)

plt.show()
