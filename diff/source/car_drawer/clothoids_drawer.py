import math as m
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from math import pi, sqrt, cos, sin, copysign, pow, atan2


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Pose:
    def __init__(self, x, y, angle):
        self.point = Point(x, y)
        self.angle = angle


class AngleFinder:
    def __init__(self):
        self.is_first_time = True
        self.true_angle = 0
        self.last_angle = 0

    def getAngle(self, bad_angle):
        if self.is_first_time:
            self.true_angle = bad_angle
            self.is_first_time = False
        else:
            if -pi < bad_angle - self.last_angle < pi:
                self.true_angle += bad_angle - self.last_angle
            elif pi < bad_angle - self.last_angle:
                self.true_angle += bad_angle - self.last_angle - 2 * pi
            else:
                self.true_angle += bad_angle - self.last_angle + 2 * pi

        self.last_angle = bad_angle

        return self.true_angle


########################################################################################################################

def f(x):
    return (1 + 0.926 * x) / (2 + 1.792 * x + 3.104 * x ** 2)


def g(x):
    return 1 / (2 + 4.412 * x + 3.492 * x ** 2 + 6.67 * x ** 3)


def Cf(x):
    return 1 / 2 + f(x) * sin(pi / 2 * x ** 2) - g(x) * cos(pi / 2 * x ** 2)


def Sf(x):
    return 1 / 2 - f(x) * cos(pi / 2 * x ** 2) - g(x) * sin(pi / 2 * x ** 2)


def Cf_by_int(x, step):
    Cf = 0.0
    l = int(x/step)
    s = 0.0
    for i in range(0, l):
        Cf += cos(pi / 2 * s ** 2) * step
        s += step
    return Cf


def Sf_by_int(x, step):
    Sf = 0.0
    l = int(x/step)
    s = 0.0
    for i in range(0, l):
        Sf += sin(pi / 2 * s ** 2) * step
        s += step
    return Sf


def xCoordByS(gamma, alpha, s, step):
    return gamma * sqrt(pi / abs(alpha)) * Cf_by_int(sqrt(abs(alpha) / pi) * s, step)


def yCoordByS(gamma, alpha, s, step):
    return gamma * copysign(1, alpha) * sqrt(pi / abs(alpha)) * Sf_by_int(sqrt(abs(alpha) / pi) * s, step)


def tCoordByS(gamma, alpha, s):
    return 0.5 * alpha * pow(s, 2)


def xCoordByD(d_cc, k_cc, d, step):
    return copysign(1, d_cc) * sqrt(2 * pi * abs(d_cc)) / k_cc * Cf_by_int(sqrt(2 * abs(d) / pi), step)


def yCoordByD(d_cc, k_cc, d, step):
    return sqrt(2 * pi * abs(d_cc)) / k_cc * Sf_by_int(sqrt(2 * abs(d) / pi), step)


def tCoordByD(d):
    return d


def transformCoords(origin_pose=None, old_poses=None, direction=None):
    T = np.array([[cos(origin_pose.angle), -sin(origin_pose.angle), origin_pose.point.x],
                  [sin(origin_pose.angle),  cos(origin_pose.angle), origin_pose.point.y],
                  [0, 0, 1]])
    new_poses = np.ndarray(shape=(old_poses.shape[0], 3))
    if direction == "b":
        T = np.linalg.inv(T)
        new_poses[:, 2] = old_poses[:, 2] - origin_pose.angle
    else:
        new_poses[:, 2] = old_poses[:, 2] + origin_pose.angle

    aux_poses = T.dot(np.vstack([old_poses[:, 0:2].T, (np.ones(shape=(old_poses[:,0].T.shape)))]))
    new_poses[:, 0:2] = aux_poses[0:2, :].T

    return new_poses

# [new_x, new_y] = ( R(to_yaw) * [cur_x, cur_y]^(T) + [to_x, to_y]^(T) )^(T)
def transform(cur_x, cur_y, cur_yaw, to_x, to_y, to_yaw):

    if (to_yaw == 0):
        new_yaw = cur_yaw
        new_xy = np.array([cur_x, cur_y])
    else:
        R = np.array([[np.cos(to_yaw), -np.sin(to_yaw)],
                      [np.sin(to_yaw),  np.cos(to_yaw)]])

        new_xy = R.dot(np.array([cur_x, cur_y]).T).T

        new_yaw = cur_yaw + to_yaw

    return np.array([new_xy[0] + to_x, new_xy[1] + to_y, new_yaw]).T


# [new_x, new_y] = ( R(from_yaw)^(T) * [cur_x, cur_y]^(T) - R(from_yaw)^(T) * [from_x, from_y]^(T) )^(T)
def itransform(cur_x, cur_y, cur_yaw, from_x, from_y, from_yaw):

    Rinv = np.array([[ np.cos(from_yaw), np.sin(from_yaw)],
                     [-np.sin(from_yaw), np.cos(from_yaw)]])

    new_xy = (Rinv.dot(np.array([cur_x, cur_y]).T) - Rinv.dot(np.array([from_x, from_y]).T)).T

    new_yaw = cur_yaw - from_yaw

    return np.array([new_xy[0], new_xy[1], new_yaw]).T


def calcClothoidPointsByS(gamma, alpha, s_end, type, step):
    poses = np.ndarray(shape=(int(10000*s_end) / int(10000*step) + 2, 3))
    if (type == "in"):
        i = 0
        for s in np.arange(0, s_end, step):
            poses[i] = np.array([xCoordByS(gamma, alpha, s, step),
                                 yCoordByS(gamma, alpha, s, step),
                                 tCoordByS(gamma, alpha, s)]).reshape((3))
            i += 1
        poses[i] = np.array([xCoordByS(gamma, alpha, s_end, step),
                             yCoordByS(gamma, alpha, s_end, step),
                             tCoordByS(gamma, alpha, s_end)]).reshape((3))
    else:

        origin_x, origin_y, origin_yaw = xCoordByS(-gamma, alpha, s_end, step), \
                                         yCoordByS(-gamma, alpha, s_end, step), \
                                         tCoordByS(gamma, alpha, s_end)
        i = 0
        for s in np.arange(0, s_end, step):
            poses[i] = np.array([xCoordByS(-gamma, alpha, s_end - s, step),
                                 yCoordByS(-gamma, alpha, s_end - s, step),
                                 tCoordByS(gamma, alpha, s_end - s)]).reshape((1, 3))
            poses[i] = itransform(poses[i,0], poses[i,1], poses[i,2], origin_x, origin_y, origin_yaw)
            i += 1

        poses[i] = np.array([xCoordByS(-gamma, alpha, 0.0, step),
                             yCoordByS(-gamma, alpha, 0.0, step),
                             tCoordByS(gamma, alpha, s_end - s)]).reshape((1, 3))
        poses[i] = itransform(poses[i,0], poses[i,1], poses[i,2], origin_x, origin_y, origin_yaw)

    return poses


def calcClothoidPointsByD(d_cc, turn_radius_cc, type, step):

    k_cc = 1.0 / turn_radius_cc

    traj_size = int(abs((10000*d_cc)) / (10000*step)) + 1
    poses = np.ndarray(shape=(traj_size, 3))

    # step = copysign(abs(step), d_cc)

    if (type == "in"):
        i = 0
        for d in np.arange(0, d_cc, copysign(step, d_cc)):
            poses[i] = np.array([xCoordByD(d_cc, k_cc, d, step),
                                 yCoordByD(d_cc, k_cc, d, step),
                                 tCoordByD(d)]).reshape((3))
            i += 1
        poses[i] = np.array([xCoordByD(d_cc, k_cc, d_cc, step),
                             yCoordByD(d_cc, k_cc, d_cc, step),
                             tCoordByD(d_cc)]).reshape((3))
    else:

        origin_x, origin_y, origin_yaw = xCoordByD(-d_cc, k_cc, d_cc, step), \
                                         yCoordByD(-d_cc, k_cc, d_cc, step), \
                                         tCoordByD(-d_cc)
        i = 0
        for d in np.arange(0, d_cc, copysign(step, d_cc)):
            poses[i] = np.array([xCoordByD(-d_cc, k_cc, d-d_cc, step),
                                 yCoordByD(-d_cc, k_cc, d-d_cc, step),
                                 tCoordByD(d-d_cc)]).reshape((1, 3))
            poses[i] = itransform(poses[i,0], poses[i,1], poses[i,2], origin_x, origin_y, origin_yaw)
            i += 1

        poses[i] = np.array([xCoordByD(-d_cc, k_cc, 0.0, step),
                             yCoordByD(-d_cc, k_cc, 0.0, step),
                             tCoordByD(0.0)]).reshape((1, 3))
        poses[i] = itransform(poses[i,0], poses[i,1], poses[i,2], origin_x, origin_y, origin_yaw)

    return poses


def calcClothoidPointsByDwithS(d_cc, turn_radius_cc, type, step):
    k_cc = 1.0 / turn_radius_cc
    alpha = (k_cc ** 2) / (2 * d_cc)
    s_end = abs(k_cc / alpha)
    gamma = copysign(1, k_cc)*copysign(1,d_cc)

    if type == "out":
        alpha *= -1

    # print(str(round(k_cc, 2))  + "\t" + \
    #       str(round(d_cc, 2))  + "\t" + \
    #       str(round(gamma, 2)) + "\t" + \
    #       str(round(alpha, 2)) + "\t" + \
    #       str(round(s_end, 2)))

    return calcClothoidPointsByS(gamma, alpha, s_end, type, step)


class Plotter():

    def __init__(self, xlim=[-22,22], ylim=[-9,9]):
        self.fig, self.ax = plt.subplots(1, 1, figsize=[10.0, 10.0], facecolor='w')
        self.ax.set_aspect('equal')
        # self.ax.set_xlim(xlim[0], xlim[1])
        # self.ax.set_ylim(ylim[0], ylim[1])
        self.ax.grid(True)
        self.ax.set_xlabel('X, [m]')
        self.ax.set_ylabel('Y, [m]')


    def draw_clothoid1(self, points, color="#A00000", lw=2):

        plots = self.ax.plot(points[:,0], points[:,1], lw=lw, color=color)

        return plots


    def draw_traj_frames_from_history(self, x, y, yaw, dt, x_col="#DFA3A3", x_len=0.7, y_col="#C5EED2", y_len=0.5, z_col="#BFDBEE", lw=4):

        frames_plots = ()

        frames_plots += tuple(self.draw_frame(x[0], y[0], yaw[0], x_col=x_col,
                                                                  y_col=y_col,
                                                                  z_col=z_col,
                                                                  lw=lw))

        t = 0.0
        for i in range(1, len(x)):
            t += (0.01)
            if (dt <= t):
                t = 0.0
                frames_plots += tuple(self.draw_frame(x[i], y[i], yaw[i], x_col=x_col,
                                                                          y_col=y_col,
                                                                          z_col=z_col,
                                                                          lw=lw))

        frames_plots += tuple(self.draw_frame(x[-1], y[-1], yaw[-1], x_col=x_col,
                                                                     y_col=y_col,
                                                                     z_col=z_col,
                                                                     lw=lw))

        return frames_plots


    def draw_frame(self, x=0.0, y=0.0, yaw=0.0, x_col="#C00000", x_len=0.7, y_col="#008000", y_len=0.5, z_col="#4682B4", lw=4):

        R = np.array([[np.cos(yaw), -np.sin(yaw)],
                      [np.sin(yaw),  np.cos(yaw)]])

        frame_x = R.dot(np.array([[0.0, 0.0], [x_len, 0.0]]).T).T
        frame_y = R.dot(np.array([[0.0, 0.0], [0.0, y_len]]).T).T

        x_plot, = self.ax.plot(frame_x[:,0] + x, frame_x[:,1] + y, lw=lw, color=x_col)
        y_plot, = self.ax.plot(frame_y[:,0] + x, frame_y[:,1] + y, lw=lw, color=y_col)
        z_plot, = self.ax.plot(x, y, marker='o', markersize=lw, color=z_col)

        return x_plot, y_plot, z_plot


# def update_plot(frame, plotter, clothoid=None):
#
#     plotting_tuple = ()
#
#     if(clothoid):
#         plotting_tuple += clothoid
#
#     return plotting_tuple


def main():
    # with plt.xkcd():

    plotter = Plotter([-4,50], [-4,20])

    # points1 = calcClothoidPointsByS(gamma= 1, alpha= 0.02,  s_end=10, type="in",  step=0.01) #green
    # points2 = calcClothoidPointsByS(gamma=-1, alpha= 0.02,  s_end=10, type="in",  step=0.01) #red
    # points3 = calcClothoidPointsByS(gamma=-1, alpha=-0.02,  s_end=10, type="in",  step=0.01) #blue
    # points4 = calcClothoidPointsByS(gamma= 1, alpha=-0.02,  s_end=10, type="in",  step=0.01) #yellow
    #
    # points5 = calcClothoidPointsByS(gamma= 1, alpha=-0.02,  s_end=10, type="out",  step=0.01) #green
    # points6 = calcClothoidPointsByS(gamma=-1, alpha=-0.02,  s_end=10, type="out",  step=0.01) #red
    # points7 = calcClothoidPointsByS(gamma=-1, alpha= 0.02,  s_end=10, type="out",  step=0.01) #blue
    # points8 = calcClothoidPointsByS(gamma= 1, alpha= 0.02,  s_end=10, type="out",  step=0.01) #yellow
    #
    # for i in range(0, len(points1)):
    #     points1[i] = transform(points1[i,0], points1[i,1], points1[i,2], 0.0, 0.0, 0.0)
    #     points2[i] = transform(points2[i,0], points2[i,1], points2[i,2], 0.0, 0.0, 0.0)
    #     points3[i] = transform(points3[i,0], points3[i,1], points3[i,2], 0.0, 0.0, 0.0)
    #     points4[i] = transform(points4[i,0], points4[i,1], points4[i,2], 0.0, 0.0, 0.0)

    # points1 = calcClothoidPointsByS(gamma=1,  alpha=0.2,  s_end=5, type="in",  step=0.01) #green
    # for i in range(0, len(points1)):
    #     points1[i] = transform(points1[i,0], points1[i,1], points1[i,2], 1.0, 0.5, 0.5)
    #
    # points2 = calcClothoidPointsByS(gamma=1,  alpha=-0.07,  s_end=7, type="out",  step=0.01) #red
    # for i in range(0, len(points2)):
    #     points2[i] = transform(points2[i,0], points2[i,1], points2[i,2], points1[-1,0], points1[-1,1], points1[-1,2],)
    #
    # points3 = calcClothoidPointsByS(gamma=1,  alpha=-0.3,  s_end=5, type="in",  step=0.01) #green
    # for i in range(0, len(points3)):
    #     points3[i] = transform(points3[i,0], points3[i,1], points3[i,2], points2[-1,0], points2[-1,1], points2[-1,2])
    #
    # points4 = calcClothoidPointsByS(gamma=1,  alpha=-0.04,  s_end=10, type="out",  step=0.01) #red
    # for i in range(0, len(points4)):
    #     points4[i] = transform(points4[i,0], points4[i,1], points4[i,2], points3[-1,0], points3[-1,1], points3[-1,2],)

    # points1 = calcClothoidPointsByD(d_cc =  1.0, turn_radius_cc= 5.0, type="in",  step=0.01) #green
    # points2 = calcClothoidPointsByD(d_cc =  1.0, turn_radius_cc=-5.0, type="in",  step=0.01) #red
    # points3 = calcClothoidPointsByD(d_cc = -1.0, turn_radius_cc= 5.0, type="in",  step=0.01) #blue
    # points4 = calcClothoidPointsByD(d_cc = -1.0, turn_radius_cc=-5.0, type="in",  step=0.01) #yellow
    #
    # points5 = calcClothoidPointsByD(d_cc =  1.0, turn_radius_cc= 5.0, type="out", step=0.01) #green
    # points6 = calcClothoidPointsByD(d_cc =  1.0, turn_radius_cc=-5.0, type="out", step=0.01) #red
    # points7 = calcClothoidPointsByD(d_cc = -1.0, turn_radius_cc= 5.0, type="out", step=0.01) #blue
    # points8 = calcClothoidPointsByD(d_cc = -1.0, turn_radius_cc=-5.0, type="out", step=0.01) #yellow

    # points1 = calcClothoidPointsByDwithS(d_cc =  1.0, turn_radius_cc= 5.0, type="in",  step=0.01) #green
    # points2 = calcClothoidPointsByDwithS(d_cc =  1.0, turn_radius_cc=-5.0, type="in",  step=0.01) #red
    # points3 = calcClothoidPointsByDwithS(d_cc = -1.0, turn_radius_cc= 5.0, type="in",  step=0.01) #blue
    # points4 = calcClothoidPointsByDwithS(d_cc = -1.0, turn_radius_cc=-5.0, type="in",  step=0.01) #yellow
    # #
    # points5 = calcClothoidPointsByDwithS(d_cc =  1.0, turn_radius_cc= 5.0, type="out", step=0.01) #green
    # points6 = calcClothoidPointsByDwithS(d_cc =  1.0, turn_radius_cc=-5.0, type="out", step=0.01) #red
    # points7 = calcClothoidPointsByDwithS(d_cc = -1.0, turn_radius_cc= 5.0, type="out", step=0.01) #blue
    # points8 = calcClothoidPointsByDwithS(d_cc = -1.0, turn_radius_cc=-5.0, type="out", step=0.01) #yellow

    # points1 = calcClothoidPointsByDwithS(d_cc =  1.0, turn_radius_cc= 1.0, type="in",  step=0.01) #green
    # points2 = calcClothoidPointsByDwithS(d_cc =  1.0, turn_radius_cc= 2.0, type="in",  step=0.01) #red
    # points3 = calcClothoidPointsByDwithS(d_cc =  1.0, turn_radius_cc= 3.0, type="in",  step=0.01) #blue
    # points4 = calcClothoidPointsByDwithS(d_cc =  1.0, turn_radius_cc= 4.0, type="in",  step=0.01) #yellow
    #
    # points5 = calcClothoidPointsByDwithS(d_cc =  1.0, turn_radius_cc= 5.0, type="in", step=0.01) #green
    # points6 = calcClothoidPointsByDwithS(d_cc =  1.0, turn_radius_cc= 6.0, type="in", step=0.01) #red
    # points7 = calcClothoidPointsByDwithS(d_cc =  1.0, turn_radius_cc= 7.0, type="in", step=0.01) #blue
    # points8 = calcClothoidPointsByDwithS(d_cc =  1.0, turn_radius_cc= 8.0, type="in", step=0.01) #yellow


    # points1 = calcClothoidPointsByDwithS(d_cc=m.pi/2, turn_radius_cc=5.0, type="in",  step=0.01) #green
    # # for i in range(0, len(points1)):
    # #     points1[i] = transform(points1[i,0], points1[i,1], points1[i,2], 1.0, 0.5, 0.5)
    #
    # points2 = calcClothoidPointsByDwithS(d_cc=m.pi/2, turn_radius_cc=5.0, type="out",  step=0.01) #red
    # for i in range(0, len(points2)):
    #     points2[i] = transform(points2[i,0], points2[i,1], points2[i,2], points1[-1,0], points1[-1,1], points1[-1,2],)
    #
    # points3 = calcClothoidPointsByDwithS(d_cc=m.pi/2, turn_radius_cc=5.0, type="in",  step=0.01) #blue
    # for i in range(0, len(points3)):
    #     points3[i] = transform(points3[i,0], points3[i,1], points3[i,2], points2[-1,0], points2[-1,1], points2[-1,2])
    #
    # points4 = calcClothoidPointsByDwithS(d_cc=m.pi/2, turn_radius_cc=5.0, type="out",  step=0.01) #yellow
    # for i in range(0, len(points4)):
    #     points4[i] = transform(points4[i,0], points4[i,1], points4[i,2], points3[-1,0], points3[-1,1], points3[-1,2],)

    turns_for_8 = 5.0

    points1 = calcClothoidPointsByDwithS(d_cc=2.27884, turn_radius_cc=turns_for_8, type="in",  step=0.01) #green
    for i in range(0, len(points1)):
        points1[i] = transform(points1[i,0], points1[i,1], points1[i,2], 0.0, 0.0, -(2.27884-m.pi/2))

    points2 = calcClothoidPointsByDwithS(d_cc=2.27884, turn_radius_cc=turns_for_8, type="out",  step=0.01) #red
    for i in range(0, len(points2)):
        points2[i] = transform(points2[i,0], points2[i,1], points2[i,2], points1[-1,0], points1[-1,1], points1[-1,2],)

    points3 = calcClothoidPointsByDwithS(d_cc=-2.27884, turn_radius_cc=-turns_for_8, type="in",  step=0.01) #blue
    for i in range(0, len(points3)):
        points3[i] = transform(points3[i,0], points3[i,1], points3[i,2], points2[-1,0], points2[-1,1], points2[-1,2])

    points4 = calcClothoidPointsByDwithS(d_cc=-2.27884, turn_radius_cc=-turns_for_8, type="out",  step=0.01) #yellow
    for i in range(0, len(points4)):
        points4[i] = transform(points4[i,0], points4[i,1], points4[i,2], points3[-1,0], points3[-1,1], points3[-1,2],)


    plotter.draw_traj_frames_from_history(points1[:,0], points1[:,1], points1[:,2], dt=2.0,
                                          x_col="#C00000", x_len=0.7, y_col="#008000", y_len=0.5, z_col="#4682B4", lw=4)
    plotter.draw_traj_frames_from_history(points2[:,0], points2[:,1], points2[:,2], dt=2.0,
                                          x_col="#C00000", x_len=0.7, y_col="#008000", y_len=0.5, z_col="#4682B4", lw=4)


    plotter.draw_traj_frames_from_history(points3[:,0], points3[:,1], points3[:,2], dt=2.0,
                                          x_col="#C00000", x_len=0.7, y_col="#008000", y_len=0.5, z_col="#4682B4", lw=4)
    plotter.draw_traj_frames_from_history(points4[:,0], points4[:,1], points4[:,2], dt=2.0,
                                          x_col="#C00000", x_len=0.7, y_col="#008000", y_len=0.5, z_col="#4682B4", lw=4)

    clothoid1_plot = plotter.draw_clothoid1(points1, color="#00A000", lw=3)
    clothoid2_plot = plotter.draw_clothoid1(points2, lw=3)
    clothoid3_plot = plotter.draw_clothoid1(points3, color="#0000A0", lw=3)
    clothoid4_plot = plotter.draw_clothoid1(points4, color="#A0A000", lw=3)
    #
    # plotter.draw_traj_frames_from_history(points5[:,0], points5[:,1], points5[:,2], dt=1.0)
    # plotter.draw_traj_frames_from_history(points6[:,0], points6[:,1], points6[:,2], dt=1.0)
    # plotter.draw_traj_frames_from_history(points7[:,0], points7[:,1], points7[:,2], dt=1.0)
    # plotter.draw_traj_frames_from_history(points8[:,0], points8[:,1], points8[:,2], dt=1.0)
    #
    # clothoid5_plot = plotter.draw_clothoid1(points5, color="#00A000")
    # clothoid6_plot = plotter.draw_clothoid1(points6)
    # clothoid7_plot = plotter.draw_clothoid1(points7, color="#0000A0")
    # clothoid8_plot = plotter.draw_clothoid1(points8, color="#A0A000")

    # animation = FuncAnimation(plotter.fig,
    #                           update_plot,
    #                           fargs=(plotter, clothoid),
    #                           interval=40,          # if too large and blit is True
    #                           blit=True)            # throw an exeption but works somehow

    plt.show()


if __name__ == "__main__":
    main()
