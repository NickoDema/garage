import numpy as np
import matplotlib.pyplot as plt
import math

U_A_MAX = 1.0
U_OMEGA_MAX = math.radians(45.0)
PHI_V = 0.01
PHI_OMEGA = 0.01
WB = 0.25  # [m] wheel base

show_animation = True


def differential_model(v, yaw, u_1, u_2):

    dx = math.cos(yaw) * v
    dy = math.sin(yaw) * v
    dv = u_1
    dyaw = v / WB * math.sin(u_2)  # tan is not good for nonlinear optimization

    return dx, dy, dyaw, dv


class TwoWheeledSystem():

    def __init__(self, init_x, init_y, init_yaw, init_v, init_a = 0.0, init_steer = 0.0):

        self.x = init_x
        self.y = init_y
        self.yaw = init_yaw
        self.v = init_v
        self.a = init_a
        self.steer = init_steer
        self.history_x = [init_x]
        self.history_y = [init_y]
        self.history_yaw = [init_yaw]
        self.history_v = [init_v]

    def update_state(self, u_1, u_2, dt=0.01):

        dx, dy, dyaw, dv = differential_model(self.v, self.yaw, u_1, u_2)

        self.x += dt * dx
        self.y += dt * dy
        self.yaw += dt * dyaw
        self.v += dt * dv
        self.a = dv

        # save
        self.history_x.append(self.x)
        self.history_y.append(self.y)
        self.history_yaw.append(self.yaw)
        self.history_v.append(self.v)


def plot_figures(plant_system, controller, iteration_num, dt):  # pragma: no cover
    # figure
    # time history
    fig_p = plt.figure()
    fig_u = plt.figure()
    fig_f = plt.figure()

    # traj
    fig_t = plt.figure()
    fig_traj = fig_t.add_subplot(111)
    fig_traj.set_aspect('equal')

    x_1_fig = fig_p.add_subplot(411)
    x_2_fig = fig_p.add_subplot(412)
    x_3_fig = fig_p.add_subplot(413)
    x_4_fig = fig_p.add_subplot(414)

    u_1_fig = fig_u.add_subplot(411)
    u_2_fig = fig_u.add_subplot(412)
    dummy_1_fig = fig_u.add_subplot(413)
    dummy_2_fig = fig_u.add_subplot(414)

    raw_1_fig = fig_f.add_subplot(311)
    raw_2_fig = fig_f.add_subplot(312)
    f_fig = fig_f.add_subplot(313)

    x_1_fig.plot(np.arange(iteration_num) * dt, plant_system.history_x)
    x_1_fig.set_xlabel("time [s]")
    x_1_fig.set_ylabel("x")

    x_2_fig.plot(np.arange(iteration_num) * dt, plant_system.history_y)
    x_2_fig.set_xlabel("time [s]")
    x_2_fig.set_ylabel("y")

    x_3_fig.plot(np.arange(iteration_num) * dt, plant_system.history_yaw)
    x_3_fig.set_xlabel("time [s]")
    x_3_fig.set_ylabel("yaw")

    x_4_fig.plot(np.arange(iteration_num) * dt, plant_system.history_v)
    x_4_fig.set_xlabel("time [s]")
    x_4_fig.set_ylabel("v")

    u_1_fig.plot(np.arange(iteration_num - 1) * dt, controller.history_u_1)
    u_1_fig.set_xlabel("time [s]")
    u_1_fig.set_ylabel("u_a")

    u_2_fig.plot(np.arange(iteration_num - 1) * dt, controller.history_u_2)
    u_2_fig.set_xlabel("time [s]")
    u_2_fig.set_ylabel("u_omega")

    dummy_1_fig.plot(np.arange(iteration_num - 1) *
                     dt, controller.history_dummy_u_1)
    dummy_1_fig.set_xlabel("time [s]")
    dummy_1_fig.set_ylabel("dummy u_1")

    dummy_2_fig.plot(np.arange(iteration_num - 1) *
                     dt, controller.history_dummy_u_2)
    dummy_2_fig.set_xlabel("time [s]")
    dummy_2_fig.set_ylabel("dummy u_2")

    raw_1_fig.plot(np.arange(iteration_num - 1) * dt, controller.history_raw_1)
    raw_1_fig.set_xlabel("time [s]")
    raw_1_fig.set_ylabel("raw_1")

    raw_2_fig.plot(np.arange(iteration_num - 1) * dt, controller.history_raw_2)
    raw_2_fig.set_xlabel("time [s]")
    raw_2_fig.set_ylabel("raw_2")

    f_fig.plot(np.arange(iteration_num - 1) * dt, controller.history_f)
    f_fig.set_xlabel("time [s]")
    f_fig.set_ylabel("optimal error")

    fig_traj.plot(plant_system.history_x,
                  plant_system.history_y, "-r")
    fig_traj.set_xlabel("x [m]")
    fig_traj.set_ylabel("y [m]")
    fig_traj.axis("equal")

    # start state
    plot_car(plant_system.history_x[0],
             plant_system.history_y[0],
             plant_system.history_yaw[0],
             controller.history_u_2[0],
             )

    # goal state
    plot_car(0.0, 0.0, 0.0, 0.0)

    plt.show()


def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

    # Vehicle parameters
    LENGTH = 0.4  # [m]
    WIDTH = 0.2  # [m]
    BACKTOWHEEL = 0.1  # [m]
    WHEEL_LEN = 0.03  # [m]
    WHEEL_WIDTH = 0.02  # [m]
    TREAD = 0.07  # [m]

    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL),
                         -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH -
                          TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")


def animation(plant, controller, dt):

    skip = 2  # skip index for animation

    for t in range(1, len(plant.history_x)): #, skip):
        x = plant.history_x[t]
        y = plant.history_y[t]
        yaw = plant.history_yaw[t]
        v = plant.history_v[t]
        accel = plant.a #history_u_1[t]
        time = t * dt
        steer = controller.steer

        # if abs(v) <= 0.01:
        #     steer = 0.0
        # else:
        #     steer = math.atan2(controller.history_u_2[t] * WB / v, 1.0)

        plt.cla()
        plt.plot(plant.history_x, plant.history_y, "-r", label="trajectory")
        plot_car(x, y, yaw, steer=steer)
        plt.axis("equal")
        plt.grid(True)
        plt.title("Time[s]:" + str(round(time, 2))+
                  ", accel[m/s]:" + str(round(accel, 2)) +
                  ", speed[km/h]:" + str(round(v * 3.6, 2)))
        plt.pause(0.001)

    plt.close("all")

class MyController():
    def __init__(self):
        self.a      = 3.0
        self.steer  = 0.7
        self.a_time = 2

    def calc_input(self, x, y, yaw, v, time):
        if (time > self.a_time):
            return 0.0, self.steer
        else:
            return self.a, self.steer


def main():
    # simulation time
    dt = 0.03
    iteration_time = 20.0  # [s]

    init_x = -4.5
    init_y = -2.5
    init_yaw = math.radians(45.0)
    init_v = 0.0 #-1.0

    # plant
    plant_system = TwoWheeledSystem(
        init_x, init_y, init_yaw, init_v)

    # controller
    controller = MyController() # NMPCController_with_CGMRES()

    iteration_num = int(iteration_time / dt)
    for i in range(1, iteration_num):
        time = float(i) * dt
        # make input
        u_1s, u_2s = controller.calc_input(
            plant_system.x, plant_system.y, plant_system.yaw, plant_system.v, time)
        # update state
        plant_system.update_state(u_1s, u_2s) #(u_1s[0], u_2s[0])

    if show_animation:  # pragma: no cover
        animation(plant_system, controller, dt)
        #plot_figures(plant_system, controller, iteration_num, dt)


if __name__ == "__main__":
    main()
