import math as m
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class Car():
    def __init__(self):
        self.wheel_radius = 0.37
        self.wheel_width  = 0.25
        self.wheelbase    = 2.75
        self.axle_track   = 1.64
        self.footprint = np.array([[ 3.77,  0.94],        # quadrants order
                                   [ 3.77, -0.94],
                                   [-1.0,  -0.94],
                                   [-1.0,   0.94]])

        self.wheel_footprint = np.array([[ self.wheel_radius,  self.wheel_width/2],
                                         [ self.wheel_radius, -self.wheel_width/2],
                                         [-self.wheel_radius, -self.wheel_width/2],
                                         [-self.wheel_radius,  self.wheel_width/2],
                                         [ self.wheel_radius,  self.wheel_width/2]])

        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self.v   = 0.0
        self.a   = 0.0

        self.steer = 0.0        # max 0.576 rad ~ 33.78 deg
        self.max_steer = 0.576
        self.min_radius = self.wheelbase / np.tan(self.max_steer)
        self.steer_fr = 0.0
        self.steer_fl = 0.0
        self.turn_radius = 10000.0

        self.history_t = []
        self.history_x = []
        self.history_y = []
        self.history_yaw = []
        self.history_steer = []
        self.history_v = []


    def set_steering(self, steering):

        if (abs(steering) > self.max_steer):
            self.steer = m.copysign(self.max_steer, steering)
            # warn!
        else:
            self.steer = steering

        if (abs(steering) < 0.001):         # ~0.056 deg
            self.turn_radius = m.copysign(10000.0, self.steer)
        else:
            self.turn_radius = self.wheelbase / np.tan(self.steer)

        self.steer_fl, self.steer_fr = self.calc_front_wheels_steerings_by_turn_radius(self.turn_radius)


    def calc_turn_radius_by_steering(self, steering):

        if (abs(steering) > self.max_steer):
            steering = m.copysign(self.max_steer, steering)
            # warn!

        if (abs(steering) < 0.001):         # ~0.056 deg
            turn_radius = m.copysign(10000.0, steering)
        else:
            turn_radius = self.wheelbase / np.tan(steering)

        return turn_radius


    def calc_front_wheels_steerings_by_turn_radius(self, turn_radius):
        return np.arctan2(self.wheelbase, turn_radius - self.axle_track / 2.0), \
               np.arctan2(self.wheelbase, turn_radius + self.axle_track / 2.0)


    def calc_front_wheels_steerings_by_steering(self, steering):
        if (abs(steering) < 0.0001):
            return 0.0, 0.0
        else:
            return np.arctan2(self.wheelbase, self.wheelbase / np.tan(steering) - self.axle_track / 2.0), \
                   np.arctan2(self.wheelbase, self.wheelbase / np.tan(steering) + self.axle_track / 2.0)


    def calc_steering_by_turn_radius(self, turn_radius):

        if (abs(turn_radius) < self.min_radius):
            turn_radius = m.copysign(self.min_radius, turn_radius)
            # warn!

        steering = np.arctan2(self.wheelbase, turn_radius)

        if (turn_radius < 0):
            steering -= np.pi

        return steering


    def calc_traj_step_by_control(self, u_1, u_2, dt=0.01):

        if (abs(u_2) > self.max_steer):
            u_2 = m.copysign(self.max_steer, u_2)

        dx = m.cos(self.yaw) * self.v
        dy = m.sin(self.yaw) * self.v
        dv = u_1
        dyaw = self.v / self.wheelbase * np.tan(u_2)

        self.x += dt * dx
        self.y += dt * dy
        self.yaw += dt * dyaw
        self.v += dt * dv
        self.a = dv
        self.set_steering(u_2)

        self.history_x.append(self.x)
        self.history_y.append(self.y)
        self.history_yaw.append(self.yaw)
        self.history_steer.append(self.steer)
        self.history_v.append(self.v)

        if self.history_t:
            self.history_t.append(self.history_t[-1] + dt)
        else:
            self.history_t.append(0.0)


    def generate_8_traj_by_control(self, dt = 0.01):

        self.yaw = np.pi/2

        #     for t in range(1, int(5.0/dt)):
        #         self.calc_traj_step_by_control(0.0, self.calc_steering_by_turn_radius(-6.0), dt)

        while (self.y < 5.0):
            if (self.v < 16):
                self.calc_traj_step_by_control(16.0, 0.0, dt)
            else:
                self.calc_traj_step_by_control(0.0, 0.0, dt)

        while (self.y > 5.0 or self.x < -5.0):
            self.calc_traj_step_by_control(0.0, self.calc_steering_by_turn_radius(5.0), dt)

        while (self.x < 5.0):
            self.calc_traj_step_by_control(0.0, 0.0, dt)

        while (self.y < -5.0 or self.x > 5.0):
            self.calc_traj_step_by_control(0.0, self.calc_steering_by_turn_radius(-5.0), dt)

        while (self.v > 0):
            self.calc_traj_step_by_control(-16.0, 0.0, dt)


class Plotter():

    def __init__(self, xlim=[-12,12], ylim=[-12,12]):
        self.fig, self.ax = plt.subplots(1, 1, figsize=[10.0, 10.0], facecolor='w')
        self.ax.set_aspect('equal')
        self.ax.set_xlim(xlim[0], xlim[1])
        self.ax.set_ylim(ylim[0], ylim[1])
        self.ax.grid(True)
        self.ax.set_xlabel('X, [m]')
        self.ax.set_ylabel('Y, [m]')


    def draw_traj_from_history(self, car, dt = 1.0, frames = False, frames_dt = 1.0, footprints = False, footprints_dt = 1.0):

        trajectory_x = [car.history_x[0]]
        trajectory_y = [car.history_y[0]]
        t = 0.0
        for i in range(1, len(car.history_t)):
            t += (car.history_t[i] - car.history_t[i-1])
            if (dt <= t):
                t = 0.0
                trajectory_x.append(car.history_x[i])
                trajectory_y.append(car.history_y[i])

        trajectory_x.append(car.history_x[-1])
        trajectory_y.append(car.history_y[-1])

        plots = tuple(self.ax.plot(trajectory_x, trajectory_y, lw=1, color="#A00000"))

        if(frames):
            plots += self.draw_traj_frames_from_history(car, frames_dt)

        if(footprints):
            plots += self.draw_traj_footprints_from_history(car, footprints_dt)

        return plots


    def draw_traj_footprints_from_history(self, car, dt):
        #F0F8FF
        footprints_plots = ()

        footprints_plots += tuple(self.draw_car(car, car.history_x[0],
                                                     car.history_y[0],
                                                     car.history_yaw[0],
                                                     car.history_steer[0],
                                                     color="#FFC6D3", lw=1))

        t = 0.0
        for i in range(1, len(car.history_t)):
            t += (car.history_t[i] - car.history_t[i-1])
            if (dt <= t):
                t = 0.0
                footprints_plots += tuple(self.draw_car(car, car.history_x[i],
                                                             car.history_y[i],
                                                             car.history_yaw[i],
                                                             car.history_steer[i],
                                                             color="#FFC6D3", lw=1))

        footprints_plots += tuple(self.draw_car(car, car.history_x[-1],
                                                     car.history_y[-1],
                                                     car.history_yaw[-1],
                                                     car.history_steer[-1],
                                                     color="#FFC6D3", lw=1))

        return footprints_plots


    def draw_traj_frames_from_history(self, car, dt):

        frames_plots = ()

        frames_plots += tuple(self.draw_frame(car.history_x[0],
                                              car.history_y[0],
                                              car.history_yaw[0],
                                              x_col="#DFA3A3",
                                              y_col="#C5EED2",
                                              z_col="#BFDBEE"))

        t = 0.0
        for i in range(1, len(car.history_t)):
            t += (car.history_t[i] - car.history_t[i-1])
            if (dt <= t):
                t = 0.0
                frames_plots += tuple(self.draw_frame(car.history_x[i],
                                                      car.history_y[i],
                                                      car.history_yaw[i],
                                                      x_col="#DFA3A3",
                                                      y_col="#C5EED2",
                                                      z_col="#BFDBEE"))

        frames_plots += tuple(self.draw_frame(car.history_x[-1],
                                              car.history_y[-1],
                                              car.history_yaw[-1],
                                              x_col="#DFA3A3",
                                              y_col="#C5EED2",
                                              z_col="#BFDBEE"))

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


    def draw_additional_car_geometry(self, car, x=0.0, y=0.0, yaw=0.0, steering=0.0):

        plots = ()

        turn_radius = car.calc_turn_radius_by_steering(steering)

        if (abs(turn_radius) < 20):

            # car Rot matrix in global frame
            R_car = np.array([[np.cos(yaw), -np.sin(yaw)],
                              [np.sin(yaw),  np.cos(yaw)]])

            wheels_to_turn_radius = np.array([[car.wheelbase, -car.axle_track/2.0],
                                              [0, turn_radius],
                                              [car.wheelbase,  car.axle_track/2.0]])

            wheels_to_turn_radius = R_car.dot(wheels_to_turn_radius.T).T

            frame_to_radius = np.array([[0, 0], [0, turn_radius]])
            frame_to_radius = R_car.dot(frame_to_radius.T).T

            plots += tuple(self.ax.plot(wheels_to_turn_radius[:,0] + x,
                                        wheels_to_turn_radius[:,1] + y,
                                        lw=1, ls=":", color="#A0A0A0"))

            plots += tuple(self.ax.plot(frame_to_radius[:,0] + x,
                                        frame_to_radius[:,1] + y,
                                        lw=1, ls=":", color="#A0A0A0"))

        return plots


    def draw_car(self, car, x=0.0, y=0.0, yaw=0.0, steering=0.0, color="#0000CD", lw=2):

        # car Rot matrix in global frame
        R_car = np.array([[np.cos(yaw), -np.sin(yaw)],
                          [np.sin(yaw),  np.cos(yaw)]])

        car_front_footprint = np.array([[car.wheelbase + 1.3*car.wheel_radius, car.footprint[0][1] - 0.25*car.wheel_width],
                                        [car.wheelbase + 1.3*car.wheel_radius, car.footprint[0][1]],
                                        car.footprint[0],
                                        car.footprint[1],
                                        [car.wheelbase + 1.3*car.wheel_radius, car.footprint[1][1]],
                                        [car.wheelbase + 1.3*car.wheel_radius, car.footprint[1][1] + 0.25*car.wheel_width]])

        car_front_footprint = R_car.dot(car_front_footprint.T).T
        car_front_footprint[:,0] += x
        car_front_footprint[:,1] += y

        car_left_side_footprint = np.array([[car.wheelbase - 1.3*car.wheel_radius, car.footprint[0][1] - 0.25*car.wheel_width],
                                            [car.wheelbase - 1.3*car.wheel_radius, car.footprint[0][1]],
                                            [1.3*car.wheel_radius, car.footprint[0][1]],
                                            [1.3*car.wheel_radius, car.footprint[0][1] - 0.25*car.wheel_width]])

        car_left_side_footprint = R_car.dot(car_left_side_footprint.T).T
        car_left_side_footprint[:,0] += x
        car_left_side_footprint[:,1] += y

        car_right_side_footprint = np.array([[car.wheelbase - 1.3*car.wheel_radius, car.footprint[1][1] + 0.25*car.wheel_width],
                                             [car.wheelbase - 1.3*car.wheel_radius, car.footprint[1][1]],
                                             [1.3*car.wheel_radius, car.footprint[1][1]],
                                             [1.3*car.wheel_radius, car.footprint[1][1] + 0.25*car.wheel_width]])

        car_right_side_footprint = R_car.dot(car_right_side_footprint.T).T
        car_right_side_footprint[:,0] += x
        car_right_side_footprint[:,1] += y

        car_rear_footprint = np.array([[- 1.3*car.wheel_radius, car.footprint[2][1] + 0.25*car.wheel_width],
                                       [- 1.3*car.wheel_radius, car.footprint[2][1]],
                                       car.footprint[2],
                                       car.footprint[3],
                                       [- 1.3*car.wheel_radius, car.footprint[0][1]],
                                       [- 1.3*car.wheel_radius, car.footprint[0][1] - 0.25*car.wheel_width]])

        car_rear_footprint = R_car.dot(car_rear_footprint.T).T
        car_rear_footprint[:,0] += x
        car_rear_footprint[:,1] += y

        rr_wheel_footprint = np.array(car.wheel_footprint)
        rr_wheel_footprint [:,1] -= car.axle_track / 2.0
        rr_wheel_footprint = R_car.dot(rr_wheel_footprint.T).T
        rr_wheel_footprint[:,0] += x
        rr_wheel_footprint[:,1] += y

        rl_wheel_footprint = np.array(car.wheel_footprint)
        rl_wheel_footprint [:,1] += car.axle_track / 2.0
        rl_wheel_footprint = R_car.dot(rl_wheel_footprint.T).T
        rl_wheel_footprint[:,0] += x
        rl_wheel_footprint[:,1] += y

        steer_fl, steer_fr = car.calc_front_wheels_steerings_by_steering(steering)

        # wheels Rot matrices in car frame
        R_steer_fl = np.array([[np.cos(steer_fl), -np.sin(steer_fl)],
                               [np.sin(steer_fl),  np.cos(steer_fl)]])

        R_steer_fr = np.array([[np.cos(steer_fr), -np.sin(steer_fr)],
                               [np.sin(steer_fr),  np.cos(steer_fr)]])

        fr_wheel_footprint = R_steer_fr.dot(car.wheel_footprint.T).T
        fr_wheel_footprint [:,1] -= car.axle_track / 2.0
        fr_wheel_footprint [:,0] += car.wheelbase
        fr_wheel_footprint = R_car.dot(fr_wheel_footprint.T).T
        fr_wheel_footprint[:,0] += x
        fr_wheel_footprint[:,1] += y

        fl_wheel_footprint = R_steer_fl.dot(car.wheel_footprint.T).T
        fl_wheel_footprint [:,1] += car.axle_track / 2.0
        fl_wheel_footprint [:,0] += car.wheelbase
        fl_wheel_footprint = R_car.dot(fl_wheel_footprint.T).T
        fl_wheel_footprint[:,0] += x
        fl_wheel_footprint[:,1] += y

        car_plot = ()

        car_plot += tuple(self.ax.plot(car_front_footprint[:,0], car_front_footprint[:,1],           lw=lw, color=color))
        car_plot += tuple(self.ax.plot(car_left_side_footprint[:,0], car_left_side_footprint[:,1],   lw=lw, color=color))
        car_plot += tuple(self.ax.plot(car_right_side_footprint[:,0], car_right_side_footprint[:,1], lw=lw, color=color))
        car_plot += tuple(self.ax.plot(car_rear_footprint[:,0], car_rear_footprint[:,1],             lw=lw, color=color))

        car_plot += tuple(self.ax.plot(rr_wheel_footprint[:,0], rr_wheel_footprint[:,1], lw=lw, color=color))
        car_plot += tuple(self.ax.plot(rl_wheel_footprint[:,0], rl_wheel_footprint[:,1], lw=lw, color=color))
        car_plot += tuple(self.ax.plot(fr_wheel_footprint[:,0], fr_wheel_footprint[:,1], lw=lw, color=color))
        car_plot += tuple(self.ax.plot(fl_wheel_footprint[:,0], fl_wheel_footprint[:,1], lw=lw, color=color))

        return car_plot


def update_plot(frame, car, plotter, traj_plot=None):

    info_data = ""

    if (frame < len(car.history_x)):
        car.x = car.history_x[frame]
        car.y = car.history_y[frame]
        car.yaw = car.history_yaw[frame]
        car.set_steering(car.history_steer[frame])
        # plotter.title.set_text("Time[s]:" + str(round(car.history_t[frame], 2))+
        #           # ", accel[m/s]:" + str(round(accel, 2)) +

        info_data = "Time[s]:" + str(round(car.history_t[frame], 2)) + \
                    "\nSpeed[km/h]:" + str(round(car.history_v[frame] * 3.6, 2))

        info = plotter.ax.text(0.75, 0.92, info_data,
                                bbox={'facecolor':'w', 'alpha':0.5, 'pad':5},
                                ha="left", transform=plotter.ax.transAxes, ),

    else:
        car.x = car.history_x[-1]
        car.y = car.history_y[-1]
        car.yaw = car.history_yaw[-1]
        car.set_steering(car.history_steer[-1])

    plotting_tuple = ()

    if(traj_plot):
        plotting_tuple += traj_plot

    plotting_tuple += plotter.draw_car(car, car.x, car.y, car.yaw, car.steer)
    plotting_tuple += plotter.draw_frame(car.x, car.y, car.yaw)
    plotting_tuple += plotter.draw_additional_car_geometry(car, car.x, car.y, car.yaw, car.steer)

    if (info_data):
        plotting_tuple += info

    return plotting_tuple


def main():
    # with plt.xkcd():

    car = Car()
    car.generate_8_traj_by_control(dt=0.02)

    plotter = Plotter([-13,13], [-13,13])

    animation_dt = 0.05
    animation_period = 1000./(1./animation_dt)
    traj_plot = plotter.draw_traj_from_history(car, dt=animation_dt, frames=True,
                                                                     frames_dt=0.4,
                                                                     footprints=True,
                                                                     footprints_dt=0.4)

    animation = FuncAnimation(plotter.fig,
                              update_plot,
                              fargs=(car, plotter, traj_plot),
                              interval=40,          # if too large and blit is True
                              blit=True)            # throw an exeption but works somehow

    plt.show()


if __name__ == "__main__":
    main()
