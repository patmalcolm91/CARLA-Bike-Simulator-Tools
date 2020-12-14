"""
Classes that handle the vehicle dynamics aspect of the simulation.
"""

from __future__ import print_function

import carla

from pygame.locals import K_COMMA
from pygame.locals import K_DOWN
from pygame.locals import K_LEFT
from pygame.locals import K_PERIOD
from pygame.locals import K_RIGHT
from pygame.locals import K_SPACE
from pygame.locals import K_UP
from pygame.locals import K_a
from pygame.locals import K_d
from pygame.locals import K_m
from pygame.locals import K_q
from pygame.locals import K_s
from pygame.locals import K_w


import time
from scipy.integrate import ode
import numpy as np
import math


class VehicleDynamics:
    """Template class for Vehicle Dynamics."""
    def __init__(self, actor, **kwargs):
        """
        Initialize a VehicleDynamics object.

        :param actor: carla Actor object whose physics are to be controlled.
        :type actor: carla.Actor
        """
        self.player = actor
        self._kwargs = kwargs

    def tick(self, **kwargs):
        """Function to be called every simulation step."""
        raise NotImplementedError("Child class must override tick() method.")


class VehicleDynamicsPaul(VehicleDynamics):
    """Vehicle Dynamics Model developed by Paul Pabst in his Master Thesis work."""
    def __init__(self, actor, torque_curve=None, steering_curve=None, max_rpm=500, throttle_scale=1600,
                 brake_threshold=0.0025, steering_scale=90):
        super().__init__(actor)
        self.throttle = 0
        self.brake = 0
        self.steering_scale = steering_scale
        self.throttle_scale = throttle_scale
        self.brake_threshold = brake_threshold
        self.physics = self.player.get_physics_control()
        if isinstance(self.player, carla.Vehicle):
            self._control = carla.VehicleControl()
        else:
            raise NotImplementedError("Actor type not supported")
        self.physics.max_rpm = max_rpm
        self.physics.torque_curve = torque_curve if torque_curve is not None else [
            (0, 200),
            (200, 200),
            (500, 30)
        ]
        self.physics.steering_curve = steering_curve if steering_curve is not None else [
            (0, 1),
            (55, 0.2),
            (120, 0.1)
        ]
        # Theoretical setup to adjust engine to use a single gear:
        # physics.forward_gears = [
        #   carla.GearPhysicsControl(
        #       ratio = float(XX),
        #       down_ratio = float(XX),
        #       up_ratio = float(XX)
        #       )
        # ]
        self.player.apply_physics_control(self.physics)

    def tick(self, speed_input, steering_input):
        # scale and apply steering
        self._control.steer = self._control.steer = steering_input / self.steering_scale
        # check if speed value has changed
        #   check if speed decrease exceeds brake_threshold else set brake to 0
        #       set brake to how much brake_threshold is exceeded
        # cosmetic: if throttle is 0 set brake to 0
        # store reference brake value
        if self.throttle != speed_input / self.throttle_scale:
            if self.throttle - speed_input / self.throttle_scale > self.brake_threshold:
                self.brake = self.throttle - (speed_input / self.throttle_scale + self.brake_threshold)
            else:
                self.brake = 0
        elif self.throttle == 0:
            self.brake = 0
        self._control.brake = self.brake

        # scale simulator speed output and limit to maximum 1
        # store reference throttle value
        if speed_input / self.throttle_scale >= 1:
            self._control.throttle = 1
        else:
            self._control.throttle = speed_input / self.throttle_scale
        self.throttle = speed_input / self.throttle_scale

        self._control.reverse = self._control.gear < 0
        self.player.apply_control(self._control)


class VehicleDynamicsSingleTrack(VehicleDynamics):
    """Vehicle Dynamics Model developed by Georgios Grigoropoulos"""
    def __init__(self, actor, start_yaw=0, rpm_factor=1600, start_v=0, start_delta=0):
        super().__init__(actor)

        self.rpm_factor = rpm_factor
        self.physics = self.player.get_physics_control()
        self.player.set_simulate_physics(False)

        self.b = 0.0  # side slip angle
        self.yaw = start_yaw
        self.sol = ode(self.singletrack_fun).set_integrator("dopri5", max_step=0.050)
        self.sol.set_initial_value([0, self.b]).set_f_params(start_v, start_delta)

        self.L = 0.1  # in m
        self.f = 0.3  # factor descripbing the porportion of the width of the bike tire that is in contact with the road surface
        self.br = 0.032 * self.f
        self.ka = 20000  # self defined constant. This constat is defined for tires through lab experiments
        self.c = 0.5 * self.br * self.ka * self.L ** 2  # L is approximately the length of the surface covered by the wheel page169 assumption of a constant patch breadth br (width of tire) page171 with a corresponding constant k. The constant depends on
        self.cav = self.c  # front wheel cornering stiffness  For small slip angles less than 12 degrees a has a linear relationship to the Fy lateral force it is then valid Fy =ca*a page 176(164)
        self.cah = self.c  # rear wheel cornering stiffness For small slip angles a it is then valid a =ca*a page 176(164)
        self.lb = 1.02  # bike body length in meters
        self.lv = 0.43  # distance from the center of gravity to the front wheel
        self.lh = self.lb - self.lv  # distances from the center of gravity to the front
        self.theta = 2.8  # yaw inertia check  Table 11.8 page 315 thetazz here value is from Meijaard et al.
        self.m_bike = 6  # mass of bike
        self.m_rider = 65  # mass of rider
        self.m = self.m_bike + self.m_rider  # mass of bike + mass of rider

    def singletrack_fun(self, t, x, v, delta):
        """
        Vehicle dynamics by Schramm et al. 2018 (DOI: 10.1007/978-3-662-54483-9)
        Example values on page 315
        https://www.coursera.org/lecture/intro-self-driving-cars/lesson-5-lateral-dynamics-of-bicycle-model-1Opvo
        https://www.bvl.de/files/1951/1988/1852/2239/10.23773-2017_1.pdf
        [1] J. P. Meijaard, J. M. Papadopoulos, A. Ruina, and A. L. Schwab, “Linearized dynamics equations for the balance and steer of a bicycle: a benchmark and review,” Proc. R. Soc. A Math. Phys. Eng. Sci., vol. 463, no. 2084, pp. 1955–1982, 2007.

        :param t: current time
        :param x: vector of inputs [psi_dot_V, beta]
        :param v: velocity
        :param delta: steering angle
        """
        if v == 0:
            return [0, 0]

        u = delta
        A11 = -(1 / v) * (self.cav * (self.lv ** 2) + self.cah * (self.lh ** 2)) / self.theta
        A12 = -(self.cav * self.lv - self.cah * self.lh) / self.theta
        A21 = -1 - (1 / v ** 2) * (self.cav * self.lv - self.cah * self.lh) / self.m
        A22 = -(1 / v) * (self.cav + self.cah) / self.m

        B11 = self.cav * self.lv / self.theta
        B12 = (1 / v) * (self.cav / self.m)

        A = np.array([[A11, A12], [A21, A22]])
        B = np.array([B11, B12])

        xprime = np.dot(A, x) + np.dot(B, u)

        return np.array([xprime[0], xprime[1]])

    def tick(self, speed_input, steering_input, time_step):
        delta = steering_input*math.pi/180  # steering input in radians
        v = np.float(speed_input / self.rpm_factor)  # velocity
        transform = self.player.get_transform()  # https://carla.readthedocs.io/en/latest/python_api/#carla.Transform rotation is yaw it is around center of gravity. Get initital values from last tick

        self.sol.set_f_params(v, delta)
        yaw_rate, b = self.sol.integrate(self.sol.t+time_step/1000)  # return result from differential equytion of single track model motion

        self.b = b
        self.yaw += yaw_rate * time_step/1000

        transform.location.x += v * math.cos(self.yaw)
        transform.location.y += v * math.sin(self.yaw)
        transform.location.z = 0
        transform.rotation.yaw = self.yaw*180/math.pi

        self.player.set_transform(transform)


class VehicleDynamicsKeyboard(VehicleDynamics):
    """Vehicle Dynamics Model using keyboard as input. For debugging purposes."""
    def __init__(self, actor):
        super().__init__(actor)
        if isinstance(self.player, carla.Vehicle):
            self._control = carla.VehicleControl()
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0

    def tick(self, event, keys, milliseconds):
        if hasattr(event, "key"):
            if event.key == K_q:
                self._control.gear = 1 if self._control.reverse else -1

            elif event.key == K_m:
                self._control.manual_gear_shift = not self._control.manual_gear_shift
                self._control.gear = self.player.get_control().gear
            elif self._control.manual_gear_shift and event.key == K_COMMA:
                self._control.gear = max(-1, self._control.gear - 1)
            elif self._control.manual_gear_shift and event.key == K_PERIOD:
                self._control.gear = self._control.gear + 1

        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]
        self.player.apply_control(self._control)

