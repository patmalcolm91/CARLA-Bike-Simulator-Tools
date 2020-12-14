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
    def __init__(self, actor, torque_curve=None, steering_curve=None, max_rpm=500):
        super().__init__(actor)

        self.throttle = 0
        self.brake = 0
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

        self.player.apply_physics_control(self.physics)

        self.b = 0.0  # side slip angle
        self.delta = 0.0
        self.psi_V = 0.0
        self.v = 0.0
        y = np.array([self.psi_V, self.b, self.v, self.delta])
        milliseconds = 50
        self.sol = ode(self.singletrack_fun).set_integrator("dopri5", max_step=np.float(milliseconds/1000))
        self.sol.set_initial_value(y, 0)  # time.time()
        print(y, "init")

    def singletrack_fun(self, t, y):
        """
        Vehicle dynamics by Schramm et al. 2018 (DOI: 10.1007/978-3-662-54483-9)
        Example values on page 315
        https://www.coursera.org/lecture/intro-self-driving-cars/lesson-5-lateral-dynamics-of-bicycle-model-1Opvo
        https://www.bvl.de/files/1951/1988/1852/2239/10.23773-2017_1.pdf
        [1] J. P. Meijaard, J. M. Papadopoulos, A. Ruina, and A. L. Schwab, “Linearized dynamics equations for the balance and steer of a bicycle: a benchmark and review,” Proc. R. Soc. A Math. Phys. Eng. Sci., vol. 463, no. 2084, pp. 1955–1982, 2007.

        :param t: current time
        :param y: y is vector of inputs y[2] is velocity, y[3] is steering angle
        """

        L = 0.1  # in m
        f = 0.3  # factor descripbing the porportion of the width of the bike tire that is in contact with the road surface
        br = 0.032 * f
        ka = 20000  # self defined constant. This constat is defined for tires through lab experiments
        c = 0.5 * br * ka * L ** 2  # L is approximately the length of the surface covered by the wheel page169 assumption of a constant patch breadth br (width of tire) page171 with a corresponding constant k. The constant depends on
        cav = c  # front wheel cornering stiffness  For small slip angles less than 12 degrees a has a linear relationship to the Fy lateral force it is then valid Fy =ca*a page 176(164)
        cah = c  # rear wheel cornering stiffness For small slip angles a it is then valid a =ca*a page 176(164)
        lb = 1.02  # bike body length in meters
        lv = 0.43  # distance from the center of gravity to the front wheel
        lh = lb - lv  # distances from the center of gravity to the front
        theta = 2.8  # yaw inertia check  Table 11.8 page 315 thetazz here value is from Meijaard et al.
        m_bike = 6  # mass of bike
        m_rider = 65  # mass of rider
        m = m_bike + m_rider  # mass of bike + mass of rider

        self.psi_V_d = y[0]
        self.b = y[1]
        self.v = y[2]
        self.delta = y[3]
        x = np.array([self.psi_V_d, self.b])
        u = np.array(self.delta)
        v = self.v
        A11 = -(1 / v) * (cav * (lv ** 2) + cah * (lh ** 2)) / theta
        A12 = -(cav * lv - cah * lh) / theta
        A21 = -1 - (1 / v ** 2) * (cav * lv - cah * lh) / m
        A22 = -(1 / v) * (cav + cah) / m

        B11 = cav * lv / theta
        B12 = (1 / v) * (cav / m)

        A = np.array([[A11, A12], [A21, A22]])
        B = np.array([B11, B12])

        xprime = np.dot(A, x) + np.dot(B, u)
        print(A11, A12, A21, A22, B11, B12, x, u, "A")
        print(np.dot(A, x))
        print(np.dot(B, u))
        print(np.dot(A, x) + np.dot(B, u))
        print(xprime)
        print(xprime[0], xprime[1])

        return np.array([xprime[0], xprime[1], x[0], x[1]])

    def tick(self, speed_input, steering_input, milliseconds):

        def dist_fun(t, y):  # y is vector of inputs y[0] is x_dist and y[1] is y_dist. Initial values come from previous location
            v_prime = y
            return v_prime

        self.delta = steering_input*math.pi/180 #is the steering input angle
        print (speed_input)
        self.v = np.float(speed_input / 1600) #velocity
        transform = self.player.get_transform() #https://carla.readthedocs.io/en/latest/python_api/#carla.Transform rotation is yaw it is around center of gravity. Get initital values from last tick
        # scale and apply steering
        self.psi_V = (transform.rotation.yaw)*math.pi/180  # yaw rate #used parameters get_angular_velocity

        y = np.array([self.psi_V, self.b, self.v, self.delta])
        print(y, "external")

        self.sol.set_f_params(y)
        psi_V_2d, b_d, psi_V_d, b = self.sol.integrate(self.sol.t+milliseconds/1000) #return result from differential equytion of single track model motion
#        startTime = time.time()
        print (psi_V_2d, b_d, psi_V_d, b,"FEXPORT")
        self.b = b
        psi_V = self.psi_V + psi_V_d * milliseconds / 1000
        vv = np.array([self.v*math.cos(psi_V),self.v*math.sin(psi_V),0])
        print("values", "velocity", self.v, "delta steering", self.delta, "milliseconds", milliseconds,
              "slip angle", self.b, "yaw", self.psi_V, "yaw rate", psi_V_d, "b_d", b_d)

        sol_dist = ode(dist_fun)
        sol_dist.set_initial_value(vv, 0)
#        print ('The script took {0} second !'.format(time.time() - startTime))
        x_dist, y_dist, z_dist = sol_dist.integrate(sol_dist.t+milliseconds/1000) # return result from differential equytion of single track model motion

        transform.location.x += x_dist
        transform.location.y += y_dist
        transform.location.z = 0
        transform.rotation.yaw = 180*psi_V/math.pi

        self.player.set_transform(transform)
        print('moved bike to %s' % transform, x_dist, y_dist, b)


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

