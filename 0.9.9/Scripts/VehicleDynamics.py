"""
Classes that handle the vehicle dynamics aspect of the simulation.
"""

from __future__ import print_function

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

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

