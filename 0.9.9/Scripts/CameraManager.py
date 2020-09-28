"""
Stripped-down camera manager class
"""


from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


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


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
# all unneeded imports were removed

import carla

from carla import ColorConverter as cc

import weakref

try:
    import pygame
    from pygame.locals import K_ESCAPE
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================
# Decoupling of surface resolution and display size has been implemented.


class CameraManager(object):

    DEFAULT_PARAMS = {
        "resolution": (1920, 1080),
        "camera_x": 0,
        "camera_y": 0,
        "camera_z": 0,
        "camera_roll": 0,
        "camera_pitch": 0,
        "camera_yaw": 0,
        "fov": 70,
        "gamma": 2.2
    }

    def __init__(self, parent_actor, display_size, **camera_params):
        camera_params = {**self.DEFAULT_PARAMS, **camera_params}
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.display_size = display_size
        self.resolution = camera_params["resolution"]
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        attachment = carla.AttachmentType
        self._camera_transforms = [
            (carla.Transform(carla.Location(x=camera_params["camera_x"],
                                            y=camera_params["camera_y"],
                                            z=camera_params["camera_z"]),
                             carla.Rotation(roll=camera_params["camera_roll"],
                                            pitch=camera_params["camera_pitch"],
                                            yaw=camera_params["camera_yaw"])), attachment.Rigid)]
        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
        ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):

                # set field of view and surface resolution
                bp.set_attribute('fov', str(camera_params["fov"]))
                bp.set_attribute('image_size_x', str(self.resolution[0]))
                bp.set_attribute('image_size_y', str(self.resolution[1]))

                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(camera_params["gamma"]))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            item.append(bp)
        self.index = None

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def set_sensor(self, index, force_respawn=False):
        index = index % len(self.sensors)
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        self.index = index

    def render(self, display):
        if self.surface is not None:
            # scale surface to display size
            display.blit(pygame.transform.scale(self.surface, self.display_size), (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        image.convert(self.sensors[self.index][1])
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
