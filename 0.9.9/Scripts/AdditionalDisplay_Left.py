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

import time
import argparse
import logging
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
# -- World ---------------------------------------------------------------------
# ==============================================================================
# HUD was completely erased from this module.
# Controller has been removed and parse_events has been integrated into World
# Decoupling of surface resolution and display size was implemented.
# A routine that looks for the 'hero' vehicle was added.
# restart() was stripped of all functions except to create a CameraManager

class World(object):
    def __init__(self, carla_world, args):
        self.world = carla_world
        self.displaysize = (args.dis_width, args.dis_height)
        self.resolution = (args.res_width, args.res_height)

        self.player = None
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == args.rolename:
                self.player = actor
                break
        self.camera_manager = None
        self._gamma = args.gamma
        self.restart()

    def restart(self):
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        self.camera_manager = CameraManager(self.player, self._gamma, self.resolution, self.displaysize)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index)

    def render(self, display):
        self.camera_manager.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        actors = [
            self.camera_manager.sensor,
            ]
        for actor in actors:
            if actor is not None:
                actor.destroy()

    def parse_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE)


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================
# Decoupling of surface resolution and display size has been implemented.
# The camera Transform has been adjusted to the left screen


class CameraManager(object):
    def __init__(self, parent_actor, gamma_correction, resolution, displaysize):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.displaysize = displaysize
        self.resolution = resolution
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        Attachment = carla.AttachmentType
        self._camera_transforms = [
            (carla.Transform(carla.Location(x=0.3, z=1.7), carla.Rotation(yaw=-70)), Attachment.Rigid)]
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
                bp.set_attribute('fov', str(70))
                bp.set_attribute('image_size_x', str(self.resolution[0]))
                bp.set_attribute('image_size_y', str(self.resolution[1]))

                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            item.append(bp)
        self.index = None

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
            display.blit(pygame.transform.scale(self.surface, self.displaysize), (0, 0))

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


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================
# sleep function has been added to wait for the hero vehicle to spawn.
# Spawn position of windows has been defined.
# NOFRAME flag was added to display.

def game_loop(args):
    time.sleep(int(args.sleep))
    pygame.init()
    pygame.font.init()
    world = None

    refreshrate = int(args.refresh)

    # displaysize will be screen size
    displaysize = (args.dis_width, args.dis_height)

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (1920, 0)
        display = pygame.display.set_mode(
            displaysize,
            pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.NOFRAME)

        world = World(client.get_world(), args)

        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(refreshrate)
            if world.parse_events():
                return
            world.render(display)
            pygame.display.flip()

    finally:

        if world is not None:
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================
# Arguments for display size, sleep time and refresh rate have been added.
# Resolution and display size have been calibrated to the monitor position.

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='960x540',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--dis',
        metavar='WIDTHxHEIGHT',
        default='1920x1080',
        help='display size (default: 1920x1080)')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sleep',
        metavar='sleep',
        default=0,
        help='wait for hero vehicle to launch')
    argparser.add_argument(
        '--refresh',
        metavar='refreshrate',
        default=20,
        help='refresh rate of clients')
    args = argparser.parse_args()

    args.res_width, args.res_height = [int(x) for x in args.res.split('x')]
    args.dis_width, args.dis_height = [int(x) for x in args.dis.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
