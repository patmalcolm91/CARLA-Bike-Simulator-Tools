from __future__ import print_function

import os
import glob
import time
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import time
import yaml
import argparse
import logging

from CameraManager import CameraManager

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
        self.display_size = args.display_size
        self.resolution = args.resolution

        self.player = None
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == args.rolename:
                self.player = actor
                break
        self.camera_manager = None
        self.camera_params = {k: getattr(args, k, CameraManager.DEFAULT_PARAMS[k]) for k in CameraManager.DEFAULT_PARAMS}
        self.restart()

    def restart(self):
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        self.camera_manager = CameraManager(self.player, **self.camera_params)
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
        return key == K_ESCAPE


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
    refresh_rate = int(args.refresh_rate)
    display_size = args.display_size

    try:
        logging.info('listening to server %s:%s', args.host, args.port)
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % tuple(args.window_pos)

        display = pygame.display.set_mode(
            display_size,
            pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.NOFRAME)

        world = World(client.get_world(), args)

        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(refresh_rate)
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
        '--resolution',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--display_size',
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
        '--refresh_rate',
        metavar='refresh_rate',
        default=20,
        help='refresh rate of clients')
    argparser.add_argument(
        '--config',
        type=str,
        help='Path to YAML config file containing display settings.')
    argparser.add_argument(
        '--display_name',
        metavar='display_name',
        help='Name of display. Used to get parameters from config file.')
    args = argparser.parse_args()

    if args.config is not None:
        # read config file
        with open(args.config) as f:
            cfg = yaml.load(f, Loader=yaml.Loader)
        cfg_default_params = cfg.get("default", {})
        cfg_display_params = cfg.get(args.display_name, {})
        cfg_params = {**cfg_default_params, **cfg_display_params}
        # set parameters
        args.resolution = cfg_params.pop("resolution", args.resolution)
        args.display_size = cfg_params.pop("display_size", args.display_size)
        for param in cfg_params:
            setattr(args, param, cfg_params[param])

    args.resolution = tuple([int(x) for x in args.resolution.split('x')])
    args.display_size = tuple([int(x) for x in args.display_size.split('x')])

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    print(__doc__)

    try:
        game_loop(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()
