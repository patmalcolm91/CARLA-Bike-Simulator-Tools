#!/usr/bin/env python

# Copyright (c) 2019 Intel Labs
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Bike Simulator Main Client
"""


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

# additional imports
from socket import *  # used for Arduino connection
import os  # used to define the display position

import carla

from carla import ColorConverter as cc

import argparse
import yaml
import collections
import datetime
import logging
import math
import random
import re
import weakref
import warnings

from BikeSensor import BikeSensor
from VehicleDynamics import VehicleDynamicsPaul, VehicleDynamicsKeyboard, VehicleDynamicsSingleTrack
from DataSync import Server as DataSyncServer

from configparser import ConfigParser

import numpy as np


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================

def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================

class World:
    def __init__(self, carla_world, args):
        self.world = carla_world
        self.display_size = args.display_size  # set up correct display size for CameraManager
        self.resolution = args.resolution  # set up the correct resolution for CameraManager
        self.actor_role_name = args.rolename  # allow to use a custom player name

        # following section was copied from manual_control.py
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)

        scenario_cfg = {}
        if args.scenario_config is not None:
            with open(args.scenario_config) as f:
                scenario_cfg = yaml.load(f, Loader=yaml.Loader)
        self.data_sync_server = None
        self.data_sync_instr_image_index = None
        if "message_servers" in scenario_cfg:
            if "instruction_images" in scenario_cfg:
                dscfg = scenario_cfg["message_servers"]["instruction_images"]
                self.data_sync_server = DataSyncServer(ip=dscfg["ip"], port=dscfg["port"], fmt=dscfg["fmt"])
                self.data_sync_instr_image_index = dscfg["index"]
        self.spectator = self.world.get_spectator()
        self.player = None
        self._player_blueprint_name = "vehicle.diamondback.century"
        self._player_start_pos = None
        self._player_start_yaw = None
        if "ego" in scenario_cfg:
            if "blueprint" in scenario_cfg["ego"]:
                self._player_blueprint_name = scenario_cfg["ego"]["blueprint"]
            if "start_pos" in scenario_cfg["ego"]:
                self._player_start_pos = tuple(scenario_cfg["ego"]["start_pos"])
            if "role_name" in scenario_cfg["ego"]:
                self.actor_role_name = scenario_cfg["ego"]["role_name"]
            if "start_yaw" in scenario_cfg["ego"]:
                self._player_start_yaw = scenario_cfg["ego"]["start_yaw"]
        self._actor_filter = args.filter
        self.camera_params = {k: getattr(args, k, CameraManager.DEFAULT_PARAMS[k]) for k in CameraManager.DEFAULT_PARAMS}
        self.restart()  # On World instantiation use bicycle and new engine setup
        # self.world.on_tick(hud.on_world_tick)

    def restart(self):
        blueprint = self.world.get_blueprint_library().find(self._player_blueprint_name)
        blueprint.set_attribute("role_name", self.actor_role_name)
        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        while self.player is None:
            if self._player_start_pos is None:
                spawn_points = self.world.get_map().get_spawn_points()
                spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            else:
                spawn_point = carla.Transform(carla.Location(*self._player_start_pos),
                                              carla.Rotation(roll=0, pitch=0, yaw=self._player_start_yaw))
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)

    def tick(self, clock):
        actor_transform = self.player.get_transform()
        self.spectator.set_transform(actor_transform.transform(carla.Vector3D(0, 0, 1.6)))
        if self.data_sync_server is not None:
            msgs = self.data_sync_server.get_messages()
        else:
            msgs = []
        if len(msgs) > 0:
            msg = msgs[-1]
            instruction = msg if self.data_sync_instr_image_index is None else msg[self.data_sync_instr_image_index]
            instruction = instruction if instruction != 0 else None
            # TODO: implement instruction display

    def destroy(self):
        actors = [self.player]
        for actor in actors:
            if actor is not None:
                actor.destroy()


# ==============================================================================
# -- Control -----------------------------------------------------------
# ==============================================================================

class Control():
    def __init__(self, world, dynamics_model="single-track"):
        self._steer_cache = 0.0
        self._dynamics_model = dynamics_model
        if self._dynamics_model == "paul":
            self._vehicle_dynamics_paul = VehicleDynamicsPaul(world.player, steering_scale=135)
        elif self._dynamics_model == "single-track":
            self._vehicle_dynamics_single_track = VehicleDynamicsSingleTrack(world.player)
        else:
            raise NotImplementedError("Invalid dynamics model specified in DualControl.")

    def parse_events(self, world, clock, bike_sensor):
        self._parse_vehicle_controller_input(bike_sensor, clock)

    def _parse_vehicle_controller_input(self, bike_sensor, clock):
        # request sensor outputs from arduino
        speed, steering = bike_sensor.get_speed_and_steering()
        # send the sensor readings to the vehicle dynamics module
        if self._dynamics_model == "single-track":
            self._vehicle_dynamics_single_track.tick(speed_input=speed, steering_input=steering, time_step=clock.get_time())
        elif self._dynamics_model == "paul":
            self._vehicle_dynamics_paul.tick(speed_input=speed, steering_input=steering)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================
# The game loop now will instantiate an arduino object that stores the connection
# to the arduino. A line of code was added before launching the display to set
# its position. The display got an additional NOFRAME flag. The update rate of
# the client has been reduced from 60 to 20.


def game_loop(args):
    world = None
    refresh_rate = int(args.refresh_rate)
    display_size = args.display_size

    try:
        logging.info('listening to server %s:%s', args.host, args.port)
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        bike_sensor = BikeSensor()

        world = World(client.get_world(), args)
        controller = Control(world)

        while True:
            clock.tick_busy_loop(refresh_rate)  # TODO: replace with non-pygame function
            if controller.parse_events(world, clock, bike_sensor):
                return
            world.tick(clock)
            world.render()

    finally:

        if world is not None:
            world.destroy()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================
# Three additional arguments have been added: display size, rolename and gamma.
# Rolename and gamma have been directly adapted from manual_control.py. Display
# size is based on the existing resolution argument, which use has been slightly
# repurposed to reflect the decoupling of display size and surface resolution.

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
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
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
    argparser.add_argument(
        '--scenario_config',
        type=str,
        default=None,
        help='Path to YAML config file containing scenario settings.')
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

    try:
        game_loop(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()
