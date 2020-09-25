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


import carla

from carla import ColorConverter as cc

import time
# importlib.import_module('manual_control_ext')
import socket


import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref

try:
    import pygame
    from pygame.locals import K_f
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


class World(object):
    def __init__(self, carla_world, args):
        self.world = carla_world
        self.actor_role_name = args.rolename
        self.displaysize = (args.dis_width, args.dis_height)
        self.resolution = (args.res_width, args.res_height)

        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        #self.hud = hud

        self.player = None
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == 'hero':
                self.player = actor
                break
        self.camera_manager = None
        #self._weather_presets = find_weather_presets()
        #self._weather_index = 0
        #self._actor_filter = args.filter
        self._gamma = args.gamma
        self.restart()
        #self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0

    def restart(self):
        #self.player_max_speed = 1.589
        #self.player_max_speed_fast = 3.713
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        #blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        #blueprint.set_attribute('role_name', self.actor_role_name)
        #if blueprint.has_attribute('color'):
        #    color = random.choice(blueprint.get_attribute('color').recommended_values)
        #    blueprint.set_attribute('color', color)
        #if blueprint.has_attribute('driver_id'):
        #    driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
        #    blueprint.set_attribute('driver_id', driver_id)
        #if blueprint.has_attribute('is_invincible'):
        #    blueprint.set_attribute('is_invincible', 'true')
        # set the max speed
        #if blueprint.has_attribute('speed'):
        #    self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
        #    self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])
        #else:
        #    print("No recommended values for 'speed' attribute")
        # Spawn the player.
        #if self.player is not None:
        #    spawn_point = self.player.get_transform()
        #    spawn_point.location.z += 2.0
        #    spawn_point.rotation.roll = 0.0
        #    spawn_point.rotation.pitch = 0.0
        #    self.destroy()
        #    self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        #while self.player is None:
        #    if not self.map.get_spawn_points():
        #        print('There are no spawn points available in your map/town.')
        #        print('Please add some Vehicle Spawn Point to your UE4 scene.')
        #        sys.exit(1)
        #    spawn_points = self.map.get_spawn_points()
        #    spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
        #    self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        # Set up the sensors.
        #self.collision_sensor = CollisionSensor(self.player, self.hud)
        #self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        #self.gnss_sensor = GnssSensor(self.player)
        #self.imu_sensor = IMUSensor(self.player)
        ##oc# self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager = CameraManager(self.player, self._gamma, self.resolution, self.displaysize)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        #actor_type = get_actor_display_name(self.player)
        #self.hud.notification(actor_type)

    # def next_weather(self, reverse=False):
    #     self._weather_index += -1 if reverse else 1
    #     self._weather_index %= len(self._weather_presets)
    #     preset = self._weather_presets[self._weather_index]
    #     self.hud.notification('Weather: %s' % preset[1])
    #     self.player.get_world().set_weather(preset[0])
    #
    # def toggle_radar(self):
    #     if self.radar_sensor is None:
    #         self.radar_sensor = RadarSensor(self.player)
    #     elif self.radar_sensor.sensor is not None:
    #         self.radar_sensor.sensor.destroy()
    #         self.radar_sensor = None

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        #self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        #if self.radar_sensor is not None:
        #    self.toggle_radar()
        actors = [
            self.camera_manager.sensor,
            #self.collision_sensor.sensor,
            #self.lane_invasion_sensor.sensor,
            #self.gnss_sensor.sensor,
            #self.imu_sensor.sensor,
            #self.player
            ]
        for actor in actors:
            if actor is not None:
                actor.destroy()

# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world, displaysize, screen):
        self.fullscreen = False
        self.displaysize = displaysize
        self.screen = screen
        #self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._lights = carla.VehicleLightState.NONE
            #world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        #world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

    def parse_events(self):
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_f:
                    self.fullscreen = not self.fullscreen
                    if self.fullscreen:
                        self.screen = pygame.display.set_mode(self.displaysize,
                                                pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.FULLSCREEN)
                    else:
                        self.screen = pygame.display.set_mode(self.displaysize,
                                                pygame.HWSURFACE | pygame.DOUBLEBUF)

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE)

# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        #self._notifications = FadingText(font, (width, 40), (0, height - 40))
        #self.help = HelpText(pygame.font.Font(mono, 16), width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = False
        self._info_text = []
        self._server_clock = pygame.time.Clock()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        compass = world.imu_sensor.compass
        heading = 'N' if compass > 270.5 or compass < 89.5 else ''
        heading += 'S' if 90.5 < compass < 269.5 else ''
        heading += 'E' if 0.5 < compass < 179.5 else ''
        heading += 'W' if 180.5 < compass < 359.5 else ''
        colhist = world.collision_sensor.get_collision_history()
        collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        max_col = max(1.0, max(collision))
        collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name,
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Compass:% 17.0f\N{DEGREE SIGN} % 2s' % (compass, heading),
            'Accelero: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.accelerometer),
            'Gyroscop: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.gyroscope),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        self._info_text += [
            '',
            'Collision:',
            collision,
            '',
            'Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            for d, vehicle in sorted(vehicles):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        #self._notifications.render(display)
        #self.help.render(display)


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, gamma_correction, resolution, displaysize):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.displaysize = displaysize
        self.resolution = resolution
        #self.hud = hud
        self.recording = False
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        Attachment = carla.AttachmentType
        self._camera_transforms = [
            (carla.Transform(carla.Location(x=0.3, z=1.7), carla.Rotation(yaw=-136)), Attachment.Rigid)]
        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            # ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
            # ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
            # ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
            # ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)', {}],
            # ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
            #     'Camera Semantic Segmentation (CityScapes Palette)', {}],
            # ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {}],
            # ['sensor.camera.rgb', cc.Raw, 'Camera RGB Distorted',
            #     {'lens_circle_multiplier': '3.0',
            #     'lens_circle_falloff': '3.0',
            #     'chromatic_aberration_intensity': '0.5',
            #     'chromatic_aberration_offset': '0'}]
        ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                #bp.set_attribute('image_size_x', str(hud.dim[0]))
                #bp.set_attribute('image_size_y', str(hud.dim[1]))
                bp.set_attribute('fov', str(62))
                bp.set_attribute('image_size_x', str(self.resolution[0]))
                bp.set_attribute('image_size_y', str(self.resolution[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            elif item[0].startswith('sensor.lidar'):
                bp.set_attribute('range', '50')
            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
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
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            #self.surface = pygame.transform.scale(self.surface, self.displaysize)
            #display.blit(self.surface, (0, 0))
            display.blit(pygame.transform.scale(self.surface, self.displaysize), (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 3), 3))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / 100.0
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype = int)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)



def game_loop(args):
    time.sleep(int(args.sleep))

    pygame.init()
    pygame.font.init()
    world = None

    # displaysize will be screen size
    displaysize = (args.dis_width, args.dis_height)

    try:
        client = carla.Client(args.host, args.port)

        client.set_timeout(2.0)

        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (67, 27)
        display = pygame.display.set_mode(
            displaysize,
            pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.NOFRAME)

        world = World(client.get_world(), args)
        controller = KeyboardControl(world, displaysize, display)

        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)
            if controller.parse_events():
                return
            #world.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


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
        help='TCP port of CARLA server to listen to (default: 2000)')
    argparser.add_argument(
        '-cp', '--client_port',
        metavar='P',
        default=5000,
        type=int,
        help='TCP port of manual_control client to listen to (default: 5000)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='800x450',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--dis',
        metavar='WIDTHxHEIGHT',
        default='1853x1053',
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
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--sleep',
        metavar='sleep',
        default=0,
        help='wait for hero vehicle to launch')
    args = argparser.parse_args()

    #client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM | socket.SOCK_NONBLOCK)



    #Pseudocode
    # check if ID is  None
    #if args.id is None:
    #    try:
    #        client_socket.connect(args.host, args.cp)
    #    except:
    #        args.id = client_socket.recv(1024)


    # either an ID was handed over or
    # connect to server // Pipe
    # if args.id = 0
    #       exit

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
