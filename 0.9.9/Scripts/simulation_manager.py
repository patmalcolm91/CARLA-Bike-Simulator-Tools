"""
A tool for managing various simulation parameters.
"""

import carla
import argparse
import re
import tkinter as tk


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return {name(x): getattr(carla.WeatherParameters, x) for x in presets}


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


def move_spectator_to_actor(spectator, actor):
    actor_transform = actor.get_transform()  # type: carla.Transform
    a_rot = actor_transform.rotation  # type: carla.Rotation
    new_loc = actor_transform.transform(carla.Vector3D(-3, 0, 3))
    new_rot = carla.Rotation(roll=a_rot.roll, pitch=a_rot.pitch-30, yaw=a_rot.yaw)
    spectator.set_transform(carla.Transform(new_loc, new_rot))


def run_manager(host, port):
    # Initialize carla client and get necessary information
    client = carla.Client(host, port)
    client.set_timeout(2.0)
    world = client.get_world()  # type: carla.World
    weather_presets = find_weather_presets()  # returns dict {name: weather_params}
    vehicles = world.get_actors().filter('vehicle.*')  # type: list[carla.Actor]
    vehicles_dict = {veh.attributes["role_name"]: veh for veh in vehicles}
    spectator = world.get_spectator()

    # Create GUI
    root = tk.Tk()
    root.title("Carla Simulation Manager")
    main_frame = tk.Frame(root)
    main_frame.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
    main_frame.pack(padx=10, pady=10)
    # weather preset widget
    tkweather = tk.StringVar(root)
    tkweather.set("Default" if "Default" in weather_presets else next(iter(weather_presets)))
    tk.OptionMenu(main_frame, tkweather, *weather_presets.keys()).grid(row=1, column=2)
    tk.Label(main_frame, text="Weather Presets").grid(row=1, column=1)
    tkweather.trace("w", lambda *args: world.set_weather(weather_presets[tkweather.get()]))
    # ego information and control
    tkego = tk.StringVar(root)
    if "hero" in vehicles_dict:
        tkego.set("hero")
    elif "ego" in vehicles_dict:
        tkego.set("ego")
    tk.Label(main_frame, text="Ego Vehicle").grid(row=2, column=1)
    vehicles_dropdown = tk.OptionMenu(main_frame, tkego, *vehicles_dict.keys() if len(vehicles_dict) else [""])
    vehicles_dropdown.grid(row=2, column=2)
    emergency_stop_button = tk.Button(main_frame, text="Stop Ego",
                                      command=lambda: vehicles_dict[tkego.get()].set_velocity(carla.Vector3D(0, 0, 0))
                                      ).grid(row=3, column=1)
    tr = carla.Transform(carla.Location(0, -1, 3), carla.Rotation(0, 0, 0))
    snap_to_ego = tk.Button(main_frame, text="Snap to Ego",
                            command=lambda: move_spectator_to_actor(spectator, vehicles_dict[tkego.get()])
                            ).grid(row=3, column=2)

    # Start GUI main loop
    root.mainloop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Carla Simulation Manager")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="IP of the host server (default: 127.0.0.1)")
    parser.add_argument("-p", "--port", type=int, default=2000, help="TCP port to listen to (default: 2000)")
    args = parser.parse_args()
    run_manager(args.host, args.port)
