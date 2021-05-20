"""
Contains class for synchronizing CARLA and SUMO traffic lights.
"""

import matplotlib.pyplot as plt
from matplotlib.patches import CirclePolygon
from matplotlib.lines import Line2D
from matplotlib import transforms
import SumoNetVis
from shapely.geometry import Point
import yaml
import argparse

import carla
import traci

CMAP = plt.get_cmap("Set3")
RADIUS = 3


def tls_sumo_to_carla(sumo_tls_state):
    """Maps a sumo TLS state to a carla one."""
    if sumo_tls_state == "r":
        return carla.TrafficLightState.Red
    elif sumo_tls_state in "uyY":
        return carla.TrafficLightState.Yellow
    elif sumo_tls_state in "gG":
        return carla.TrafficLightState.Green
    else:
        return carla.TrafficLightState.Off


class TLSSync:
    def __init__(self, world, net_file, config_file=None):
        """
        Class for controlling CARLA TrafficLight objects based on SUMO TLS states with a user-defined mapping.

        :param world: carla World object for a currently connected CARLA client
        :type world: carla.World
        :param net_file: path to the SUMO net file
        :type net_file: str
        :param config_file: path to YAML config file containing mappings from CARLA TrafficLight to SUMO TLS
        :type config_file: str
        """
        self.world = world
        self.net_file = net_file
        self.net = SumoNetVis.Net(self.net_file)
        self.carla_tls = [actor for actor in world.get_actors() if actor.type_id == "traffic.traffic_light"]
        self._nearest_junctions = []
        self._associate_to_junctions()
        if config_file is None:
            self.config = None
        else:
            with open(config_file, "r") as f:
                self.config = yaml.load(f, Loader=yaml.Loader)
                self.config = {str(k): self.config[k] for k in self.config}

    def _associate_to_junctions(self):
        """Find the nearest junction for each CARLA TrafficLight object."""
        self._nearest_junctions = []
        ox, oy = self.net.netOffset
        for tl in self.carla_tls:
            loc = tl.get_transform().location
            x, y = loc.x+ox, -loc.y+oy
            best_jn, best_dist = None, 100000
            for junction in self.net.junctions:
                jn = self.net.junctions[junction]
                if jn.shape is None:
                    continue
                dist = jn.shape.distance(Point((x, y)))
                if dist < best_dist:
                    best_jn = junction
                    best_dist = dist
                if best_dist <= 0:
                    break
            self._nearest_junctions.append(best_jn)

    def tick(self):
        """Update CARLA TLS states based on SUMO TLS states. Should be run every simulation step."""
        sumo_states = {jn: traci.trafficlight.getRedYellowGreenState(jn) for jn in set(self._nearest_junctions)}
        for tl, junction in zip(self.carla_tls, self._nearest_junctions):  # type: carla.TrafficLight, str
            pole_index = tl.get_pole_index()
            if junction in self.config and pole_index in self.config[junction]:
                sumo_state = sumo_states[junction][self.config[junction][pole_index]]
                carla_state = tls_sumo_to_carla(sumo_state)
                tl.freeze(True)
                tl.set_state(carla_state)

    def plot(self, ax=None):
        """Plot the SUMO net, all CARLA TrafficLight objects in the current world, and any configured TLS links."""
        if ax is None:
            ax = plt.gca()
        self.net.plot(ax, apply_netOffset=True, alpha=0.4)
        xoff, yoff = self.net.netOffset
        tr = transforms.Affine2D().translate(-xoff, -yoff) + ax.transData
        ox, oy = self.net.netOffset
        for tl, junction in zip(self.carla_tls, self._nearest_junctions):
            loc = tl.get_transform().location
            pole_index = tl.get_pole_index()
            centroid_x, centroid_y = self.net.junctions[junction].shape.centroid.xy
            centroid_x, centroid_y = centroid_x[0], centroid_y[0]
            color = CMAP(pole_index)
            line = Line2D([loc.x, centroid_x-ox], [-loc.y, centroid_y-oy], color=color, dashes=(1, 1))
            ax.add_line(line)
            patch = CirclePolygon((loc.x, -loc.y), RADIUS, alpha=0.9, color=color, ec="black", linewidth=1)
            ax.annotate(str(pole_index), (1.05, 1.05), xycoords=patch)
            ax.add_patch(patch)
            if self.config is not None and junction in self.config:
                if pole_index in self.config[junction]:
                    ln_id = self.config[junction][pole_index]
                    ln = self.net.junctions[junction].intLanes[ln_id]
                    ln.plot_shape(ax, color=color, transform=tr, alpha=0.9)
        ax.relim()
        ax.autoscale()
        ax.set_aspect("equal")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--carla_host", type=str, default="localhost")
    parser.add_argument("--carla_port", type=int, default=2000)
    parser.add_argument("--net", type=str, required=True, help="Path to SUMO net file.")
    parser.add_argument("--config", type=str, required=True, help="Path to TLS YAML config file.")
    parser.add_argument("--traci_port", type=int, default=8813)
    parser.add_argument("--traci_client_order", type=int, default=99)
    args = parser.parse_args()
    client = carla.Client(args.carla_host, args.carla_port)
    # client.load_world("Town04")
    tls_sync = TLSSync(client.get_world(), args.net, config_file=args.config)
    if args.run:
        traci.init(port=args.traci_port, label="Carla_TLSSync")
        cxn = traci.getConnection("Carla_TLSSync")
        cxn.setOrder(args.traci_client_order)
        while True:
            tls_sync.tick()
            traci.simulationStep()
    else:
        tls_sync.plot()
        plt.show()
