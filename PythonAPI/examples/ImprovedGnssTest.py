import glob
import os
import sys
import pandas as pd
#sys.path.append('../../Unreal/CarlaUE4/Plugins/Carla/Source/Carla')

from update_rinex_nav import update_rinex_nav

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import random
import time


def main():
    actor_list = []

    try:

        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)

        world = client.get_world()
        blueprint_library = world.get_blueprint_library()

        bp = random.choice(blueprint_library.filter('vehicle'))

        transform = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(bp, transform)

        actor_list.append(vehicle)
        print('created %s' % vehicle.type_id)

        vehicle.set_autopilot(True)

        update_rinex_nav("slac2190.20n",1,time_stamp.elapsed_seconds,pd)

        gnss_bp = blueprint_library.find('sensor.other.improvedgnss')
        transform = carla.Transform(carla.Location(x=0.8, z=1.7))
        gnss = world.spawn_actor(gnss_bp, transform, attach_to=vehicle)
        actor_list.append(gnss)
        print('created %s' % gnss.type_id)

        gnss.listen(callback)

        time.sleep(5)

        time_stamp = carla.timestamp()
        update_rinex_nav("slac2190.20n",1,time_stamp.elapsed_seconds,pd)

        time.sleep(5)

    finally:

        print('destroying actors')
        gnss.destroy()
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print('done.')

def callback(location):

    print('Latitude: %f Longitude: %f Altitude: %f' % (location.latitude, location.longitude, location.altitude))


if __name__ == '__main__':

    main()
