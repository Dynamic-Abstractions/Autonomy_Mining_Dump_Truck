import carla
import random
import time


def run_mining_simulation():
    # 1. Connect and Load Map
    client = carla.Client('localhost', 2000)
    client.set_timeout(60.0) # Give the heavy UE5 map time to load
    world = client.load_world('/Game/Carla/Maps/Mine_01')
    time.sleep(5)  # Wait for the map to fully settle

    # 2. Set Environment
    world.set_weather(carla.WeatherParameters.ClearNoon)

    # 3. Spawn Vehicle
    bp_lib = world.get_blueprint_library()
    truck_bp = bp_lib.filter('vehicle.miningtruck.miningtruck')[0]
    spawn_points = world.get_map().get_spawn_points()
    spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
    vehicle = world.spawn_actor(truck_bp, spawn_point)

    # 4. Set Spectator Camera
    spec = world.get_spectator()
    spec.set_transform(carla.Transform(vehicle.get_location() + carla.Location(z=30, x=-40), carla.Rotation(pitch=-30)))

    vehicle.set_autopilot(True)
    print(f"Spawned {vehicle.type_id} at {spawn_point.location}")


if __name__ == "__main__":
    run_mining_simulation()
