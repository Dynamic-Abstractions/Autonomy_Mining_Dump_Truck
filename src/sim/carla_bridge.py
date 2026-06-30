import carla
import math
import random

class CarlaBridge:

    def __init__(self):

        print("Connecting to CARLA...")

        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(30.0)

        # Load Mine map
        self.world = self.client.load_world('/Game/Carla/Maps/Mine_01')

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)

        print("Mine_01 loaded")

        # Clean old actors (important)
        actors = self.world.get_actors().filter('vehicle.*')
        for act in actors:
            try:
                act.destroy()
            except:
                pass

        # Spawn mining truck
        bp_lib = self.world.get_blueprint_library()

        truck_bp_list = bp_lib.filter('vehicle.miningtruck*')

        if len(truck_bp_list) == 0:
            raise RuntimeError("Mining truck blueprint not found")

        truck_bp = truck_bp_list[0]

        spawn_points = self.world.get_map().get_spawn_points()

        spawn_point = random.choice(spawn_points)

        self.vehicle = self.world.spawn_actor(truck_bp, spawn_point)

        self.vehicle.set_autopilot(False)

        print("Mining truck spawned:", self.vehicle.type_id)

        # Camera (optional but very useful)
        spectator = self.world.get_spectator()
        spectator.set_transform(
            carla.Transform(
                spawn_point.location + carla.Location(z=30, x=-40),
                carla.Rotation(pitch=-30)
            )
        )


    def step(self, steer, throttle, brake):

        steer = float(steer)
        throttle = float(throttle)
        brake = float(brake)

        # Apply control from Simulink
        control = carla.VehicleControl(
            throttle=throttle,
            steer=steer,
            brake=brake
        )

        self.vehicle.apply_control(control)

        self.world.tick()

        # Get state
        tf = self.vehicle.get_transform()
        vel = self.vehicle.get_velocity()

        x = tf.location.x
        y = tf.location.y
        psi = tf.rotation.yaw * math.pi / 180.0

        vx = vel.x

        return [x, y, psi, vx]


    def close(self):

        print("Destroying vehicle...")

        try:
            if self.vehicle:
                self.vehicle.destroy()
        except:
            pass
