import carla
import math
import random

class CarlaBridge:

    def __init__(self):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)

        self.world = self.client.get_world()

        # synchronous mode
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)

        # spawn vehicle
        bp = self.world.get_blueprint_library().filter('vehicle.*')[0]
        spawn = random.choice(self.world.get_map().get_spawn_points())

        self.vehicle = self.world.spawn_actor(bp, spawn)

    def step(self, steer):

        self.vehicle.apply_control(
            carla.VehicleControl(
                throttle=0.3,
                steer=float(steer),
                brake=0.0
            )
        )

        self.world.tick()

        tf = self.vehicle.get_transform()
        vel = self.vehicle.get_velocity()

        x = tf.location.x
        y = tf.location.y
        yaw = tf.rotation.yaw * math.pi/180  # rad

        return [x, y, yaw, vel.x]