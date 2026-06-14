import carla
import pygame
import numpy as np
import math

WIDTH, HEIGHT = 1280, 720


def main():
    pygame.init()
    display = pygame.display.set_mode((WIDTH, HEIGHT))
    font = pygame.font.SysFont("Consolas", 18, bold=True)
    clock = pygame.time.Clock()

    client = carla.Client('localhost', 2000)
    client.set_timeout(60.0)
    world = client.get_world()

    # 1. Spawn Vehicle & Force Automatic Transmission
    bp_lib = world.get_blueprint_library()
    truck_bp = bp_lib.filter('vehicle.miningtruck.miningtruck')[0]
    spawn_point = world.get_map().get_spawn_points()[0]
    vehicle = world.spawn_actor(truck_bp, spawn_point)

    # Apply Physics for Automatic Gearbox
    physics = vehicle.get_physics_control()
    physics.use_automatic_gear = True
    vehicle.apply_physics_control(physics)

    # 2. Setup High Rear Camera
    cam_bp = bp_lib.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', str(WIDTH))
    cam_bp.set_attribute('image_size_y', str(HEIGHT))
    cam_transform = carla.Transform(carla.Location(x=-28, z=14), carla.Rotation(pitch=-20))
    camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)

    data = {'surface': None}

    def render_view(image):
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))[:, :, :3]
        data['surface'] = pygame.surfarray.make_surface(array[:, :, ::-1].swapaxes(0, 1))

    camera.listen(lambda image: render_view(image))

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT: return

            # 3. Driving Control (WASD)
            keys = pygame.key.get_pressed()
            ctrl = carla.VehicleControl()
            if keys[pygame.K_w]:
                ctrl.throttle = 1.0
                ctrl.reverse = False
            elif keys[pygame.K_s]:
                ctrl.throttle = 1.0
                ctrl.reverse = True
            else:
                ctrl.throttle = 0.0

            ctrl.steer = -0.4 if keys[pygame.K_a] else (0.4 if keys[pygame.K_d] else 0.0)
            ctrl.brake = 1.0 if keys[pygame.K_SPACE] else 0.0
            vehicle.apply_control(ctrl)

            # 4. Draw Background
            if data['surface'] is not None:
                display.blit(data['surface'], (0, 0))

            # 5. Telemetry Logic
            v_physics = vehicle.get_physics_control()
            v_vel = vehicle.get_velocity()
            kmh = 3.6 * math.sqrt(v_vel.x ** 2 + v_vel.y ** 2 + v_vel.z ** 2)

            overlay = pygame.Surface((380, 240))
            overlay.set_alpha(180);
            overlay.fill((0, 0, 0))
            display.blit(overlay, (10, 10))

            lines = [
                f"MINE TRUCK - {vehicle.type_id.upper()}",
                f"TOTAL SPEED: {kmh:.1f} km/h",
                f"CURRENT GEAR: {vehicle.get_control().gear}",
                f"----------------------------",
                f"WHEEL SPEEDS:"
            ]

            for i, wheel in enumerate(v_physics.wheels):
                # Safe attribute grabbing for radius in 0.10.x
                radius = getattr(wheel, 'radius', 100.0)  # Default to 100 if not found
                # Visual proxy for wheel speed
                w_speed = kmh * (1.05 if ctrl.throttle > 0 and kmh < 10 else 1.0)
                name = ["FL", "FR", "RL", "RR"][i]
                lines.append(f" Wheel {name}: {w_speed:.1f} km/h (R: {radius:.1f}cm)")

            for i, line in enumerate(lines):
                display.blit(font.render(line, True, (0, 255, 0)), (20, 20 + (i * 24)))

            pygame.display.flip()
            clock.tick(60)

    finally:
        camera.destroy();
        vehicle.destroy();
        pygame.quit()


if __name__ == "__main__":
    main()
