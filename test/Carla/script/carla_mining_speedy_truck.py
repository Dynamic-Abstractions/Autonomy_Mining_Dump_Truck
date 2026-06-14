import carla
import pygame
import numpy as np
import math

WIDTH, HEIGHT = 1280, 720


# =========================
# IMAGE
# =========================
def process_img(image, display):
    i = np.frombuffer(image.raw_data, dtype=np.uint8)
    i2 = np.reshape(i, (image.height, image.width, 4))
    i3 = i2[:, :, :3]

    surface = pygame.surfarray.make_surface(i3.swapaxes(0, 1))
    display.blit(surface, (0, 0))


# =========================
# UI
# =========================
def draw_help(display, font):
    help_text = [
        "Controls:",
        "W : Throttle",
        "S : Reverse",
        "A/D : Steering",
        "SPACE : Brake",
        "ESC : Quit"
    ]

    panel = pygame.Surface((260, len(help_text)*22 + 10))
    panel.set_alpha(120)
    panel.fill((0, 0, 0))
    display.blit(panel, (5, 5))

    for i, line in enumerate(help_text):
        display.blit(font.render(line, True, (255,255,255)), (10, 10 + i*20))


def get_speed(vehicle):
    v = vehicle.get_velocity()
    return 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)


def draw_status(display, font, control, vehicle):
    speed = get_speed(vehicle)
    rot = vehicle.get_transform().rotation

    status = [
        f"Throttle: {control.throttle:.2f}",
        f"Steer: {control.steer:.2f}",
        f"Brake: {control.brake:.2f}",
        f"Speed: {speed:.1f} km/h",
        f"Pitch: {rot.pitch:.2f}",
        f"Yaw: {rot.yaw:.2f}",
        f"Roll: {rot.roll:.2f}",
    ]

    for i, line in enumerate(status):
        display.blit(font.render(line, True, (0,255,0)), (WIDTH-260, 10 + i*20))


# =========================
# MAIN
# =========================
def main():
    pygame.init()
    font = pygame.font.SysFont("Arial", 18)
    display = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Mining Truck (Stable + Fast)")

    client = carla.Client('localhost', 2000)
    client.set_timeout(60.0)

    world = client.load_world('Mine_01')
    print("MAP:", world.get_map().name)

    bp_lib = world.get_blueprint_library()

    truck_bp = bp_lib.filter('vehicle.miningtruck.miningtruck')[0]
    spawn_point = world.get_map().get_spawn_points()[0]
    vehicle = world.spawn_actor(truck_bp, spawn_point)

    # =========================
    # CAMERA
    # =========================
    cam_bp = bp_lib.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', str(WIDTH))
    cam_bp.set_attribute('image_size_y', str(HEIGHT))

    cam_transform = carla.Transform(
        carla.Location(x=-25, z=12),
        carla.Rotation(pitch=-20)
    )

    camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)
    camera.listen(lambda image: process_img(image, display))

    try:
        clock = pygame.time.Clock()

        throttle_cmd = 0.0  # smoother control

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return

            keys = pygame.key.get_pressed()
            control = carla.VehicleControl()
            control.manual_gear_shift = False

            speed = get_speed(vehicle)

            # =========================
            # SMART THROTTLE BOOST
            # =========================

            if keys[pygame.K_w]:
                control.throttle = 1.0
                control.reverse = False

                # 🚀 TURBO BOOST (main trick)
                if speed < 60:  # limit boost to avoid instability
                    velocity = vehicle.get_velocity()

                    boost_factor = 1.15  # increase to go faster (1.1–1.3 safe)
                    new_velocity = carla.Vector3D(
                        velocity.x * boost_factor,
                        velocity.y * boost_factor,
                        velocity.z
                    )

                    vehicle.set_target_velocity(new_velocity)


            elif keys[pygame.K_s]:
                control.throttle = 1.0
                control.reverse = True
            else:
                throttle_cmd *= 0.9  # decay
                control.throttle = throttle_cmd

            # Steering
            if keys[pygame.K_a]:
                control.steer = -0.5
            elif keys[pygame.K_d]:
                control.steer = 0.5

            # Brake
            if keys[pygame.K_SPACE]:
                control.brake = 1.0

            vehicle.apply_control(control)

            draw_help(display, font)
            draw_status(display, font, control, vehicle)

            pygame.display.flip()
            clock.tick(60)

    finally:
        print("Cleaning up...")
        camera.destroy()
        vehicle.destroy()
        pygame.quit()


if __name__ == "__main__":
    main()