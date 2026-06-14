# __author__: Julio M (20260610)

import carla
import pygame
import numpy as np

import csv
import math
import time

# File to save data
log_file = open("vehicle_dynamics.csv", "w", newline="")
writer = csv.writer(log_file)

# Headers
writer.writerow([
    "t",
    "vx_body", "vy_body",
    "yaw_rad",
    "yaw_rate",
    "steer"
])


# 1. Configuration
WIDTH, HEIGHT = 1280, 720


def process_img(image, display):
    # Convert raw CARLA BGRA to RGB for PyGame
    i = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    i2 = np.reshape(i, (image.height, image.width, 4))
    i3 = i2[:, :, :3]
    # Rotate and flip to match PyGame's coordinate system
    surface = pygame.surfarray.make_surface(i3.swapaxes(0, 1))
    display.blit(surface, (0, 0))


def draw_help(display, font):
    help_text = [
        "Controls:",
        "W : Throttle forward",
        "S : Reverse",
        "A : Steer left",
        "D : Steer right",
        "SPACE : Brake",
        "ESC : Quit"
    ]

    # Background panel
    panel_width = 250
    panel_height = len(help_text) * 22 + 10
    panel = pygame.Surface((panel_width, panel_height))
    panel.set_alpha(120)  # transparency
    panel.fill((0, 0, 0))

    display.blit(panel, (5, 5))

    for i, line in enumerate(help_text):
        text_surface = font.render(line, True, (255, 255, 255))
        display.blit(text_surface, (10, 10 + i * 20))

def draw_status(display, font, control):
    status = [
        f"Throttle: {control.throttle:.2f}",
        f"Steer: {control.steer:.2f}",
        f"Brake: {control.brake:.2f}"
    ]

    for i, line in enumerate(status):
        text_surface = font.render(line, True, (0, 255, 0))
        display.blit(text_surface, (WIDTH - 180, 10 + i * 20))


def main():
    pygame.init()
    font = pygame.font.SysFont("Arial", 18)
    display = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Synkrotron Mining Truck Controller")

    client = carla.Client('localhost', 2000)
    client.set_timeout(60.0)
    #world = client.get_world()  # Assumes you already loaded Mine_01
    world = client.load_world('Mine_01')
    print("MAP:", world.get_map().name)
    # 2. Find or Spawn Truck
    bp_lib = world.get_blueprint_library()
    truck_bp = bp_lib.filter('vehicle.miningtruck.miningtruck')[0]
    spawn_point = world.get_map().get_spawn_points()[0]
    vehicle = world.spawn_actor(truck_bp, spawn_point)

    # 3. Attach High Rear Camera (Fixed View)
    cam_bp = bp_lib.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', str(WIDTH))
    cam_bp.set_attribute('image_size_y', str(HEIGHT))

    # Position: x=-20 (behind), z=10 (high), pitch=-20 (looking down)
    cam_transform = carla.Transform(carla.Location(x=-25, z=12), carla.Rotation(pitch=-20))
    camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)

    # Listen to camera data
    camera.listen(lambda image: process_img(image, display))
    start_time = time.time()
    prev_yaw = None
    prev_time = None

    try:
        clock = pygame.time.Clock()
        while True:
            # 4. Handle Inputs from PyGame Window
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return

            keys = pygame.key.get_pressed()
            control = carla.VehicleControl()
            control.manual_gear_shift = True
            control.gear = 1  # start in first gear

            gear = control.gear

            # W/S: Throttle and Reverse
            if keys[pygame.K_w]:
                control.throttle = 2
                control.reverse = False
            elif keys[pygame.K_s]:
                control.throttle = 2
                control.reverse = True

            # A/D: Steering
            if keys[pygame.K_a]:
                control.steer = -0.6
            elif keys[pygame.K_d]:
                control.steer = 0.6

            # Space: Brake
            if keys[pygame.K_SPACE]:
                control.brake = 1.0

            # Gear up (.)
            if keys[pygame.K_PERIOD]:
                control.gear += 1
                print(f"Gear up: {control.gear}")

            # Gear down (,)
            if keys[pygame.K_COMMA]:
                control.gear -= 1
                print(f"Gear down: {control.gear}")

            # Optional: Neutral
            if keys[pygame.K_n]:
                control.gear = 0

            # Optional: Reverse
            if keys[pygame.K_r]:
                control.gear = -1

            vehicle.apply_control(control)

            # Update PyGame Window
            draw_help(display, font)
            draw_status(display, font, control)
            pygame.display.flip()
            clock.tick(60)

            # ---------------------------
            # LOG VEHICLE DYNAMICS (FIXED)
            # ---------------------------
            current_time = time.time() - start_time

            transform = vehicle.get_transform()
            velocity = vehicle.get_velocity()

            yaw = math.radians(transform.rotation.yaw)

            # Body frame velocity
            vx_body = math.cos(yaw) * velocity.x + math.sin(yaw) * velocity.y
            vy_body = -math.sin(yaw) * velocity.x + math.cos(yaw) * velocity.y

            angular_velocity = vehicle.get_angular_velocity()
            yaw_rate = math.radians(angular_velocity.z)

            try:
                deg_FL = vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
                deg_FR = vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
                delta_f = math.radians((deg_FL + deg_FR) / 2.0)
            except:
                delta_f = control.steer * 0.5  # fallback

            # Save
            writer.writerow([
                current_time,
                vx_body,
                vy_body,
                yaw,
                yaw_rate,
                delta_f
            ])

            # Update previous
            prev_yaw = yaw
            prev_time = current_time

    # todo: get vehicle dynamics to control and show the pitch, yaw and raw for example

    finally:
        print("Cleaning up assets...")
        camera.destroy()
        vehicle.destroy()
        pygame.quit()
        log_file.close()

if __name__ == "__main__":
    main()
