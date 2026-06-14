import carla
import pygame
import numpy as np
import math
from collections import deque

# =========================
# CONFIG
# =========================
WIDTH, HEIGHT = 1280, 720
FPS = 60

# Plot settings
PLOT_WIDTH = 340
PLOT_HEIGHT = 180
PLOT_MARGIN = 10
PLOT_BG_ALPHA = 140
PLOT_HISTORY = 220  # number of samples shown


# =========================
# IMAGE CONVERSION
# =========================
latest_surface = None


def camera_callback(image):
    global latest_surface
    arr = np.frombuffer(image.raw_data, dtype=np.uint8)
    arr = np.reshape(arr, (image.height, image.width, 4))
    rgb = arr[:, :, :3][:, :, ::-1]  # BGRA -> RGB
    latest_surface = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))


# =========================
# UTILS
# =========================
def get_speed(vehicle):
    v = vehicle.get_velocity()
    return 3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)


def wrap_angle_deg(angle):
    """Wrap angle to [-180, 180]."""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


# =========================
# LIVE ORIENTATION PLOT
# =========================
class OrientationPlot:
    def __init__(self, x, y, width, height, history=200):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

        self.pitch_hist = deque(maxlen=history)
        self.yaw_hist = deque(maxlen=history)
        self.roll_hist = deque(maxlen=history)

        # colors
        self.pitch_color = (255, 80, 80)   # red
        self.yaw_color = (80, 255, 80)     # green
        self.roll_color = (80, 180, 255)   # blue
        self.axis_color = (180, 180, 180)
        self.text_color = (255, 255, 255)
        self.grid_color = (70, 70, 70)

    def update(self, pitch, yaw, roll):
        self.pitch_hist.append(pitch)
        self.yaw_hist.append(wrap_angle_deg(yaw))
        self.roll_hist.append(roll)

    def _draw_single_plot(self, display, font, title, values, color, rect, y_min, y_max):
        x, y, w, h = rect

        # background
        panel = pygame.Surface((w, h), pygame.SRCALPHA)
        panel.fill((0, 0, 0, PLOT_BG_ALPHA))
        display.blit(panel, (x, y))

        # border
        pygame.draw.rect(display, (150, 150, 150), (x, y, w, h), 1)

        # zero line if visible
        if y_min < 0 < y_max:
            zero_y = y + h - int((0 - y_min) / (y_max - y_min) * h)
            pygame.draw.line(display, self.axis_color, (x, zero_y), (x + w, zero_y), 1)

        # light grid
        for i in range(1, 4):
            gy = y + int(i * h / 4)
            pygame.draw.line(display, self.grid_color, (x, gy), (x + w, gy), 1)

        # title
        title_surf = font.render(f"{title} [{y_min:.0f},{y_max:.0f}] deg", True, self.text_color)
        display.blit(title_surf, (x + 6, y + 4))

        # line
        if len(values) < 2:
            return

        pts = []
        n = len(values)
        for i, v in enumerate(values):
            px = x + int(i * (w - 1) / max(n - 1, 1))
            # clamp just in case
            v = max(min(v, y_max), y_min)
            py = y + h - int((v - y_min) / (y_max - y_min) * h)
            pts.append((px, py))

        if len(pts) >= 2:
            pygame.draw.lines(display, color, False, pts, 2)

        # latest value
        last_val = values[-1]
        value_surf = font.render(f"{last_val:7.2f}", True, color)
        display.blit(value_surf, (x + w - 95, y + 4))

    def draw(self, display, font):
        # 3 stacked plots
        section_h = self.height // 3
        pitch_rect = (self.x, self.y, self.width, section_h - 4)
        yaw_rect   = (self.x, self.y + section_h, self.width, section_h - 4)
        roll_rect  = (self.x, self.y + 2 * section_h, self.width, section_h - 4)

        # fixed scales make plots stable and easy to read
        self._draw_single_plot(display, font, "Pitch", list(self.pitch_hist), self.pitch_color, pitch_rect, -45, 45)
        self._draw_single_plot(display, font, "Yaw",   list(self.yaw_hist),   self.yaw_color,   yaw_rect,   -180, 180)
        self._draw_single_plot(display, font, "Roll",  list(self.roll_hist),  self.roll_color,  roll_rect,  -45, 45)

        # legend under/near plot
        legend_y = self.y + self.height + 4
        pitch_txt = font.render("Pitch", True, self.pitch_color)
        yaw_txt   = font.render("Yaw", True, self.yaw_color)
        roll_txt  = font.render("Roll", True, self.roll_color)

        display.blit(pitch_txt, (self.x, legend_y))
        display.blit(yaw_txt, (self.x + 70, legend_y))
        display.blit(roll_txt, (self.x + 130, legend_y))


# =========================
# UI
# =========================
def draw_help(display, font):
    lines = [
        "Controls:",
        "W : Throttle / turbo",
        "S : Reverse",
        "A / D : Steer",
        "SPACE : Brake",
        "C : Clear plots",
        "ESC : Quit"
    ]

    panel_w = 250
    panel_h = len(lines) * 22 + 10
    panel = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
    panel.fill((0, 0, 0, 120))
    display.blit(panel, (10, 10))

    for i, line in enumerate(lines):
        text_surface = font.render(line, True, (255, 255, 255))
        display.blit(text_surface, (16, 16 + i * 20))


def draw_status(display, font, control, vehicle):
    speed = get_speed(vehicle)
    rot = vehicle.get_transform().rotation

    lines = [
        f"Throttle: {control.throttle:.2f}",
        f"Steer   : {control.steer:.2f}",
        f"Brake   : {control.brake:.2f}",
        f"Speed   : {speed:.1f} km/h",
        f"Pitch   : {rot.pitch:.2f} deg",
        f"Yaw     : {rot.yaw:.2f} deg",
        f"Roll    : {rot.roll:.2f} deg",
    ]

    panel_w = 250
    panel_h = len(lines) * 22 + 10
    x = 10
    y = HEIGHT - panel_h - 10

    panel = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
    panel.fill((0, 0, 0, 120))
    display.blit(panel, (x, y))

    for i, line in enumerate(lines):
        text_surface = font.render(line, True, (0, 255, 0))
        display.blit(text_surface, (x + 8, y + 8 + i * 20))


# =========================
# MAIN
# =========================
def main():
    global latest_surface

    pygame.init()
    pygame.font.init()

    display = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("CARLA Mining Truck - Live Orientation Plot")
    font = pygame.font.SysFont("Consolas", 18)
    small_font = pygame.font.SysFont("Consolas", 15)

    client = carla.Client("localhost", 2000)
    client.set_timeout(60.0)

    # If you already loaded the map manually, you can change this to:
    # world = client.get_world()
    world = client.load_world("Mine_01")
    print("MAP:", world.get_map().name)

    bp_lib = world.get_blueprint_library()

    # Truck
    truck_bp = bp_lib.filter("vehicle.miningtruck.miningtruck")[0]
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No spawn points found in current map.")

    spawn_point = spawn_points[0]
    vehicle = world.spawn_actor(truck_bp, spawn_point)

    # Camera
    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(WIDTH))
    cam_bp.set_attribute("image_size_y", str(HEIGHT))
    cam_bp.set_attribute("fov", "90")

    cam_transform = carla.Transform(
        carla.Location(x=-25, z=12),
        carla.Rotation(pitch=-20)
    )
    camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)
    camera.listen(camera_callback)

    # Plot panel on the right
    plot = OrientationPlot(
        x=WIDTH - PLOT_WIDTH - PLOT_MARGIN,
        y=10,
        width=PLOT_WIDTH,
        height=PLOT_HEIGHT,
        history=PLOT_HISTORY
    )

    clock = pygame.time.Clock()

    # smooth throttle state
    throttle_cmd = 0.0
    running = True

    try:
        while running:
            clock.tick(FPS)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_c:
                        plot.pitch_hist.clear()
                        plot.yaw_hist.clear()
                        plot.roll_hist.clear()

            keys = pygame.key.get_pressed()

            # Create control every loop
            control = carla.VehicleControl()
            control.manual_gear_shift = False

            speed = get_speed(vehicle)

            # Forward / turbo assist
            if keys[pygame.K_w]:
                throttle_cmd += 0.05
                if speed < 20:
                    throttle_cmd += 0.05
                throttle_cmd = min(throttle_cmd, 1.0)

                control.throttle = throttle_cmd
                control.reverse = False

                # extra boost without using unsupported physics APIs
                if speed < 90:
                    vel = vehicle.get_velocity()
                    boost_factor = 1.03 if speed > 40 else 1.06
                    boosted = carla.Vector3D(
                        vel.x * boost_factor,
                        vel.y * boost_factor,
                        vel.z
                    )
                    vehicle.set_target_velocity(boosted)

            elif keys[pygame.K_s]:
                throttle_cmd = 0.6
                control.throttle = 1.0
                control.reverse = True
            else:
                throttle_cmd *= 0.90
                if throttle_cmd < 0.01:
                    throttle_cmd = 0.0
                control.throttle = throttle_cmd

            # Steering
            if keys[pygame.K_a]:
                control.steer = -0.5
            elif keys[pygame.K_d]:
                control.steer = 0.5
            else:
                control.steer = 0.0

            # Brake
            if keys[pygame.K_SPACE]:
                control.brake = 1.0
                throttle_cmd = 0.0
            else:
                control.brake = 0.0

            vehicle.apply_control(control)

            # Get orientation and update plots
            rot = vehicle.get_transform().rotation
            plot.update(rot.pitch, rot.yaw, rot.roll)

            # Draw background/frame
            if latest_surface is not None:
                display.blit(latest_surface, (0, 0))
            else:
                display.fill((30, 30, 30))

            # Draw overlays
            draw_help(display, font)
            draw_status(display, font, control, vehicle)
            plot.draw(display, small_font)

            pygame.display.flip()

    finally:
        print("Cleaning up...")
        try:
            camera.stop()
        except Exception:
            pass
        camera.destroy()
        vehicle.destroy()
        pygame.quit()


if __name__ == "__main__":
    main()