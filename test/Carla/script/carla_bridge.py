import carla
import math


class CarlaBridge:

    def __init__(self):

        self.vehicle = None  # ensure close() is always safe

        print("Connecting to CARLA...")

        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(30.0)

        self.world = self.client.load_world('/Game/Carla/Maps/Mine_01')

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.01
        self.world.apply_settings(settings)

        print("Mine_01 loaded")

        # --------------------------------------------------
        # Destroy existing vehicles
        # --------------------------------------------------

        actors = self.world.get_actors().filter('vehicle.*')

        for act in actors:
            try:
                act.destroy()
            except:
                pass

        # --------------------------------------------------
        # Get mining truck blueprint
        # --------------------------------------------------

        bp_lib = self.world.get_blueprint_library()
        truck_bp_list = bp_lib.filter('vehicle.miningtruck*')
        if len(truck_bp_list) == 0:
            raise RuntimeError(
                "Mining truck blueprint not found"
            )

        truck_bp = truck_bp_list[0]
        desired_location = carla.Location(
            x=-11.57,
            y=95.74,
            z=5.0
        )
        spawn_wp = self.world.get_map().get_waypoint(
            desired_location,
            project_to_road=True
        )
        spawn_transform = spawn_wp.transform
        # No yaw flip — vehicle faces road travel direction (west ≈ 181°)
        # lift truck slightly above road
        spawn_transform.location.z += 3.0
        yaw_rad = math.radians(
            spawn_transform.rotation.yaw
        )
        # offset = 0: spawn at road centreline so initial lateral error is 0
        offset = 0
        spawn_transform.location.x += (
                -offset * math.sin(yaw_rad)
        )
        spawn_transform.location.y += (
                offset * math.cos(yaw_rad)
        )

        print(
            f"Spawning at road waypoint:"
            f" x={spawn_transform.location.x:.2f}"
            f" y={spawn_transform.location.y:.2f}"
            f" z={spawn_transform.location.z:.2f}"
            f" yaw={spawn_transform.rotation.yaw:.2f}"
        )

        self.vehicle = self.world.try_spawn_actor(
            truck_bp,
            spawn_transform
        )

        if self.vehicle is None:
            raise RuntimeError(
                "Failed to spawn vehicle at desired location"
            )

        self.spawn_transform = spawn_transform

        # Store map-frame origin so all returned positions start at (0, 0)
        self.origin_x = spawn_transform.location.x
        self.origin_y = spawn_transform.location.y

        # --------------------------------------------------
        # Manual control only
        # --------------------------------------------------

        self.vehicle.set_autopilot(False)

        print(
            "Mining truck spawned:",
            self.vehicle.type_id
        )

        print(
            f"Spawn location: "
            f"{self.spawn_transform.location.x:.2f}, "
            f"{self.spawn_transform.location.y:.2f}, "
            f"{self.spawn_transform.location.z:.2f}"
        )

        self.world.tick()

        # --------------------------------------------------
        # Spectator
        # --------------------------------------------------

        spectator = self.world.get_spectator()

        spectator.set_transform(
            carla.Transform(
                self.spawn_transform.location +
                carla.Location(
                    x=-40,
                    z=30
                ),
                carla.Rotation(
                    pitch=-30
                )
            )
        )

        # --------------------------------------------------
        # BUILD REFERENCE PATH ONCE
        # --------------------------------------------------

        self.reference_path = []

        start_wp = self.world.get_map().get_waypoint(
            self.vehicle.get_location(),
            project_to_road=True
        )

        wp = start_wp

        for _ in range(500):

            self.reference_path.append(wp)

            nxt = wp.next(2.0)

            if len(nxt) == 0:
                break

            wp = nxt[0]

        print(
            f"Reference path size: "
            f"{len(self.reference_path)}"
        )

        # --------------------------------------------------
        # Planned path 1→2→3 using Road 5 (haul road) from topology
        # Road 5 Lane 1: (30.2, 85.2, -64.0) → (220.3, 19.2, -29.0)
        # --------------------------------------------------

        topology = self.world.get_map().get_topology()

        # Get Road 5 start waypoint directly from topology
        road5_start_wp = None
        for seg_start, seg_end in topology:
            if seg_start.road_id == 5 and seg_start.lane_id == 1:
                road5_start_wp = seg_start
                break

        # Walk next() along Road 5 to collect all waypoints
        self.haul_road_wps = []
        arc_s = 0.0
        marker_wp = road5_start_wp if road5_start_wp else start_wp

        if road5_start_wp:
            fwd_wp = road5_start_wp
            self.haul_road_wps.append(fwd_wp)
            for _ in range(5000):
                nxt_list = fwd_wp.next(2.0)
                if not nxt_list:
                    break
                nxt = nxt_list[0]
                if nxt.road_id != 5:
                    break
                l1 = fwd_wp.transform.location
                l2 = nxt.transform.location
                arc_s += math.sqrt((l2.x - l1.x) ** 2 + (l2.y - l1.y) ** 2)
                fwd_wp = nxt
                self.haul_road_wps.append(fwd_wp)
            marker_wp = fwd_wp

        print(f"Road 5 haul path: {len(self.haul_road_wps)} wps, "
              f"arc={arc_s:.1f} m")

        # ---- Full trajectory: Road 5 start → chain next() through all roads ----
        # Road 5 Lane 1 is the confirmed forward haul road (next() works).
        # Not filtering by road_id so next() chains naturally through junctions.
        road5_start_wp = None
        for seg_start, seg_end in topology:
            if seg_start.road_id == 5 and seg_start.lane_id == 1:
                road5_start_wp = seg_start
                break

        self.trajectory_wps = []
        traj_arc = 0.0
        traj_end_wp = road5_start_wp if road5_start_wp else start_wp

        if road5_start_wp:
            fwd_wp = road5_start_wp
            self.trajectory_wps.append(fwd_wp)
            traj_start_loc = road5_start_wp.transform.location
            for _ in range(10000):
                nxt_list = fwd_wp.next(2.0)
                if not nxt_list:
                    break
                nxt = nxt_list[0]
                l1 = fwd_wp.transform.location
                l2 = nxt.transform.location
                traj_arc += math.sqrt((l2.x - l1.x) ** 2 + (l2.y - l1.y) ** 2)
                fwd_wp = nxt
                self.trajectory_wps.append(fwd_wp)
                # Stop if we've looped back near the start
                if len(self.trajectory_wps) > 300:
                    d = math.sqrt((l2.x - traj_start_loc.x) ** 2 +
                                  (l2.y - traj_start_loc.y) ** 2)
                    if d < 30.0:
                        break
            traj_end_wp = fwd_wp

        print(f"Full trajectory: {len(self.trajectory_wps)} wps, "
              f"arc={traj_arc:.1f} m")

        # Pre-compute cumulative arc-lengths for reference generation
        self.traj_arc_s = [0.0]
        for i in range(1, len(self.trajectory_wps)):
            l1 = self.trajectory_wps[i-1].transform.location
            l2 = self.trajectory_wps[i].transform.location
            self.traj_arc_s.append(
                self.traj_arc_s[-1] + math.sqrt(
                    (l2.x-l1.x)**2 + (l2.y-l1.y)**2
                )
            )
        self.traj_total_s = self.traj_arc_s[-1]

        # ========================================================
        # REFERENCE PATH — currently: 50 m straight line from spawn
        # Complex path (U-turn + connector + haul road) is below,
        # commented out.  Uncomment and swap once straight-line
        # tracking is verified.
        # ========================================================

        LINE_LEN  = 500.0   # straight-line length [m]
        LINE_STEP =  2.0   # point spacing [m]
        sx   = self.spawn_transform.location.x
        sy   = self.spawn_transform.location.y
        sz   = self.spawn_transform.location.z + 1.5
        syaw = math.radians(self.spawn_transform.rotation.yaw)

        self.full_path_pts = []
        n_pts = int(LINE_LEN / LINE_STEP) + 1
        for i in range(n_pts):
            s = i * LINE_STEP
            self.full_path_pts.append((
                sx + s * math.cos(syaw),
                sy + s * math.sin(syaw),
                sz
            ))

        # Cumulative arc-lengths
        self.full_path_s = [0.0]
        for i in range(1, len(self.full_path_pts)):
            p1, p2 = self.full_path_pts[i-1], self.full_path_pts[i]
            self.full_path_s.append(
                self.full_path_s[-1] + math.sqrt(
                    (p2[0]-p1[0])**2 + (p2[1]-p1[1])**2
                )
            )
        self.full_path_total_s = self.full_path_s[-1]

        # Headings: constant = spawn heading along the straight line
        self.full_path_psi = [syaw] * len(self.full_path_pts)

        # Persistent closest-index
        self._fp_idx = 0

        print(f"Reference path: straight line {LINE_LEN:.0f} m, "
              f"{len(self.full_path_pts)} pts, heading {math.degrees(syaw):.1f}°")

        # Draw straight-line reference (cyan)
        ln_col = carla.Color(0, 220, 255)
        for i in range(len(self.full_path_pts) - 1):
            p1, p2 = self.full_path_pts[i], self.full_path_pts[i+1]
            self.world.debug.draw_line(
                carla.Location(x=p1[0], y=p1[1], z=p1[2]),
                carla.Location(x=p2[0], y=p2[1], z=p2[2]),
                thickness=0.5, color=ln_col, life_time=0.0
            )

        # --------------------------------------------------------
        # COMPLEX PATH — uncomment below when ready
        # --------------------------------------------------------
        # spawn_road_wp = self.world.get_map().get_waypoint(
        #     self.vehicle.get_location(), project_to_road=True)
        #
        # # Forward section (30 m along road before U-turn)
        # fwd_wps = []
        # cur_wp = spawn_road_wp
        # for _ in range(15):
        #     nxt = cur_wp.next(2.0)
        #     if not nxt: break
        #     cur_wp = nxt[0]; fwd_wps.append(cur_wp)
        #
        # # U-turn arc (R=25 m, 180° clockwise)
        # tp = fwd_wps[-1].transform if fwd_wps else self.spawn_transform
        # tx, ty, tz = tp.location.x, tp.location.y, tp.location.z + 1.5
        # tyaw = math.radians(tp.rotation.yaw)
        # cx = tx + 25.0 * math.cos(tyaw - math.pi/2)
        # cy = ty + 25.0 * math.sin(tyaw - math.pi/2)
        # a0 = math.atan2(ty - cy, tx - cx)
        # u_pts = [(cx + 25.0*math.cos(a0 - i/30*math.pi),
        #           cy + 25.0*math.sin(a0 - i/30*math.pi), tz)
        #          for i in range(1, 31)]
        #
        # # Connector (backward from spawn to Road 5)
        # connector_wps = []
        # cur_wp = spawn_road_wp
        # for _ in range(200):
        #     prev = cur_wp.previous(2.0)
        #     if not prev: break
        #     cur_wp = prev[0]; connector_wps.append(cur_wp)
        #     if cur_wp.road_id == 5: break
        #
        # # Assemble full path
        # self.full_path_pts = (
        #     [(w.transform.location.x, w.transform.location.y,
        #       w.transform.location.z + 1.5) for w in fwd_wps] +
        #     list(u_pts) +
        #     [(w.transform.location.x, w.transform.location.y,
        #       w.transform.location.z + 1.5) for w in connector_wps] +
        #     [(w.transform.location.x, w.transform.location.y,
        #       w.transform.location.z + 1.5) for w in self.trajectory_wps]
        # )
        # ... rebuild full_path_s and full_path_psi after swapping ...
        # --------------------------------------------------------

        self.world.tick()

        # Record CARLA sim time at bridge start so that
        # reference signals start at t=0 regardless of server uptime.
        self.t0_carla = (
            self.world.get_snapshot().timestamp.elapsed_seconds
        )
        # Local simulation timer: incremented by fixed_delta_seconds each
        # step() call.  Avoids dependence on CARLA server uptime.
        self.sim_time = 0.0
        self._dt = 0.01  # must match settings.fixed_delta_seconds

        # Accumulators for continuous (unwrapped) psi and psi_d
        self._psi_unwrap    = None
        self._psi_d_unwrap  = None

        self._psi0 = None   # initial heading for relative-angle output

        # Reference frame origin captured at first step() so that
        # Xd(0)=0, Yd(0)=0, psi_d(0)=vehicle heading (zero initial error).
        self._ref_x0 = None
        self._ref_y0 = None
        # Frozen Xd/Yd after lane-change completes, prevents reference drift
        # caused by the look-ahead y-coordinate slowly changing with arc length.
        self._Xd_frozen = None
        self._Yd_frozen = None

        # Data log: list of dicts, written to CSV on close()
        self._log = []
        self._log_path = (
            r'G:\Control_Research\Autonomy_Mining_Dump_Truck\bridge_log.csv'
        )

    # =====================================================
    # BUILD LANE-CHANGE TRAJECTORY
    # =====================================================

    def _build_lane_change_trajectory(self, wps, lane_width=6.0, n_changes=4):
        """
        Apply n_changes cubic-smoothstep lane changes distributed evenly
        along the trajectory waypoints.  Odd maneuvers shift left of the
        vehicle, even ones shift back (alternating).

        Returns list of dicts {x, y, z}.
        """
        if len(wps) < 2:
            return []

        # Cumulative arc-length
        arc_s = [0.0]
        for i in range(1, len(wps)):
            l1 = wps[i - 1].transform.location
            l2 = wps[i].transform.location
            arc_s.append(arc_s[-1] + math.sqrt(
                (l2.x - l1.x) ** 2 + (l2.y - l1.y) ** 2
            ))
        total_s = arc_s[-1]

        # Distribute n_changes evenly; each spans 8 % of total length
        lc_length = total_s * 0.08
        spacing   = total_s / (n_changes + 1)
        lane_changes = []
        cumulative_offset = 0.0
        for k in range(n_changes):
            start_s = spacing * (k + 1) - lc_length / 2
            delta   = lane_width if k % 2 == 0 else -lane_width
            lane_changes.append((start_s, lc_length, delta))
            cumulative_offset += delta

        def smoothstep(tau):
            return 3 * tau * tau - 2 * tau * tau * tau

        def lateral_offset(s):
            d = 0.0
            for lc_start, lc_len, lc_delta in lane_changes:
                lc_end = lc_start + lc_len
                if s < lc_start:
                    pass
                elif s < lc_end:
                    tau = (s - lc_start) / lc_len
                    d += lc_delta * smoothstep(tau)
                else:
                    d += lc_delta
            return d

        path = []
        for i, wp in enumerate(wps):
            s   = arc_s[i]
            loc = wp.transform.location
            # Vehicle faces opposite to waypoint yaw, so
            # vehicle-left = +sin(yaw), -cos(yaw)
            yaw = math.radians(wp.transform.rotation.yaw)
            perp_x =  math.sin(yaw)
            perp_y = -math.cos(yaw)
            d = lateral_offset(s)
            path.append({
                'x': loc.x + d * perp_x,
                'y': loc.y + d * perp_y,
                'z': loc.z
            })

        return path

    # =====================================================
    # DRAW MAP TOPOLOGY
    # =====================================================

    def _draw_map_topology(self):
        """
        Draw every road segment from the CARLA map topology:
          - Yellow box  = segment start waypoint
          - Cyan box    = segment end waypoint
          - White line  = segment path
          - White text  = "R{road_id} L{lane_id}" label at midpoint
        All drawn permanently (life_time=0). Prints a table to terminal.
        """
        topology = self.world.get_map().get_topology()
        print(f"\n=== MAP TOPOLOGY ({len(topology)} segments) ===")
        print(f"{'Seg':>4}  {'Road':>4}  {'Lane':>5}  "
              f"{'Start (x,y,z)':^28}  {'End (x,y,z)':^28}")

        # Assign a unique colour per road_id for easy visual grouping
        road_colours = {}
        palette = [
            carla.Color(255, 200,   0),   # yellow
            carla.Color(  0, 220, 255),   # cyan
            carla.Color(255,  80, 200),   # magenta
            carla.Color( 80, 255,  80),   # green
            carla.Color(255, 140,   0),   # orange
            carla.Color(180,   0, 255),   # purple
            carla.Color(  0, 180, 255),   # sky blue
            carla.Color(255,  50,  50),   # red
        ]

        for idx, (seg_start, seg_end) in enumerate(topology):
            r_id = seg_start.road_id
            l_id = seg_start.lane_id
            colour = road_colours.setdefault(r_id, palette[r_id % len(palette)])

            s = seg_start.transform.location
            e = seg_end.transform.location

            # Start box (smaller)
            self.world.debug.draw_box(
                carla.BoundingBox(
                    carla.Location(x=s.x, y=s.y, z=s.z + 2.0),
                    carla.Vector3D(0.5, 0.5, 0.2)
                ),
                seg_start.transform.rotation,
                thickness=0.15,
                color=colour,
                life_time=0.0
            )
            # End box (smaller)
            self.world.debug.draw_box(
                carla.BoundingBox(
                    carla.Location(x=e.x, y=e.y, z=e.z + 2.0),
                    carla.Vector3D(0.3, 0.3, 0.15)
                ),
                seg_end.transform.rotation,
                thickness=0.12,
                color=colour,
                life_time=0.0
            )
            # Label at midpoint
            mx = (s.x + e.x) / 2.0
            my = (s.y + e.y) / 2.0
            mz = (s.z + e.z) / 2.0 + 4.0
            self.world.debug.draw_string(
                carla.Location(x=mx, y=my, z=mz),
                f"R{r_id} L{l_id}",
                draw_shadow=True,
                color=carla.Color(255, 255, 255),
                life_time=0.0
            )
            # Terminal log
            print(f"{idx:>4}  {r_id:>4}  {l_id:>5}  "
                  f"({s.x:7.1f},{s.y:7.1f},{s.z:6.1f})  "
                  f"({e.x:7.1f},{e.y:7.1f},{e.z:6.1f})")

        print("=== END TOPOLOGY ===\n")

    # =====================================================
    # ANALYTICAL LANE-CHANGE REFERENCE
    # =====================================================

    def _lane_change_reference(self, sim_time, vx_safe, x_raw, y_raw, z_raw,
                                psi_current=None):
        """
        Closest-point + look-ahead reference on the full path
        (U-turn arc → Road-4 connector → haul-road trajectory).

        The reference never runs away from the vehicle: it always starts
        from the closest point on the path and advances LOOK_AHEAD metres.

        Returns (Xd, Yd, psi_d, psi_dot_d, psi_dd_d).
        """
        LOOK_AHEAD = 20.0
        CURV_STEP  =  5.0   # metres for curvature estimate
        vx_ref  = 10.0
        yf      = 0.0   # 0 = straight-line only; set to 6.0 to enable lane change
        t_start = 15.0
        T       = 10.0

        pts = self.full_path_pts
        arc = self.full_path_s
        N   = len(pts)

        # ---- Find closest point (search forward from last index) ----
        search_start = max(0, self._fp_idx - 5)
        search_end   = min(N - 1, self._fp_idx + 60)
        best_d2, best_i = 1e18, self._fp_idx
        for i in range(search_start, search_end + 1):
            dx = pts[i][0] - x_raw
            dy = pts[i][1] - y_raw
            d2 = dx*dx + dy*dy
            if d2 < best_d2:
                best_d2 = d2
                best_i  = i
        self._fp_idx = best_i

        # ---- Look-ahead point ----
        s_close  = arc[best_i]
        s_target = s_close + LOOK_AHEAD
        la_idx   = best_i
        for i in range(best_i, N):
            if arc[i] >= s_target:
                la_idx = i
                break
        else:
            la_idx = N - 1

        ref_x = pts[la_idx][0]
        ref_y = pts[la_idx][1]
        psi_d = self.full_path_psi[la_idx]

        # ---- Unwrap psi_d ----
        if self._psi_d_unwrap is None:
            self._psi_d_unwrap = psi_current if psi_current is not None else psi_d
        else:
            delta = math.atan2(math.sin(psi_d - self._psi_d_unwrap),
                               math.cos(psi_d - self._psi_d_unwrap))
            self._psi_d_unwrap += delta
        psi_d = self._psi_d_unwrap

        # ---- Lane-change lateral offset ----
        if sim_time < t_start:
            d_lat = 0.0; d_lat1 = 0.0; d_lat2 = 0.0; d_lat3 = 0.0
        elif sim_time <= t_start + T:
            tau    = (sim_time - t_start) / T
            d_lat  = yf * (3*tau**2 - 2*tau**3)
            d_lat1 = yf * (6*tau  - 6*tau**2)  / T
            d_lat2 = yf * (6.0 - 12.0*tau)     / T**2
            d_lat3 = -12.0 * yf / T**3
        else:
            d_lat = yf; d_lat1 = 0.0; d_lat2 = 0.0; d_lat3 = 0.0

        perp_x = math.sin(psi_d)
        perp_y = -math.cos(psi_d)

        # ---- Reference position, zeroed at first call ----
        ref_x_w = ref_x + d_lat * perp_x
        ref_y_w = ref_y + d_lat * perp_y
        if self._ref_x0 is None:
            self._ref_x0 = ref_x_w
            self._ref_y0 = ref_y_w
        Xd = ref_x_w - self._ref_x0
        Yd = ref_y_w - self._ref_y0
        # Freeze Xd/Yd once lane change is complete so the reference
        # does not drift as the look-ahead advances along the path.
        if sim_time > t_start + T:
            if self._Yd_frozen is None:
                self._Xd_frozen = Xd
                self._Yd_frozen = Yd
            Xd = self._Xd_frozen
            Yd = self._Yd_frozen

        # ---- Road curvature → psi_dot_d ----
        ahead_idx = min(la_idx + max(1, int(CURV_STEP /
                        max(arc[la_idx] - arc[la_idx - 1], 1e-3))),
                        N - 1)
        psi_ahead = self.full_path_psi[ahead_idx]
        ds_curv   = arc[ahead_idx] - arc[la_idx]
        if ds_curv > 1e-3:
            dpsi = math.atan2(math.sin(psi_ahead - psi_d),
                              math.cos(psi_ahead - psi_d))
            psi_dot_road = dpsi * vx_ref / ds_curv
        else:
            psi_dot_road = 0.0

        v2 = vx_ref**2 + d_lat1**2
        psi_dot_lane = (vx_ref * d_lat2) / v2 if v2 > 1e-6 else 0.0
        psi_dot_d    = psi_dot_road + psi_dot_lane

        # ---- psi_dd_d ----
        if v2 > 1e-6:
            N_   = vx_ref * d_lat2
            Nd   = vx_ref * d_lat3
            Dd   = 2.0 * d_lat1 * d_lat2
            psi_dd_d = (Nd * v2 - N_ * Dd) / (v2**2)
        else:
            psi_dd_d = 0.0

        return Xd, Yd, psi_d, psi_dot_d, psi_dd_d

    # =====================================================
    # STEP
    # =====================================================

    def step(self, steer, throttle, brake):

        steer = float(steer)
        throttle = float(throttle)
        brake = float(brake)

        self.vehicle.apply_control(
            carla.VehicleControl(
                throttle=throttle,
                steer=steer,
                brake=brake
            )
        )

        self.world.tick()

        tf = self.vehicle.get_transform()

        vel = self.vehicle.get_velocity()

        # Raw CARLA coordinates (used for waypoint search and debug draw)
        x_raw = tf.location.x
        y_raw = tf.location.y

        # Shifted coordinates (origin = spawn point)
        x = x_raw - self.origin_x
        y = y_raw - self.origin_y

        psi = math.radians(tf.rotation.yaw)

        # Unwrap psi to avoid ±π jumps in the heading error e_psi = psi - psi_d
        if self._psi_unwrap is None:
            self._psi_unwrap = psi
        else:
            dpsi = math.atan2(math.sin(psi - self._psi_unwrap),
                              math.cos(psi - self._psi_unwrap))
            self._psi_unwrap += dpsi
        psi = self._psi_unwrap

        # Capture spawn heading on first step.
        # All psi outputs are relative to this so that vx*psi stays
        # valid in the small-angle regime regardless of road direction.
        if self._psi0 is None:
            self._psi0 = psi
        psi_rel = psi - self._psi0

        # Longitudinal speed: project global velocity onto vehicle heading
        vx = vel.x * math.cos(psi) + vel.y * math.sin(psi)

        # Clamp vx to a minimum to prevent division-by-zero in A and B
        # controller terms.  Use 2.0 m/s so A and B stay bounded when
        # the vehicle is slow (gains were designed at vx_ref = 10 m/s).
        VX_MIN = 3.0
        vx_safe = math.copysign(max(abs(vx), VX_MIN), 1.0)

        # Measured yaw rate from CARLA angular velocity (Z axis, deg/s → rad/s)
        omega = self.vehicle.get_angular_velocity()
        psi_dot = math.radians(omega.z)

        # Global lateral velocity (Y-axis of CARLA world frame)
        Y_dot = vel.y

        # --------------------------------------------------
        # Simulation time  (local counter, starts at 0)
        # --------------------------------------------------

        self.sim_time += self._dt
        sim_time = self.sim_time

        Xd, Yd, psi_d, psi_dot_d, psi_dd_d = (
            self._lane_change_reference(sim_time, vx_safe,
                                        x_raw, y_raw, tf.location.z,
                                        psi)  # pass current heading for init
        )

        # psi_d relative to same origin as psi_rel
        psi_d_rel = psi_d - self._psi0

        # Output vx_safe (clamped) as port 4 so controller A,B terms never
        # divide by zero.  The raw vx is still used for kinematic terms inside
        # the bridge and for reference generation.
        # Log inputs and outputs for offline debugging
        self._log.append({
            't':         self.sim_time,
            'steer_in':  steer,
            'throttle':  throttle,
            'brake':     brake,
            'x':         x,
            'y':         y,
            'psi':       psi_rel,
            'vx':        vx_safe,
            'Xd':        Xd,
            'Yd':        Yd,
            'psi_d':     psi_d_rel,
            'psi_dot_d': psi_dot_d,
            'psi_dd_d':  psi_dd_d,
            'psi_dot':   psi_dot,
            'Y_dot':     Y_dot,
            'psi_abs':   psi,          # absolute (debug)
            'vx_raw':    vx,           # unclamped (debug)
        })

        return [x, y, psi_rel, vx_safe, Xd, Yd, psi_d_rel, psi_dot_d, psi_dd_d, psi_dot, Y_dot]

    # =====================================================
    # CLOSE
    # =====================================================

    def close(self):
        print("Destroying vehicle...")
        try:
            if self.vehicle is not None:
                self.vehicle.destroy()
        except Exception as e:
            print("Destroy error:", e)

        # Save log to CSV
        if self._log:
            import csv, os
            try:
                keys = list(self._log[0].keys())
                tmp_path = self._log_path + '.tmp'
                with open(tmp_path, 'w', newline='', encoding='utf-8') as f:
                    writer = csv.DictWriter(f, fieldnames=keys)
                    writer.writeheader()
                    writer.writerows(self._log)
                os.replace(tmp_path, self._log_path)
                print(f"Bridge log saved: {self._log_path} ({len(self._log)} rows)")
            except Exception as e:
                print(f"Log save error: {e}")