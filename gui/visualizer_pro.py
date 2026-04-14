"""
visualizer_pro.py — 3D Robot Arm Visualizer for 2-DoF Satellite Dish

Features:
  ✓ DH-based forward kinematics (T0, T1, T2 frames)
  ✓ Parabolic dish + horn + feed point
  ✓ Joint angle arc indicators (q1 azimuth, q2 elevation)
  ✓ End-effector coordinate frame
  ✓ DH parameter table overlay
  ✓ Workspace wireframe envelope
  ✓ Trajectory trace path during animation
  ✓ Auto-fit camera centered on robot
"""

import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor
from PySide6.QtWidgets import QVBoxLayout, QWidget
from PySide6.QtCore import QTimer
from gui.theme import Theme


class RobotArmVisualizer(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout(self)
        self.plotter = QtInteractor(self)
        self.layout.addWidget(self.plotter)

        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self._animation_step)
        self.trajectory_q = None
        self.current_frame = 0
        self.config = None

        self.actors = {}
        self.is_built = False
        self.trace_points = []  # End-effector trajectory trace

    # ═══════════════════════════════════════════════════════════════════════
    # DH Kinematics
    # ═══════════════════════════════════════════════════════════════════════

    DH_D1 = 1.601
    DH_A1 = 0.192
    DH_A2 = 0.936

    def get_dh_frames(self, q1, q2):
        d1, a1, a2 = self.DH_D1, self.DH_A1, self.DH_A2

        T0 = np.eye(4)
        P0 = np.array([0.0, 0.0, 0.0])
        P1 = np.array([0.0, 0.0, d1])

        c1, s1 = np.cos(q1), np.sin(q1)
        T1 = np.array([
            [c1, 0,  s1, a1 * c1],
            [s1, 0, -c1, a1 * s1],
            [0,  1,  0,  d1],
            [0,  0,  0,  1]
        ])
        P2 = T1[:3, 3]

        c2, s2 = np.cos(q2), np.sin(q2)
        T12 = np.array([
            [c2, -s2, 0, a2 * c2],
            [s2,  c2, 0, a2 * s2],
            [0,   0,  1, 0],
            [0,   0,  0, 1]
        ])
        T2 = T1 @ T12
        P3 = T2[:3, 3]

        return [T0, T1, T2], [P0, P1, P2, P3]

    # ═══════════════════════════════════════════════════════════════════════
    # Geometry Helpers
    # ═══════════════════════════════════════════════════════════════════════

    @staticmethod
    def _make_orthonormal_basis(normal):
        """Build orthonormal basis from a single normal vector."""
        n = normal / np.linalg.norm(normal)
        helper = np.array([0, 0, 1]) if abs(n[2]) < 0.9 else np.array([1, 0, 0])
        b1 = np.cross(n, helper)
        b1 /= np.linalg.norm(b1)
        b2 = np.cross(n, b1)
        b2 /= np.linalg.norm(b2)
        return b1, b2

    def _make_dish_mesh(self, center, horn_dir, dish_r=0.468, focal=0.25,
                        n_ring=48, n_depth=14):
        """Create parabolic dish mesh opening towards horn_dir."""
        open_dir = horn_dir / np.linalg.norm(horn_dir)
        b1, b2 = self._make_orthonormal_basis(open_dir)

        pts = [center.copy()]
        for j in range(1, n_depth + 1):
            r = dish_r * j / n_depth
            depth = (r ** 2) / (4 * focal)
            for k in range(n_ring):
                theta = 2 * np.pi * k / n_ring
                pt = (center + depth * open_dir
                      + r * np.cos(theta) * b1
                      + r * np.sin(theta) * b2)
                pts.append(pt)

        pts = np.array(pts)
        faces = []
        for k in range(n_ring):
            k_next = (k + 1) % n_ring
            faces += [3, 0, 1 + k, 1 + k_next]
        for j in range(1, n_depth):
            for k in range(n_ring):
                k_next = (k + 1) % n_ring
                i0 = 1 + (j - 1) * n_ring + k
                i1 = 1 + (j - 1) * n_ring + k_next
                i2 = 1 + j * n_ring + k_next
                i3 = 1 + j * n_ring + k
                faces += [3, i0, i1, i2, 3, i0, i2, i3]
        return pv.PolyData(pts, np.array(faces))

    def _make_rim_mesh(self, center, horn_dir, dish_r=0.468, focal=0.25,
                       n_ring=48, tube_r=0.015):
        """Create tube ring at dish edge."""
        open_dir = horn_dir / np.linalg.norm(horn_dir)
        b1, b2 = self._make_orthonormal_basis(open_dir)
        depth = (dish_r ** 2) / (4 * focal)
        rim_pts = []
        for k in range(n_ring + 1):
            theta = 2 * np.pi * k / n_ring
            pt = (center + depth * open_dir
                  + dish_r * np.cos(theta) * b1
                  + dish_r * np.sin(theta) * b2)
            rim_pts.append(pt)
        return pv.Spline(np.array(rim_pts), n_points=n_ring + 1).tube(radius=tube_r)

    def _make_angle_arc(self, center, axis, ref_dir, angle, radius=0.3):
        """
        Create a tube arc showing joint angle via Rodrigues' rotation.
        Always returns at least a reference line (dashed stub) even at 0°.
        """
        axis = axis / np.linalg.norm(axis)
        ref_dir = ref_dir / np.linalg.norm(ref_dir)

        # If angle is near zero, draw a short reference line along ref_dir
        if abs(angle) < np.radians(1):
            p0 = center
            p1 = center + radius * ref_dir
            return pv.Line(p0, p1).tube(radius=0.008)

        n = max(int(abs(angle) / np.radians(5)), 6)
        pts = []
        for i in range(n + 1):
            t = angle * i / n
            ct, st = np.cos(t), np.sin(t)
            v = (ref_dir * ct
                 + np.cross(axis, ref_dir) * st
                 + axis * np.dot(axis, ref_dir) * (1 - ct))
            pts.append(center + radius * v)

        pts = np.array(pts)
        if len(pts) < 2:
            p0 = center
            p1 = center + radius * ref_dir
            return pv.Line(p0, p1).tube(radius=0.008)
        return pv.Spline(pts, n_points=len(pts)).tube(radius=0.012)

    def _make_workspace_wireframe(self):
        """
        Create wireframe circles + arcs showing the reachable workspace
        envelope of the end-effector.
        """
        meshes = []

        # Horizontal circles at different elevation angles
        for q2_deg in [-20, 0, 30, 60, 80]:
            q2 = np.radians(q2_deg)
            pts = []
            for i in range(49):
                q1 = 2 * np.pi * i / 48
                _, Ps = self.get_dh_frames(q1, q2)
                pts.append(Ps[3])
            meshes.append(pv.Spline(np.array(pts), n_points=len(pts)))

        # Vertical arcs at different azimuth angles
        for q1_deg in [0, 90, 180, 270]:
            q1 = np.radians(q1_deg)
            pts = []
            for j in range(25):
                q2 = np.radians(-20 + 100 * j / 24)
                _, Ps = self.get_dh_frames(q1, q2)
                pts.append(Ps[3])
            meshes.append(pv.Spline(np.array(pts), n_points=len(pts)))

        combined = meshes[0]
        for m in meshes[1:]:
            combined = combined.merge(m)
        return combined

    @staticmethod
    def _points_to_polyline(pts):
        """Convert Nx3 point array to a polyline PolyData."""
        n = len(pts)
        cells = np.zeros(3 * (n - 1), dtype=np.int64)
        for i in range(n - 1):
            cells[3 * i] = 2
            cells[3 * i + 1] = i
            cells[3 * i + 2] = i + 1
        return pv.PolyData(pts, lines=cells)

    # ═══════════════════════════════════════════════════════════════════════
    # Scene Construction
    # ═══════════════════════════════════════════════════════════════════════

    def build_scene(self, config):
        self.config = config
        self.plotter.clear()
        self.plotter.set_background(Theme.BG_CANVAS)
        self.actors = {}
        self.trace_points = []

        # ── Static Base Plate ────────────────────────────────────────────
        base = pv.Cylinder(center=(0, 0, -0.025), direction=(0, 0, 1),
                           radius=0.3, height=0.05)
        self.plotter.add_mesh(base, color="#3b3b3b", pbr=True, metallic=0.6)

        # ── Dynamic actors (placeholder spheres, replaced each frame) ────
        actor_defs = {
            # Links
            'l1_vert':   dict(color="#dcdcdc", pbr=True, metallic=0.8),
            'l1_offset': dict(color="#dcdcdc", pbr=True, metallic=0.8),
            # Joint 2
            'j2_cyl':    dict(color="#ffaa00", pbr=True, metallic=0.7),
            # Dish
            'dish':      dict(color="#a0a0a0", pbr=True, metallic=0.9),
            'dish_rim':  dict(color="#888888", pbr=True, metallic=0.8),
            # Horn
            'horn':      dict(color="#cc8800", pbr=True, metallic=0.6),
            # Feed point
            'ee_sphere': dict(color="#ff0000", pbr=True, metallic=0.3),
        }
        for name, kwargs in actor_defs.items():
            self.actors[name] = self.plotter.add_mesh(
                pv.Sphere(radius=0.01), **kwargs)

        # ── Angle arcs (q1 = blue, q2 = orange) ─────────────────────────
        self.actors['arc_q1'] = self.plotter.add_mesh(
            pv.Sphere(radius=0.001), color="#58a6ff", opacity=0.85)
        self.actors['arc_q2'] = self.plotter.add_mesh(
            pv.Sphere(radius=0.001), color="#f0883e", opacity=0.85)
        # Arc endpoint arrows (small spheres at arc tips)
        self.actors['arc_q1_label'] = self.plotter.add_mesh(
            pv.Sphere(radius=0.001), color="#58a6ff", opacity=0.9)
        self.actors['arc_q2_label'] = self.plotter.add_mesh(
            pv.Sphere(radius=0.001), color="#f0883e", opacity=0.9)

        # ── Coordinate frames (T0, T1, T2) — large arrows + axis labels ──
        axis_colors = {'X': '#ff4444', 'Y': '#44cc44', 'Z': '#4488ff'}
        frame_names = ['Base', 'Joint2', 'EE']
        self.label_polys = {}  # Store PolyData to move point labels dynamically
        for i in range(3):
            for ax_name, color in axis_colors.items():
                # Arrow mesh (will be replaced each frame)
                self.actors[f'T{i}_{ax_name}'] = self.plotter.add_mesh(
                    pv.Sphere(radius=0.01), color=color)
                # Axis tip label (e.g. "X'", "Y'", "Z'" for each frame)
                key = f'T{i}_{ax_name}_lbl'
                poly = pv.PolyData([0.0, 0.0, 0.0])
                self.label_polys[key] = poly
                self.actors[key] = self.plotter.add_point_labels(
                    poly, [f"{ax_name}'"], font_size=9, bold=True,
                    text_color=color, point_size=0,
                    shape=None, render_points_as_spheres=False,
                    always_visible=True, shadow=True, fill_shape=False)

        # ── Frame origin labels (T₀, T₁, T₂) ────────────────────────────
        for i, (name, color) in enumerate(zip(
                frame_names, ['#ff6b6b', '#ffaa00', '#ff4444'])):
            key = f'lbl_T{i}'
            subscript = chr(0x2080 + i)
            poly = pv.PolyData([0.0, 0.0, 0.0])
            self.label_polys[key] = poly
            self.actors[key] = self.plotter.add_point_labels(
                poly, [f"T{subscript} ({name})"], font_size=9, bold=True,
                text_color=color, point_size=0, shape=None,
                render_points_as_spheres=False, always_visible=True,
                shadow=True, fill_shape=False)

        # ── Trajectory trace (initially invisible, builds during anim) ──
        self.actors['trace'] = self.plotter.add_mesh(
            pv.Sphere(radius=0.001), color="#000000", opacity=0.75)

        # ── Workspace wireframe envelope (very subtle) ────────────────────
        try:
            ws = self._make_workspace_wireframe()
            self.plotter.add_mesh(ws, color='#888888', opacity=0.15,
                                  style='wireframe', line_width=1.0)
        except Exception:
            pass

        # ── Axes widget ──────────────────────────────────────────────────
        self.plotter.add_axes()

        # ── HUD: Joint angles ────────────────────────────────────────────
        self.actors['hud_text'] = self.plotter.add_text(
            "Azimuth  (q\u2081): 0.0\u00b0\nElevation (q\u2082): 0.0\u00b0",
            position='upper_left', font_size=9,
            color=Theme.TEXT_PRIMARY, shadow=False)

        # ── HUD: DH Parameter table ──────────────────────────────────────
        dh_text = (
            "DH Parameters\n"
            "──────────────────────────────\n"
            "  i  |  ai (m)  |  di (m)  |  alpha\n"
            "  1  |  0.192   |  1.601   |   90°\n"
            "  2  |  0.936   |  0.000   |    0°"
        )
        self.actors['dh_table'] = self.plotter.add_text(
            dh_text, position='lower_left', font_size=10,
            color='#c9d1d9', shadow=False)

        # ── Lighting ─────────────────────────────────────────────────────
        self.plotter.add_light(
            pv.Light(position=(3, -5, 5), focal_point=(0, 0, 1), intensity=1.5))
        self.plotter.add_light(
            pv.Light(light_type='headlight', intensity=0.7))

        # ── Camera (auto-centered on robot) ──────────────────────────────
        self._fit_camera()

        # ── Initial pose ─────────────────────────────────────────────────
        self._update_geometry(0, 0)
        self.is_built = True

    def _fit_camera(self):
        """Set camera with proper framing, robot centered in viewport."""
        self.plotter.camera_position = [
            (3.8, -3.8, 3.0),      # camera position (closer)
            (0.2,  0.1, 0.9),      # focal point (center of robot body)
            (0.0,  0.0, 1.0)       # view-up vector
        ]
        self.plotter.camera.view_angle = 42  # wider FOV to fit dish

    # ═══════════════════════════════════════════════════════════════════════
    # Geometry Update (called every animation frame)
    # ═══════════════════════════════════════════════════════════════════════

    def _update_geometry(self, q1, q2, record_trace=False):
        Ts, Ps = self.get_dh_frames(q1, q2)
        P0, P1, P2, P3 = Ps
        T0, T1, T2 = Ts

        R_ARM = 0.04
        R_JOINT = 0.08

        # ── Link 1 (vertical + offset segments) ─────────────────────────
        self.actors['l1_vert'].mapper.dataset.copy_from(
            pv.Line(P0, P1).tube(radius=R_ARM))
        self.actors['l1_offset'].mapper.dataset.copy_from(
            pv.Line(P1, P2).tube(radius=R_ARM))

        # ── Joint 2 cylinder ─────────────────────────────────────────────
        Z1_dir = T1[:3, 2]
        self.actors['j2_cyl'].mapper.dataset.copy_from(
            pv.Cylinder(center=P2, direction=Z1_dir,
                        radius=R_JOINT, height=0.25))

        # ── Horn direction (P2 → P3) ─────────────────────────────────────
        horn_vec = P3 - P2
        horn_len = np.linalg.norm(horn_vec)
        horn_dir = horn_vec / horn_len if horn_len > 1e-6 else T2[:3, 0]

        # ── Dish (parabolic reflector) ───────────────────────────────────
        self.actors['dish'].mapper.dataset.copy_from(
            self._make_dish_mesh(center=P2, horn_dir=horn_dir))
        self.actors['dish_rim'].mapper.dataset.copy_from(
            self._make_rim_mesh(center=P2, horn_dir=horn_dir))

        # ── Horn / waveguide cone ────────────────────────────────────────
        if horn_len > 1e-6:
            self.actors['horn'].mapper.dataset.copy_from(pv.Cone(
                center=(P2 + P3) / 2, direction=horn_dir,
                height=horn_len, radius=0.04, resolution=24))

        # ── Feed point (end-effector sphere — larger than joint) ──────────
        self.actors['ee_sphere'].mapper.dataset.copy_from(
            pv.Sphere(radius=0.06, center=P3))

        # ── Joint angle arcs ─────────────────────────────────────────────
        # q1: azimuth arc at height d1, around Z-axis
        arc_q1 = self._make_angle_arc(
            center=P1, axis=np.array([0, 0, 1]),
            ref_dir=np.array([1, 0, 0]), angle=q1, radius=0.45)
        self.actors['arc_q1'].mapper.dataset.copy_from(arc_q1)
        # Arc midpoint indicator
        if abs(q1) > np.radians(3):
            c1, s1 = np.cos(q1 / 2), np.sin(q1 / 2)
            mid_pt = P1 + 0.42 * np.array([c1, s1, 0])
            self.actors['arc_q1_label'].mapper.dataset.copy_from(
                pv.Sphere(radius=0.025, center=mid_pt))
        else:
            self.actors['arc_q1_label'].mapper.dataset.copy_from(
                pv.Sphere(radius=0.001, center=P1))

        # q2: elevation arc at joint 2, around T1's Z-axis
        arc_q2 = self._make_angle_arc(
            center=P2, axis=T1[:3, 2],
            ref_dir=T1[:3, 0], angle=q2, radius=0.40)
        self.actors['arc_q2'].mapper.dataset.copy_from(arc_q2)
        if abs(q2) > np.radians(3):
            t_mid = q2 / 2
            ct, st = np.cos(t_mid), np.sin(t_mid)
            ax = T1[:3, 2]
            rd = T1[:3, 0]
            v = rd * ct + np.cross(ax, rd) * st + ax * np.dot(ax, rd) * (1 - ct)
            mid_pt2 = P2 + 0.37 * v
            self.actors['arc_q2_label'].mapper.dataset.copy_from(
                pv.Sphere(radius=0.025, center=mid_pt2))
        else:
            self.actors['arc_q2_label'].mapper.dataset.copy_from(
                pv.Sphere(radius=0.001, center=P2))

        # ── Coordinate frames (T0, T1, T2) — prominent with labels ─────
        FRAME_SCALE = [0.70, 0.55, 0.65]  # Very large arrows
        SHAFT_R     = [0.035, 0.030, 0.030]
        TIP_R       = 0.10
        TIP_L       = 0.20

        axis_hex_colors = {'X': '#ff4444', 'Y': '#44cc44', 'Z': '#4488ff'}
        for i, T in enumerate(Ts):
            origin = T[:3, 3]
            sc = FRAME_SCALE[i]
            sr = SHAFT_R[i]
            for axis_name, col in zip(['X', 'Y', 'Z'], [0, 1, 2]):
                direction = T[:3, col]
                col_hex = axis_hex_colors[axis_name]
                
                # Arrow mesh
                self.actors[f'T{i}_{axis_name}'].mapper.dataset.copy_from(
                    pv.Arrow(start=origin, direction=direction,
                             scale=sc, tip_radius=TIP_R,
                             tip_length=TIP_L, shaft_radius=sr))
                # Axis tip label position (slightly past the arrow tip)
                tip_pos = origin + direction * sc * 1.10
                key = f'T{i}_{axis_name}_lbl'
                if key in self.label_polys:
                    self.label_polys[key].points = np.array([tip_pos], dtype=np.float32)
                    self.actors[key].GetMapper().Modified()

        # ── Frame origin labels (T₀ at base, T₁ at joint2, T₂ at EE) ────
        # Position each label near its own frame origin, offset to avoid overlap
        label_positions = [
            P0 + np.array([0.45, 0.0,  0.10]),    # T₀ beside base
            P2 + np.array([0.0, -0.40,  0.0]),    # T₁ below-left of joint2
            P3 + np.array([0.20, 0.20,  0.10]),   # T₂ above-right of EE
        ]
        f_names = ['Base', 'Joint2', 'EE']
        f_colors = ['#ff6b6b', '#ffaa00', '#ff4444']
        for i, pos in enumerate(label_positions):
            key = f'lbl_T{i}'
            if key in self.label_polys:
                self.label_polys[key].points = np.array([pos], dtype=np.float32)
                self.actors[key].GetMapper().Modified()

        # ── Trajectory trace ─────────────────────────────────────────────
        if record_trace:
            self.trace_points.append(P3.copy())
            if len(self.trace_points) >= 2:
                pts = np.array(self.trace_points)
                trace_line = self._points_to_polyline(pts)
                self.actors['trace'].mapper.dataset.copy_from(
                    trace_line.tube(radius=0.012))

        # ── HUD: update angle readout ────────────────────────────────────
        self.actors['hud_text'].SetText(
            2,
            f"Azimuth  (q\u2081): {np.degrees(q1):+.1f}\u00b0\n"
            f"Elevation (q\u2082): {np.degrees(q2):+.1f}\u00b0"
        )

    # ═══════════════════════════════════════════════════════════════════════
    # Animation Control
    # ═══════════════════════════════════════════════════════════════════════

    def start_animation(self, q_data, config):
        if not self.is_built:
            self.build_scene(config)

        self.trace_points = []  # Reset trace for new simulation
        self.trajectory_q = q_data
        self.current_frame = 0
        self.animation_timer.start(20)

    def _animation_step(self):
        if self.trajectory_q is None or self.current_frame >= len(self.trajectory_q):
            self.animation_timer.stop()
            return

        q1 = self.trajectory_q[self.current_frame, 0]
        q2 = self.trajectory_q[self.current_frame, 1]

        self._update_geometry(q1, q2, record_trace=True)
        self.plotter.render()

        self.current_frame += max(1, len(self.trajectory_q) // 250)