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

    def get_dh_frames(self, q1, q2):
        # Use physical parameters from the paper (Fig 3) 
        # to ensure exact proportions, not mock defaults.
        d1 = 1.601
        a1 = 0.192
        a2 = 0.936

        # T0: Global Frame (Base)
        T0 = np.eye(4)
        P0 = np.array([0, 0, 0])
        
        # P1: End of vertical rod
        P1 = np.array([0, 0, d1])
        
        # T1: Frame at Joint 2
        # T1 = RotZ(q1) * TransZ(d1) * TransX(a1) * RotX(+pi/2)
        c1, s1 = np.cos(q1), np.sin(q1)
        T1 = np.array([
            [c1, 0,  s1, a1*c1],
            [s1, 0, -c1, a1*s1],
            [0,  1,  0,  d1],
            [0,  0,  0,  1]
        ])
        P2 = T1[:3, 3]

        # T2: Frame at End Effector
        # T12 = RotZ(q2) * TransX(a2)
        c2, s2 = np.cos(q2), np.sin(q2)
        T12 = np.array([
            [c2, -s2, 0, a2*c2],
            [s2,  c2, 0, a2*s2],
            [0,   0,  1, 0],
            [0,   0,  0, 1]
        ])
        T2 = T1 @ T12
        P3 = T2[:3, 3]
        
        return [T0, T1, T2], [P0, P1, P2, P3]

    def build_scene(self, config):
        self.config = config
        self.plotter.clear()
        self.plotter.set_background(Theme.BG_CANVAS)
        
        self.actors = {}
        
        # Static Base Plate (Reduced size)
        base_plate = pv.Cylinder(center=(0, 0, -0.025), direction=(0, 0, 1), radius=0.3, height=0.05)
        self.plotter.add_mesh(base_plate, color="#3b3b3b", pbr=True, metallic=0.6)
        
        # Moving Parts
        self.actors['l1_vert'] = self.plotter.add_mesh(pv.Sphere(radius=0.01), color="#dcdcdc", pbr=True, metallic=0.8)
        self.actors['l1_offset'] = self.plotter.add_mesh(pv.Sphere(radius=0.01), color="#dcdcdc", pbr=True, metallic=0.8)
        self.actors['j2_cyl'] = self.plotter.add_mesh(pv.Sphere(radius=0.01), color="#ffaa00", pbr=True, metallic=0.7)
        self.actors['l2_arm'] = self.plotter.add_mesh(pv.Sphere(radius=0.01), color="#3fb950", pbr=True, metallic=0.6)
        self.actors['ee_sphere'] = self.plotter.add_mesh(pv.Sphere(radius=0.01), color="#ff0000", pbr=True, metallic=0.3)
        
        # Add labels dynamically via 3D text (Optional, we'll just use arrows for now to ensure speed)
        
        # Frames (T0, T1, T2) - RGB Arrows
        for i in range(3):
            self.actors[f'T{i}_X'] = self.plotter.add_mesh(pv.Sphere(radius=0.01), color="red")
            self.actors[f'T{i}_Y'] = self.plotter.add_mesh(pv.Sphere(radius=0.01), color="green")
            self.actors[f'T{i}_Z'] = self.plotter.add_mesh(pv.Sphere(radius=0.01), color="blue")

        # Set initial geometry moved to end
        
        self.plotter.add_axes()
        
        # Add dynamic HUD text
        self.actors['hud_text'] = self.plotter.add_text(
            "Azimuth (q1): 0.0°\nElevation (q2): 0.0°", 
            position='upper_left', 
            font_size=12, 
            color=Theme.TEXT_PRIMARY,
            shadow=False
        )
        
        light1 = pv.Light(position=(3, -5, 5), focal_point=(0, 0, 1), intensity=1.5)
        self.plotter.add_light(light1)
        self.plotter.add_light(pv.Light(light_type='headlight', intensity=0.7))
        
        self.plotter.camera_position = [(7.0, -7.0, 5.0), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0)]
        # self.plotter.reset_camera() 
        
        # Set initial geometry 
        self._update_geometry(0, 0)
        
        self.is_built = True

    def _update_geometry(self, q1, q2):
        Ts, Ps = self.get_dh_frames(q1, q2)
        P0, P1, P2, P3 = Ps
        T0, T1, T2 = Ts
        
        R_ARM = 0.04
        R_JOINT = 0.08
        
        # Link 1
        self.actors['l1_vert'].mapper.dataset.copy_from(pv.Line(P0, P1).tube(radius=R_ARM))
        self.actors['l1_offset'].mapper.dataset.copy_from(pv.Line(P1, P2).tube(radius=R_ARM))
        
        # Joint 2
        Z1_dir = T1[:3, 2]
        self.actors['j2_cyl'].mapper.dataset.copy_from(
            pv.Cylinder(center=P2, direction=Z1_dir, radius=R_JOINT, height=0.25)
        )
        
        # Link 2 + End Effector
        self.actors['l2_arm'].mapper.dataset.copy_from(pv.Line(P2, P3).tube(radius=R_ARM * 0.8))
        self.actors['ee_sphere'].mapper.dataset.copy_from(pv.Sphere(radius=0.06, center=P3))
        
        # Frames 
        for i, T in enumerate(Ts):
            origin = T[:3, 3]
            X_dir = T[:3, 0]
            Y_dir = T[:3, 1]
            Z_dir = T[:3, 2]
            
            Scale = 0.20 if i > 0 else 0.30
            
            shaft_r = 0.02
            tip_r = 0.06
            tip_l = 0.25
            
            self.actors[f'T{i}_X'].mapper.dataset.copy_from(pv.Arrow(start=origin, direction=X_dir, scale=Scale, tip_radius=tip_r, tip_length=tip_l, shaft_radius=shaft_r))
            self.actors[f'T{i}_Y'].mapper.dataset.copy_from(pv.Arrow(start=origin, direction=Y_dir, scale=Scale, tip_radius=tip_r, tip_length=tip_l, shaft_radius=shaft_r))
            self.actors[f'T{i}_Z'].mapper.dataset.copy_from(pv.Arrow(start=origin, direction=Z_dir, scale=Scale, tip_radius=tip_r, tip_length=tip_l, shaft_radius=shaft_r))

        # Update HUD text
        hud_str = f"Azimuth (q1): {np.degrees(q1):+.1f} deg\nElevation (q2): {np.degrees(q2):+.1f} deg"
        self.actors['hud_text'].SetText(2, hud_str)

    def start_animation(self, q_data, config):
        if not self.is_built:
            self.build_scene(config)
            
        self.trajectory_q = q_data
        self.current_frame = 0
        # Slower, smoother animation
        self.animation_timer.start(20) 

    def _animation_step(self):
        if self.trajectory_q is None or self.current_frame >= len(self.trajectory_q):
            self.animation_timer.stop()
            return

        q1 = self.trajectory_q[self.current_frame, 0] 
        q2 = self.trajectory_q[self.current_frame, 1] 
        
        self._update_geometry(q1, q2)
        self.plotter.render()
        
        # Show more frames for smoother "process" visibility
        self.current_frame += max(1, len(self.trajectory_q) // 250)
