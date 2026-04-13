import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from PySide6.QtCore import QTimer
from gui.theme import Theme

class Antenna3DVisualizer(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(5, 5), dpi=100)
        self.fig.patch.set_facecolor(Theme.MPL_BG)
        super().__init__(self.fig)
        
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Animation Data
        self.animation_timer = QTimer()
        self.animation_timer.timeout.connect(self._animation_step)
        self.trajectory_q = None
        self.config = None
        self.current_frame = 0
        
        self.apply_style()

    def apply_style(self):
        self.ax.set_facecolor(Theme.MPL_BG)
        self.ax.xaxis.pane.fill = False
        self.ax.yaxis.pane.fill = False
        self.ax.zaxis.pane.fill = False
        self.ax.grid(True, color=Theme.MPL_GRID, alpha=0.3)
        self.ax.set_xlabel('X (m)', color=Theme.MPL_TEXT)
        self.ax.set_ylabel('Y (m)', color=Theme.MPL_TEXT)
        self.ax.set_zlabel('Z (m)', color=Theme.MPL_TEXT)
        self.ax.tick_params(colors=Theme.MPL_TICK)

    def start_animation(self, q_data, config):
        """Start animation based on simulation data"""
        self.trajectory_q = q_data
        self.config = config
        self.current_frame = 0
        # Run timer (e.g. 30ms per frame for smoothness)
        self.animation_timer.start(30)

    def _animation_step(self):
        if self.trajectory_q is None or self.current_frame >= len(self.trajectory_q):
            self.animation_timer.stop()
            return

        # Get angles at current frame
        q1 = self.trajectory_q[self.current_frame, 0] # Azimuth
        q2 = self.trajectory_q[self.current_frame, 1] # Elevation
        
        self.render_frame(q1, q2)
        
        # Skip some frames if simulation is long for faster animation
        step_size = max(1, len(self.trajectory_q) // 100) 
        self.current_frame += step_size

    def render_frame(self, q1, q2):
        self.ax.clear()
        self.apply_style()
        
        # Parameters according to drawing (d1 and a2)
        l1 = self.config.physical.l1  # Pedestal (1.601m)
        l2 = self.config.physical.l2  # Arm (0.936m)
        
        # Compact Forward Kinematics
        p0 = np.array([0, 0, 0])
        p1 = np.array([0, 0, l1])
        
        # Calculate dish position (End Effector)
        # Azimuth rotation (q1) then Elevation (q2)
        ee_x = l2 * np.cos(q1) * np.cos(q2)
        ee_y = l2 * np.sin(q1) * np.cos(q2)
        ee_z = l1 + l2 * np.sin(q2)
        p2 = np.array([ee_x, ee_y, ee_z])

        # Draw Link 1 (Pedestal)
        self.ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], 
                     color=Theme.TEXT_SECONDARY, lw=4)
        
        # Draw Link 2 (Arm)
        self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 
                     color=Theme.BLUE, lw=3)

        # Draw Parabolic Dish
        u = np.linspace(0, 2 * np.pi, 15)
        v = np.linspace(0, 0.4, 8) 
        U, V = np.meshgrid(u, v)
        
        DISH_X = V * np.cos(U)
        DISH_Y = V * np.sin(U)
        DISH_Z = 1.5 * (V**2) # Parabola curvature

        # Rotation Matrices
        R_z = np.array([[np.cos(q1), -np.sin(q1), 0], [np.sin(q1), np.cos(q1), 0], [0, 0, 1]])
        R_y = np.array([[np.cos(q2), 0, np.sin(q2)], [0, 1, 0], [-np.sin(q2), 0, np.cos(q2)]])
        
        dish_points = np.stack([DISH_X.flatten(), DISH_Y.flatten(), DISH_Z.flatten()])
        rotated_dish = R_z @ R_y @ dish_points
        
        DX = rotated_dish[0, :].reshape(DISH_X.shape) + p2[0]
        DY = rotated_dish[1, :].reshape(DISH_X.shape) + p2[1]
        DZ = rotated_dish[2, :].reshape(DISH_X.shape) + p2[2]

        self.ax.plot_surface(DX, DY, DZ, color=Theme.BLUE, alpha=0.7)

        # Fix Limits to avoid "jumping" during animation
        limit = (l1 + l2)
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_zlim(0, limit * 1.2)
        self.draw()
