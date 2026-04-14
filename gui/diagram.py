"""
diagram.py — Block diagram canvas view (Simulink-style).

Hosts all blocks in a QGraphicsScene with signal-flow arrows
and animated dash patterns during simulation.
"""

import numpy as np
from PySide6.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsTextItem
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QPainter, QColor, QFont

from gui.theme import Theme
from gui.blocks import SimulinkBlock, SignalArrow, FeedbackArrow


class BlockDiagramView(QGraphicsView):
    """The Simulink-style block diagram canvas."""

    def __init__(self, on_block_clicked):
        super().__init__()
        self._scene = QGraphicsScene()
        self.setScene(self._scene)
        self.on_block_clicked = on_block_clicked
        self.setRenderHints(QPainter.RenderHint.Antialiasing |
                            QPainter.RenderHint.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)

        self._scene.setBackgroundBrush(QColor(Theme.BG_CANVAS))

        self.arrows = []
        self.feedback_arrows = []
        self._anim_timer = QTimer()
        self._anim_timer.timeout.connect(self._animate_arrows)

        self.blocks = {}
        self._build_diagram()

    def _build_diagram(self):
        """Construct the block diagram with blocks, arrows, and labels."""
        bw = SimulinkBlock.BLOCK_WIDTH
        bh = SimulinkBlock.BLOCK_HEIGHT
        gap = 60
        y_main = 80
        x_start = 50

        x1 = x_start
        x2 = x1 + bw + gap
        x3 = x2 + bw + gap
        x4 = x3 + bw + gap
        x5 = x4 + bw + gap

        y_fb = y_main + bh + 90  # feedback row

        # ═══ BLOCKS ═══
        for bid, x, y in [
            ('input', x1, y_main),
            ('ref',   x2, y_main),
            ('mrac',  x3, y_main),
            ('plant', x4, y_main),
            ('scope', x5, y_main),
            ('mit',   x3, y_fb),
        ]:
            blk = SimulinkBlock(x, y, bid, self.on_block_clicked)
            self.blocks[bid] = blk
            self._scene.addItem(blk)

        # ═══ FORWARD ARROWS ═══
        pairs = [
            ('input', 'ref',   "#000000"),
            ('ref',   'mrac',  "#000000"),
            ('mrac',  'plant', "#000000"),
            ('plant', 'scope', "#000000"),
        ]
        for src_id, dst_id, color in pairs:
            s = self.blocks[src_id]
            d = self.blocks[dst_id]
            arrow = SignalArrow(s.right_x, s.center_y,
                                d.left_x, d.center_y, color)
            self.arrows.append(arrow)
            self._scene.addItem(arrow)

        # ═══ FEEDBACK ARROWS ═══
        plant = self.blocks['plant']
        mit = self.blocks['mit']
        mrac = self.blocks['mrac']
        ref = self.blocks['ref']

        # Plant → MIT Rule (state feedback)
        fb1 = FeedbackArrow([
            (plant.center_x, plant.bottom_y),
            (plant.center_x, mit.center_y),
            (mit.right_x, mit.center_y),
        ], "#000000")
        self.feedback_arrows.append(fb1)

        # MIT Rule → MRAC (adaptive signal)
        fb2 = FeedbackArrow([
            (mit.left_x, mit.center_y),
            (mrac.left_x - 30, mit.center_y),
            (mrac.left_x - 30, mrac.center_y + 20),
            (mrac.left_x, mrac.center_y + 20),
        ], "#000000")
        self.feedback_arrows.append(fb2)

        # Reference → MIT Rule (error signal)
        fb3 = FeedbackArrow([
            (ref.center_x, ref.bottom_y),
            (ref.center_x, mit.bottom_y + 40),
            (mit.center_x, mit.bottom_y + 40),
            (mit.center_x, mit.bottom_y),
        ], "#000000")
        self.feedback_arrows.append(fb3)

        for fb in self.feedback_arrows:
            self._scene.addItem(fb)

        # ═══ SIGNAL LABELS ═══
        cy = y_main + bh / 2
        labels = [
            (x1 + bw + gap / 2 - 12, cy - 25, "r(t)"),
            (x2 + bw + gap / 2 - 20, cy - 25, "qm(t)"),
            (x3 + bw + gap / 2 - 12, cy - 25, "u(t)"),
            (x4 + bw + gap / 2 - 14, cy - 25, "q(t)"),
            (plant.center_x + 8, (plant.bottom_y + mit.center_y) / 2 - 10, "q, dq"),
            (mrac.left_x - 55, mit.center_y - 20, "θ"),
            (ref.center_x + 8, mit.bottom_y + 15, "e(t)"),
        ]
        for lx, ly, txt in labels:
            lbl = QGraphicsTextItem(txt)
            lbl.setDefaultTextColor(QColor(Theme.TEXT_SECONDARY))
            lbl.setFont(QFont("Segoe UI", 10, QFont.Weight.Bold))
            lbl.setPos(lx, ly)
            self._scene.addItem(lbl)

        # Fit scene
        self._scene.setSceneRect(
            self._scene.itemsBoundingRect().adjusted(-30, -30, 30, 30)
        )

    # ── Animation ──────────────────────────────────────────────
    def start_animation(self):
        for a in self.arrows:
            a.set_animated(True)
        for fb in self.feedback_arrows:
            fb.set_animated(True)
        self._anim_timer.start(80)

    def stop_animation(self):
        self._anim_timer.stop()
        for a in self.arrows:
            a.set_animated(False)
        for fb in self.feedback_arrows:
            fb.set_animated(False)

    def _animate_arrows(self):
        for a in self.arrows:
            a.advance_dash()
        for fb in self.feedback_arrows:
            fb.advance_dash()

    def wheelEvent(self, event):
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.scale(factor, factor)
