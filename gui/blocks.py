"""
blocks.py — Simulink-style block diagram items with custom-drawn icons.

Each block type has a unique hand-drawn icon:
    - Step Input   : Step function waveform
    - Reference    : Sine wave (ideal response)
    - MRAC Ctrl    : Gear/cog (controller)
    - Plant        : Parabolic antenna dish
    - MIT Rule     : Circular adaptation arrow
    - Scope        : Oscilloscope waveform
"""

import math
import numpy as np
from PySide6.QtWidgets import (
    QGraphicsPathItem, QGraphicsTextItem, QGraphicsDropShadowEffect
)
from PySide6.QtCore import Qt
from PySide6.QtGui import (
    QPen, QBrush, QColor, QFont, QPainterPath, QLinearGradient, QPainter
)

from gui.theme import Theme


# ═══════════════════════════════════════════════════════════════
#  ICON DRAWING FUNCTIONS
# ═══════════════════════════════════════════════════════════════

def draw_step_icon(path: QPainterPath, cx: float, cy: float, size: float = 18):
    """Draw a step-function waveform icon."""
    s = size
    # Flat low → step up → flat high
    path.moveTo(cx - s, cy + s * 0.4)
    path.lineTo(cx - s * 0.2, cy + s * 0.4)
    path.lineTo(cx - s * 0.2, cy - s * 0.4)
    path.lineTo(cx + s, cy - s * 0.4)


def draw_sine_icon(path: QPainterPath, cx: float, cy: float, size: float = 18):
    """Draw a sine wave icon (ideal reference)."""
    s = size
    n_pts = 30
    for i in range(n_pts + 1):
        x = cx - s + (2 * s * i / n_pts)
        y = cy - s * 0.4 * math.sin(2 * math.pi * i / n_pts)
        if i == 0:
            path.moveTo(x, y)
        else:
            path.lineTo(x, y)


def draw_gear_icon(path: QPainterPath, cx: float, cy: float, size: float = 16):
    """Draw a gear/cog icon (controller)."""
    r_outer = size
    r_inner = size * 0.55
    n_teeth = 8
    for i in range(n_teeth * 2):
        angle = math.pi * i / n_teeth
        r = r_outer if i % 2 == 0 else r_inner
        x = cx + r * math.cos(angle)
        y = cy + r * math.sin(angle)
        if i == 0:
            path.moveTo(x, y)
        else:
            path.lineTo(x, y)
    path.closeSubpath()
    # Center hole
    path.addEllipse(cx - size * 0.2, cy - size * 0.2, size * 0.4, size * 0.4)


def draw_antenna_icon(path: QPainterPath, cx: float, cy: float, size: float = 18):
    """Draw a parabolic dish antenna icon."""
    s = size
    # Parabolic dish (arc)
    path.moveTo(cx - s, cy + s * 0.3)
    # Draw parabola via quadratic bezier
    path.quadTo(cx, cy - s * 0.8, cx + s, cy + s * 0.3)
    # Feed horn (small line from dish center upward)
    path.moveTo(cx, cy - s * 0.1)
    path.lineTo(cx, cy + s * 0.5)
    # Base stand
    path.moveTo(cx - s * 0.4, cy + s * 0.5)
    path.lineTo(cx + s * 0.4, cy + s * 0.5)
    # Small triangle feed at top
    path.moveTo(cx - s * 0.15, cy - s * 0.1)
    path.lineTo(cx, cy - s * 0.35)
    path.lineTo(cx + s * 0.15, cy - s * 0.1)


def draw_loop_icon(path: QPainterPath, cx: float, cy: float, size: float = 16):
    """Draw a circular adaptation loop icon (MIT Rule)."""
    s = size
    r = s * 0.7
    # Circular arc (270 degrees)
    rect = (cx - r, cy - r, 2 * r, 2 * r)
    path.arcMoveTo(*rect, 30)
    path.arcTo(*rect, 30, 300)
    # Arrowhead at end of arc
    end_angle = math.radians(30)
    ex = cx + r * math.cos(end_angle)
    ey = cy - r * math.sin(end_angle)
    path.moveTo(ex, ey)
    path.lineTo(ex + 6, ey - 2)
    path.moveTo(ex, ey)
    path.lineTo(ex + 2, ey + 6)


def draw_scope_icon(path: QPainterPath, cx: float, cy: float, size: float = 18):
    """Draw an oscilloscope waveform icon."""
    s = size
    # Screen border
    path.addRoundedRect(cx - s, cy - s * 0.6, 2 * s, s * 1.2, 3, 3)
    # Waveform inside
    n_pts = 20
    for i in range(n_pts + 1):
        x = cx - s * 0.8 + (1.6 * s * i / n_pts)
        # Damped sine
        t = i / n_pts * 3
        y = cy - s * 0.35 * math.sin(3.5 * t) * math.exp(-0.6 * t)
        if i == 0:
            path.moveTo(x, y)
        else:
            path.lineTo(x, y)


# ═══════════════════════════════════════════════════════════════
#  MAP: block_id → (icon_drawer, title, subtitle)
# ═══════════════════════════════════════════════════════════════

BLOCK_DEFINITIONS = {
    'input': {
        'icon': draw_step_icon,
        'title': 'Step Input',
        'subtitle': 'u(t) — Setpoint',
        'bg': Theme.BLOCK_INPUT,
        'border': Theme.BLOCK_INPUT_BORDER,
    },
    'ref': {
        'icon': draw_sine_icon,
        'title': 'Reference Model',
        'subtitle': 'Gm(s) — 2nd Order',
        'bg': Theme.BLOCK_REF,
        'border': Theme.BLOCK_REF_BORDER,
    },
    'mrac': {
        'icon': draw_gear_icon,
        'title': 'MRAC Controller',
        'subtitle': 'Computed Torque',
        'bg': Theme.BLOCK_CTRL,
        'border': Theme.BLOCK_CTRL_BORDER,
    },
    'plant': {
        'icon': draw_antenna_icon,
        'title': 'Satellite Dish',
        'subtitle': '2-DoF Plant',
        'bg': Theme.BLOCK_PLANT,
        'border': Theme.BLOCK_PLANT_BORDER,
    },
    'mit': {
        'icon': draw_loop_icon,
        'title': 'MIT Rule',
        'subtitle': 'Adaptive Law',
        'bg': Theme.BLOCK_ADAPT,
        'border': Theme.BLOCK_ADAPT_BORDER,
    },
    'scope': {
        'icon': draw_scope_icon,
        'title': 'Scope',
        'subtitle': 'Output Display',
        'bg': Theme.BLOCK_SCOPE,
        'border': Theme.BLOCK_SCOPE_BORDER,
    },
}


# ═══════════════════════════════════════════════════════════════
#  BLOCK ITEM
# ═══════════════════════════════════════════════════════════════

class SimulinkBlock(QGraphicsPathItem):
    """
    A premium Simulink-style block with:
      - Gradient fill
      - Custom-drawn icon (antenna, gear, waveform, etc.)
      - Title + subtitle text
      - Hover glow effects
    """

    BLOCK_WIDTH = 150
    BLOCK_HEIGHT = 90

    def __init__(self, x, y, block_id, callback):
        super().__init__()
        defn = BLOCK_DEFINITIONS[block_id]

        self.block_id = block_id
        self.callback = callback
        self.base_bg = QColor(defn['bg'])
        self.base_border = QColor(defn['border'])
        self._hover = False

        w = self.BLOCK_WIDTH
        h = self.BLOCK_HEIGHT

        # ── Rounded rectangle body ──
        body = QPainterPath()
        body.addRoundedRect(x, y, w, h, 12, 12)
        self.setPath(body)

        # Gradient fill
        grad = QLinearGradient(x, y, x, y + h)
        grad.setColorAt(0, self.base_bg.lighter(140))
        grad.setColorAt(0.5, self.base_bg.lighter(115))
        grad.setColorAt(1, self.base_bg)
        self.setBrush(QBrush(grad))
        self.setPen(QPen(self.base_border, 2.5))

        # ── Icon (drawn as a child path item) ──
        icon_path = QPainterPath()
        icon_cx = x + w / 2
        icon_cy = y + 22  # upper area of block
        defn['icon'](icon_path, icon_cx, icon_cy, size=16)

        icon_item = QGraphicsPathItem(icon_path, self)
        icon_pen = QPen(QColor(defn['border']), 2.0)
        icon_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        icon_pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        icon_item.setPen(icon_pen)
        icon_item.setBrush(QBrush(Qt.BrushStyle.NoBrush))

        # ── Title text ──
        title_item = QGraphicsTextItem(defn['title'], self)
        title_item.setDefaultTextColor(QColor(Theme.TEXT_PRIMARY))
        title_font = QFont("Segoe UI", 10, QFont.Weight.Bold)
        title_item.setFont(title_font)
        tr = title_item.boundingRect()
        title_item.setPos(x + (w - tr.width()) / 2, y + 38)

        # ── Subtitle text ──
        sub_item = QGraphicsTextItem(defn['subtitle'], self)
        sub_item.setDefaultTextColor(QColor(Theme.TEXT_MUTED))
        sub_font = QFont("Segoe UI", 8)
        sub_item.setFont(sub_font)
        sr = sub_item.boundingRect()
        sub_item.setPos(x + (w - sr.width()) / 2, y + 56)

        # ── Drop shadow ──
        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(20)
        shadow.setXOffset(0)
        shadow.setYOffset(4)
        shadow.setColor(QColor(0, 0, 0, 100))
        self.setGraphicsEffect(shadow)

        self.setAcceptHoverEvents(True)
        self.setFlag(self.GraphicsItemFlag.ItemIsSelectable, True)

        # Store geometry for arrow connections
        self.bx = x
        self.by = y
        self.bw = w
        self.bh = h

    @property
    def center_y(self):
        return self.by + self.bh / 2

    @property
    def right_x(self):
        return self.bx + self.bw

    @property
    def left_x(self):
        return self.bx

    @property
    def bottom_y(self):
        return self.by + self.bh

    @property
    def top_y(self):
        return self.by

    @property
    def center_x(self):
        return self.bx + self.bw / 2

    def hoverEnterEvent(self, event):
        self._hover = True
        self.setPen(QPen(self.base_border.lighter(150), 3.5))
        effect = self.graphicsEffect()
        if effect:
            effect.setBlurRadius(35)
            effect.setColor(QColor(self.base_border.red(), self.base_border.green(),
                                   self.base_border.blue(), 90))
        self.update()
        super().hoverEnterEvent(event)

    def hoverLeaveEvent(self, event):
        self._hover = False
        self.setPen(QPen(self.base_border, 2.5))
        effect = self.graphicsEffect()
        if effect:
            effect.setBlurRadius(20)
            effect.setColor(QColor(0, 0, 0, 100))
        self.update()
        super().hoverLeaveEvent(event)

    def mousePressEvent(self, event):
        self.callback(self.block_id)
        super().mousePressEvent(event)


# ═══════════════════════════════════════════════════════════════
#  SIGNAL ARROWS
# ═══════════════════════════════════════════════════════════════

class SignalArrow(QGraphicsPathItem):
    """Forward-path signal arrow with animation support."""

    def __init__(self, x1, y1, x2, y2, color=Theme.TEXT_SECONDARY):
        super().__init__()
        self.color = QColor(color)
        self._dash_offset = 0

        path = QPainterPath()
        path.moveTo(x1, y1)
        path.lineTo(x2, y2)

        # Arrowhead
        angle = np.arctan2(y2 - y1, x2 - x1)
        sz = 10
        path.moveTo(x2, y2)
        path.lineTo(x2 - sz * np.cos(angle - np.pi / 6),
                     y2 - sz * np.sin(angle - np.pi / 6))
        path.moveTo(x2, y2)
        path.lineTo(x2 - sz * np.cos(angle + np.pi / 6),
                     y2 - sz * np.sin(angle + np.pi / 6))

        self.setPath(path)
        pen = QPen(self.color, 2)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        self.setPen(pen)

    def set_animated(self, animated: bool):
        pen = self.pen()
        if animated:
            pen.setStyle(Qt.PenStyle.DashLine)
            pen.setColor(QColor(Theme.GREEN))
        else:
            pen.setStyle(Qt.PenStyle.SolidLine)
            pen.setColor(self.color)
        self.setPen(pen)

    def advance_dash(self):
        self._dash_offset += 1
        pen = self.pen()
        pen.setDashOffset(self._dash_offset)
        self.setPen(pen)


class FeedbackArrow(QGraphicsPathItem):
    """Feedback path (multi-segment) between blocks."""

    def __init__(self, points, color=Theme.TEXT_SECONDARY):
        super().__init__()
        self.color = QColor(color)

        path = QPainterPath()
        path.moveTo(points[0][0], points[0][1])
        for px, py in points[1:]:
            path.lineTo(px, py)

        # Arrowhead at last segment
        last = points[-1]
        prev = points[-2]
        angle = np.arctan2(last[1] - prev[1], last[0] - prev[0])
        sz = 10
        path.moveTo(last[0], last[1])
        path.lineTo(last[0] - sz * np.cos(angle - np.pi / 6),
                     last[1] - sz * np.sin(angle - np.pi / 6))
        path.moveTo(last[0], last[1])
        path.lineTo(last[0] - sz * np.cos(angle + np.pi / 6),
                     last[1] - sz * np.sin(angle + np.pi / 6))

        self.setPath(path)
        pen = QPen(self.color, 2)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        self.setPen(pen)

    def set_animated(self, animated: bool):
        pen = self.pen()
        if animated:
            pen.setStyle(Qt.PenStyle.DashLine)
            pen.setColor(QColor(Theme.ORANGE))
        else:
            pen.setStyle(Qt.PenStyle.SolidLine)
            pen.setColor(self.color)
        self.setPen(pen)

    def advance_dash(self):
        pen = self.pen()
        pen.setDashOffset(pen.dashOffset() + 1)
        self.setPen(pen)
