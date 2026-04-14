"""
blocks.py — MATLAB/Simulink style block diagram items with mathematical formulas.

Blocks are rendered as flat white rectangles with formula text inside 
and titles underneath, matching the classic Simulink aesthetic.
"""

import numpy as np
from PySide6.QtWidgets import (
    QGraphicsPathItem, QGraphicsTextItem
)
from PySide6.QtCore import Qt
from PySide6.QtGui import (
    QPen, QBrush, QColor, QFont, QPainterPath
)

from gui.theme import Theme


# ═══════════════════════════════════════════════════════════════
#  MAP: block_id → (formula, title, subtitle)
# ═══════════════════════════════════════════════════════════════

BLOCK_DEFINITIONS = {
    'input': {
        'formula': '<div style="font-size:14px; font-weight:bold;">r(t)</div>',
        'title': 'Step Input',
    },
    'ref': {
        'formula': '<div style="font-size:11px;">G<sub>m</sub>(s) = ω<sub>n</sub>² / (s² + 2ζω<sub>n</sub>s + ω<sub>n</sub>²)</div>',
        'title': 'Reference Model',
    },
    'mrac': {
        'formula': '<div style="font-size:11px;">τ = M̂q̈* + V̂ + Ĝ</div>',
        'title': 'MRAC Controller',
    },
    'plant': {
        'formula': '<div style="font-size:11px;">Mq̈ + Cq̇ + G = τ</div>',
        'title': 'Satellite Dish',
    },
    'mit': {
        'formula': '<div style="font-size:13px;">θ̇ = -γ · e · φ</div>',
        'title': 'MIT Rule',
    },
    'scope': {
        'formula': '<div style="font-size:13px; font-weight:bold;">Plot</div>',
        'title': 'Scope',
    },
}


# ═══════════════════════════════════════════════════════════════
#  BLOCK ITEM
# ═══════════════════════════════════════════════════════════════

class SimulinkBlock(QGraphicsPathItem):
    """
    A plain MATLAB/Simulink style block:
      - Flat white background
      - Thin black border
      - Mathematical formula in the center
    """

    BLOCK_WIDTH = 140
    BLOCK_HEIGHT = 80

    def __init__(self, x, y, block_id, callback):
        super().__init__()
        defn = BLOCK_DEFINITIONS[block_id]

        self.block_id = block_id
        self.callback = callback
        self.base_border = QColor("#000000")
        self._hover = False

        w = self.BLOCK_WIDTH
        h = self.BLOCK_HEIGHT

        # ── Simple Rectangle body ──
        body = QPainterPath()
        body.addRect(x, y, w, h)
        self.setPath(body)

        # Flat white fill
        self.setBrush(QBrush(QColor("#FFFFFF")))
        self.setPen(QPen(self.base_border, 1.2))

        # ── Formula text (Center) ──
        self.formula_item = QGraphicsTextItem(self)
        self.formula_item.setHtml(defn['formula'])
        self.formula_item.setDefaultTextColor(QColor("#1C1C1E"))
        fr = self.formula_item.boundingRect()
        self.formula_item.setPos(x + (w - fr.width()) / 2, y + (h - fr.height()) / 2)

        # ── Title text (Below block) ──
        self.title_item = QGraphicsTextItem(defn['title'], self)
        self.title_item.setDefaultTextColor(QColor("#3A3A3C"))
        title_font = QFont("Segoe UI", 9, QFont.Weight.Bold)
        self.title_item.setFont(title_font)
        tr = self.title_item.boundingRect()
        self.title_item.setPos(x + (w - tr.width()) / 2, y + h + 2)

        self.setAcceptHoverEvents(True)
        self.setFlag(self.GraphicsItemFlag.ItemIsSelectable, True)

        # Store geometry for arrow connections
        self.bx = x
        self.by = y
        self.bw = w
        self.bh = h

    @property
    def center_y(self): return self.by + self.bh / 2
    @property
    def right_x(self):  return self.bx + self.bw
    @property
    def left_x(self):   return self.bx
    @property
    def bottom_y(self): return self.by + self.bh
    @property
    def top_y(self):    return self.by
    @property
    def center_x(self): return self.bx + self.bw / 2

    def hoverEnterEvent(self, event):
        self._hover = True
        self.setPen(QPen(QColor(Theme.BLUE), 2.0))
        self.update()
        super().hoverEnterEvent(event)

    def hoverLeaveEvent(self, event):
        self._hover = False
        self.setPen(QPen(self.base_border, 1.2))
        self.update()
        super().hoverLeaveEvent(event)

    def mousePressEvent(self, event):
        self.callback(self.block_id)
        super().mousePressEvent(event)


# ═══════════════════════════════════════════════════════════════
#  SIGNAL ARROWS
# ═══════════════════════════════════════════════════════════════

class SignalArrow(QGraphicsPathItem):
    """Clean black signal arrow."""

    def __init__(self, x1, y1, x2, y2, color="#000000"):
        super().__init__()
        self.color = QColor(color)
        self._dash_offset = 0

        path = QPainterPath()
        path.moveTo(x1, y1)
        path.lineTo(x2, y2)

        # Arrowhead
        angle = np.arctan2(y2 - y1, x2 - x1)
        sz = 8
        path.moveTo(x2, y2)
        path.lineTo(x2 - sz * np.cos(angle - np.pi / 6),
                     y2 - sz * np.sin(angle - np.pi / 6))
        path.moveTo(x2, y2)
        path.lineTo(x2 - sz * np.cos(angle + np.pi / 6),
                     y2 - sz * np.sin(angle + np.pi / 6))

        self.setPath(path)
        pen = QPen(self.color, 1.2)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        self.setPen(pen)

    def set_animated(self, animated: bool):
        pen = self.pen()
        if animated:
            pen.setStyle(Qt.PenStyle.DashLine)
            pen.setColor(QColor(Theme.BLUE))
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
    """Feedback path signal arrow."""

    def __init__(self, points, color="#000000"):
        super().__init__()
        self.color = QColor(color)
        self._points = points
        self._dash_offset = 0

        path = QPainterPath()
        path.moveTo(points[0][0], points[0][1])
        for px, py in points[1:]:
            path.lineTo(px, py)

        # Arrowhead at last segment
        last = points[-1]
        prev = points[-2]
        angle = np.arctan2(last[1] - prev[1], last[0] - prev[0])
        sz = 8
        path.moveTo(last[0], last[1])
        path.lineTo(last[0] - sz * np.cos(angle - np.pi / 6),
                     last[1] - sz * np.sin(angle - np.pi / 6))
        path.moveTo(last[0], last[1])
        path.lineTo(last[0] - sz * np.cos(angle + np.pi / 6),
                     last[1] - sz * np.sin(angle + np.pi / 6))

        self.setPath(path)
        pen = QPen(self.color, 1.2)
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
        self._dash_offset += 1
        pen = self.pen()
        pen.setDashOffset(self._dash_offset)
        self.setPen(pen)
