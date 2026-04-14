"""
theme.py — Dark theme colors and stylesheet for the Simulink GUI.

Centralized design tokens for consistent dark-mode styling.
"""


class Theme:
    """Centralized light theme colors inspired by MATLAB/Simulink."""
    # Base backgrounds
    BG_DARKEST = "#F2F2F7" # System light gray
    BG_DARK = "#FFFFFF"    # Pure white
    BG_MID = "#E5E5EA"     # Mid gray
    BG_LIGHT = "#FFFFFF"   # Bright white
    BG_CANVAS = "#FFFFFF"  # Pure white canvas

    # Borders
    BORDER = "#C7C7CC"     # Standard light border
    BORDER_LIGHT = "#D1D1D6"

    # Text
    TEXT_PRIMARY = "#1C1C1E"   # Nearly black
    TEXT_SECONDARY = "#3A3A3C" # Dark gray
    TEXT_MUTED = "#8E8E93"    # Gray

    # Accent colors (slightly more professional for light mode)
    BLUE = "#007AFF"
    BLUE_DIM = "#0051D5"
    GREEN = "#34C759"
    GREEN_DIM = "#248A3D"
    ORANGE = "#FF9500"
    RED = "#FF3B30"
    PURPLE = "#AF52DE"
    CYAN = "#32ADE6"
    YELLOW = "#FFCC00"

    # Block-specific colors (Light mode friendly)
    BLOCK_INPUT = "#FFFFFF"
    BLOCK_INPUT_BORDER = "#007AFF"
    BLOCK_REF = "#FFFFFF"
    BLOCK_REF_BORDER = "#34C759"
    BLOCK_CTRL = "#FFFFFF"
    BLOCK_CTRL_BORDER = "#FF9500"
    BLOCK_PLANT = "#FFFFFF"
    BLOCK_PLANT_BORDER = "#AF52DE"
    BLOCK_ADAPT = "#FFFFFF"
    BLOCK_ADAPT_BORDER = "#FF3B30"
    BLOCK_SCOPE = "#FFFFFF"
    BLOCK_SCOPE_BORDER = "#32ADE6"

    # Matplotlib colors (consistent with light theme)
    MPL_BG = "#FFFFFF"
    MPL_AXES = "#FFFFFF"
    MPL_GRID = "#E5E5EA"
    MPL_TEXT = "#1C1C1E"
    MPL_TICK = "#3A3A3C"


def build_stylesheet() -> str:
    """Generate the full QSS stylesheet for Light Mode."""
    T = Theme
    return f"""
QMainWindow {{
    background-color: {T.BG_DARKEST};
}}
QWidget {{
    background-color: {T.BG_DARKEST};
    color: {T.TEXT_PRIMARY};
    font-family: 'Segoe UI', 'Inter', 'Roboto', sans-serif;
    font-size: 13px;
}}
QToolBar {{
    background-color: {T.BG_DARK};
    border-bottom: 1px solid {T.BORDER};
    spacing: 8px;
    padding: 6px 12px;
}}
QToolBar QLabel {{
    color: {T.TEXT_SECONDARY};
    font-size: 12px;
    padding: 0 8px;
}}
QPushButton {{
    background-color: {T.BG_LIGHT};
    color: {T.TEXT_PRIMARY};
    border: 1px solid {T.BORDER};
    border-radius: 6px;
    padding: 8px 18px;
    font-weight: 500;
    font-size: 13px;
}}
QPushButton:hover {{
    background-color: {T.BG_MID};
    border-color: {T.BORDER};
}}
QPushButton:pressed {{
    background-color: {T.BORDER};
}}
QPushButton:disabled {{
    background-color: {T.BG_DARKEST};
    color: {T.TEXT_MUTED};
    border-color: {T.BORDER_LIGHT};
}}
QPushButton#runBtn {{
    background-color: {T.GREEN};
    border-color: {T.GREEN_DIM};
    color: white;
}}
QPushButton#runBtn:hover {{
    background-color: {T.GREEN_DIM};
}}
QPushButton#exportBtn {{
    background-color: {T.BLUE};
    border-color: {T.BLUE_DIM};
    color: white;
}}
QPushButton#exportBtn:hover {{
    background-color: {T.BLUE_DIM};
}}
QLineEdit {{
    background-color: white;
    color: {T.TEXT_PRIMARY};
    border: 1px solid {T.BORDER};
    border-radius: 4px;
    padding: 6px 10px;
    font-size: 13px;
}}
QLineEdit:focus {{
    border-color: {T.BLUE};
}}
QComboBox {{
    background-color: white;
    color: {T.TEXT_PRIMARY};
    border: 1px solid {T.BORDER};
    border-radius: 4px;
    padding: 6px 10px;
    font-size: 13px;
}}
QComboBox QAbstractItemView {{
    background-color: white;
    color: {T.TEXT_PRIMARY};
    border: 1px solid {T.BORDER};
    selection-background-color: {T.BLUE};
    selection-color: white;
}}
QGroupBox {{
    background-color: {T.BG_DARK};
    border: 1px solid {T.BORDER};
    border-radius: 8px;
    margin-top: 14px;
    padding: 16px 12px 12px 12px;
    font-weight: bold;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 2px 12px;
    color: {T.BLUE};
}}
QTabWidget::pane {{
    background-color: {T.BG_DARK};
    border: 1px solid {T.BORDER};
    border-radius: 0 0 8px 8px;
}}
QTabBar::tab {{
    background-color: {T.BG_MID};
    color: {T.TEXT_SECONDARY};
    border: 1px solid {T.BORDER};
    border-bottom: none;
    padding: 8px 16px;
    margin-right: 2px;
    border-radius: 6px 6px 0 0;
}}
QTabBar::tab:selected {{
    background-color: {T.BG_DARK};
    color: {T.BLUE};
    border-bottom: 2px solid {T.BLUE};
    font-weight: bold;
}}
QSplitter::handle {{
    background-color: {T.BORDER};
}}
QProgressBar {{
    background-color: {T.BG_MID};
    border: 1px solid {T.BORDER};
    border-radius: 4px;
    text-align: center;
    color: {T.TEXT_PRIMARY};
    height: 20px;
}}
QProgressBar::chunk {{
    background-color: {T.BLUE};
}}
QStatusBar {{
    background-color: {T.BG_DARK};
    color: {T.TEXT_SECONDARY};
    border-top: 1px solid {T.BORDER};
    font-size: 12px;
}}
QTableWidget {{
    background-color: white;
    color: {T.TEXT_PRIMARY};
    gridline-color: {T.BORDER_LIGHT};
    border: 1px solid {T.BORDER};
}}
QTableWidget QHeaderView::section {{
    background-color: {T.BG_DARKEST};
    color: {T.TEXT_PRIMARY};
    border: 1px solid {T.BORDER};
    padding: 4px;
}}
QScrollArea {{
    border: none;
    background-color: transparent;
}}
QGraphicsView {{
    background-color: {T.BG_CANVAS};
    border: 1px solid {T.BORDER};
    border-radius: 6px;
}}
QLabel#sectionTitle {{
    color: {T.BLUE};
    font-size: 14px;
    font-weight: bold;
    padding: 4px 0;
}}
QLabel#blockTitle {{
    color: {T.TEXT_PRIMARY};
    font-size: 15px;
    font-weight: bold;
}}
QFrame#separator {{
    background-color: {T.BORDER};
    max-height: 1px;
}}
"""
