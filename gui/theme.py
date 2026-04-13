"""
theme.py — Dark theme colors and stylesheet for the Simulink GUI.

Centralized design tokens for consistent dark-mode styling.
"""


class Theme:
    """Centralized dark theme colors inspired by Simulink/MATLAB."""
    # Base backgrounds
    BG_DARKEST = "#0d1117"
    BG_DARK = "#161b22"
    BG_MID = "#1c2333"
    BG_LIGHT = "#21262d"
    BG_CANVAS = "#131a27"

    # Borders
    BORDER = "#30363d"
    BORDER_LIGHT = "#3d444d"

    # Text
    TEXT_PRIMARY = "#e6edf3"
    TEXT_SECONDARY = "#8b949e"
    TEXT_MUTED = "#6e7681"

    # Accent colors
    BLUE = "#58a6ff"
    BLUE_DIM = "#1f6feb"
    GREEN = "#3fb950"
    GREEN_DIM = "#238636"
    ORANGE = "#f0883e"
    RED = "#f85149"
    PURPLE = "#bc8cff"
    CYAN = "#76e3ea"
    YELLOW = "#e3b341"

    # Block-specific colors
    BLOCK_INPUT = "#1a3a5c"
    BLOCK_INPUT_BORDER = "#58a6ff"
    BLOCK_REF = "#1a3c2e"
    BLOCK_REF_BORDER = "#3fb950"
    BLOCK_CTRL = "#3a2a1a"
    BLOCK_CTRL_BORDER = "#f0883e"
    BLOCK_PLANT = "#2a1a3a"
    BLOCK_PLANT_BORDER = "#bc8cff"
    BLOCK_ADAPT = "#3a1a2a"
    BLOCK_ADAPT_BORDER = "#f85149"
    BLOCK_SCOPE = "#1a2a3a"
    BLOCK_SCOPE_BORDER = "#76e3ea"

    # Matplotlib colors
    MPL_BG = "#0d1117"
    MPL_AXES = "#161b22"
    MPL_GRID = "#21262d"
    MPL_TEXT = "#c9d1d9"
    MPL_TICK = "#8b949e"


def build_stylesheet() -> str:
    """Generate the full QSS stylesheet."""
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
    font-weight: 600;
    font-size: 13px;
}}
QPushButton:hover {{
    background-color: {T.BORDER};
    border-color: {T.BORDER_LIGHT};
}}
QPushButton:pressed {{
    background-color: {T.BG_MID};
}}
QPushButton:disabled {{
    background-color: {T.BG_DARK};
    color: {T.TEXT_MUTED};
    border-color: {T.BG_LIGHT};
}}
QPushButton#runBtn {{
    background-color: {T.GREEN_DIM};
    border-color: {T.GREEN};
    color: white;
}}
QPushButton#runBtn:hover {{
    background-color: {T.GREEN};
}}
QPushButton#runBtn:disabled {{
    background-color: {T.BG_LIGHT};
    border-color: {T.BORDER};
    color: {T.TEXT_MUTED};
}}
QPushButton#exportBtn {{
    background-color: {T.BLUE_DIM};
    border-color: {T.BLUE};
    color: white;
}}
QPushButton#exportBtn:hover {{
    background-color: {T.BLUE};
}}
QLineEdit {{
    background-color: {T.BG_MID};
    color: {T.TEXT_PRIMARY};
    border: 1px solid {T.BORDER};
    border-radius: 4px;
    padding: 6px 10px;
    font-size: 13px;
    selection-background-color: {T.BLUE_DIM};
}}
QLineEdit:focus {{
    border-color: {T.BLUE};
}}
QComboBox {{
    background-color: {T.BG_MID};
    color: {T.TEXT_PRIMARY};
    border: 1px solid {T.BORDER};
    border-radius: 4px;
    padding: 6px 10px;
    font-size: 13px;
}}
QComboBox:hover {{
    border-color: {T.BLUE};
}}
QComboBox::drop-down {{
    border: none;
    padding-right: 8px;
}}
QComboBox QAbstractItemView {{
    background-color: {T.BG_MID};
    color: {T.TEXT_PRIMARY};
    border: 1px solid {T.BORDER};
    selection-background-color: {T.BLUE_DIM};
}}
QGroupBox {{
    background-color: {T.BG_DARK};
    border: 1px solid {T.BORDER};
    border-radius: 8px;
    margin-top: 14px;
    padding: 16px 12px 12px 12px;
    font-weight: bold;
    font-size: 13px;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 2px 12px;
    color: {T.BLUE};
    font-size: 13px;
}}
QTabWidget::pane {{
    background-color: {T.BG_DARK};
    border: 1px solid {T.BORDER};
    border-radius: 0 0 8px 8px;
}}
QTabBar::tab {{
    background-color: {T.BG_LIGHT};
    color: {T.TEXT_SECONDARY};
    border: 1px solid {T.BORDER};
    border-bottom: none;
    padding: 8px 16px;
    margin-right: 2px;
    border-radius: 6px 6px 0 0;
    font-size: 12px;
    font-weight: 500;
}}
QTabBar::tab:selected {{
    background-color: {T.BG_DARK};
    color: {T.BLUE};
    border-bottom: 2px solid {T.BLUE};
    font-weight: 700;
}}
QTabBar::tab:hover:!selected {{
    background-color: {T.BG_MID};
    color: {T.TEXT_PRIMARY};
}}
QSplitter::handle {{
    background-color: {T.BORDER};
}}
QSplitter::handle:horizontal {{
    width: 2px;
}}
QSplitter::handle:vertical {{
    height: 2px;
}}
QProgressBar {{
    background-color: {T.BG_LIGHT};
    border: 1px solid {T.BORDER};
    border-radius: 4px;
    text-align: center;
    color: {T.TEXT_PRIMARY};
    font-size: 11px;
    height: 20px;
}}
QProgressBar::chunk {{
    background-color: {T.BLUE};
    border-radius: 3px;
}}
QStatusBar {{
    background-color: {T.BG_DARK};
    color: {T.TEXT_SECONDARY};
    border-top: 1px solid {T.BORDER};
    font-size: 12px;
    padding: 2px 8px;
}}
QTableWidget {{
    background-color: {T.BG_DARK};
    color: {T.TEXT_PRIMARY};
    gridline-color: {T.BORDER};
    border: 1px solid {T.BORDER};
    border-radius: 6px;
    font-size: 12px;
}}
QTableWidget::item {{
    padding: 4px 8px;
}}
QTableWidget::item:selected {{
    background-color: {T.BLUE_DIM};
}}
QHeaderView::section {{
    background-color: {T.BG_LIGHT};
    color: {T.TEXT_PRIMARY};
    border: 1px solid {T.BORDER};
    padding: 6px;
    font-weight: bold;
    font-size: 12px;
}}
QScrollArea {{
    border: none;
    background-color: transparent;
}}
QGraphicsView {{
    background-color: {T.BG_CANVAS};
    border: 1px solid {T.BORDER};
    border-radius: 8px;
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
