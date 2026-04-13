"""
control — Control algorithms for the 2-DoF satellite dish.

Contains:
    - controller: Computed Torque + MRAC (MIT Rule) controller
"""

from control.controller import ComputedTorqueController, MRACController
