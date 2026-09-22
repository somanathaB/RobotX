"""Localization: turning sensor fixes into the robot's position and heading.

Sits between `robotx.hardware` (GPS acquisition) and `robotx.navigation`
(routing), so neither has to know about the other's concerns.
"""
