"""Compatibility entry for the filename requested in tasks.md.

Python import statements cannot start a module name with a digit; import the
same controller from inspire_sdk.target_pos_control or inspire_sdk instead.
"""
from .target_pos_control import TargetPositionController

__all__ = ["TargetPositionController"]
