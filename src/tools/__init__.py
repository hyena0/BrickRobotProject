"""Deterministic geometry tools (Stages 1-4)."""

from .axes import axes_intersect
from .geometry import (axis_world, check_gear_mesh, find_connections,
                       get_parts, load_catalog, rigid_groups, trace_axle)
from .transmission import axle_carrier, gear_graph, solve_transmission

__all__ = ["axes_intersect", "axis_world", "axle_carrier",
           "check_gear_mesh", "find_connections", "gear_graph",
           "get_parts", "load_catalog", "rigid_groups",
           "solve_transmission", "trace_axle"]
