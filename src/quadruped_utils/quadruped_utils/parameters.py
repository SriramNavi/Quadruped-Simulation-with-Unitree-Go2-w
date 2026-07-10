"""Parameter helpers for rclpy nodes."""

from __future__ import annotations

from typing import Any

from rclpy.node import Node


def declare_parameter_if_missing(node: Node, name: str, default_value: Any) -> Any:
    """Declare a parameter only when it has not already been declared."""
    if not node.has_parameter(name):
        node.declare_parameter(name, default_value)
    return node.get_parameter(name).value
