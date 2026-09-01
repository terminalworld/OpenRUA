"""GraphNode: the rclpy node that carries the robot's whole surface.

Wires the publish side (sensors) and the command side (ports) onto one
node and re-publishes current state on a wall timer (observation never
advances the world). The env, config, and sim-owner thread arrive as
constructor parameters; nothing here knows how the world was built or
what counts as success.

The on-graph node name stays ``robot_bridge`` (agent-visible surface;
frozen).
"""

from __future__ import annotations

from rclpy.node import Node

from .controllers import CommandPorts
from .sensors import SensorPublishers

REPUBLISH_PERIOD_S = 0.5  # wall-clock re-publish of current state (no step)


class GraphNode(Node):
    def __init__(self, env, cfg: dict, sim):
        super().__init__("robot_bridge")
        self.sensors = SensorPublishers(self, env, cfg, sim=sim)
        self.ports = CommandPorts(
            self, env, cfg, on_step=self.sensors.publish, sim=sim
        )
        # Observation never advances the world: republish on a wall timer.
        self.create_timer(REPUBLISH_PERIOD_S, self.sensors.publish)

    def refresh(self) -> None:
        """Re-align the graph to the world after it changed under us:
        re-apply controller tuning (the substrate rebuilds controllers
        when the world is rebuilt) and push current state to the topics.
        Why the world changed is not this side's business."""
        self.ports.apply_tuning()
        self.sensors.publish()
