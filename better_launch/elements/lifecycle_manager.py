from enum import IntEnum
from collections import deque

from ros2service.api import get_service_names_and_types
from lifecycle_msgs.msg import TransitionEvent, State, Transition
from lifecycle_msgs.srv import ChangeState as ChangeLifecycleState


class LifecycleStage(IntEnum):
    PRISTINE = 0
    CONFIGURED = 1
    ACTIVE = 2
    FINALIZED = 3


_stage_to_ros_state = {
    LifecycleStage.PRISTINE: State.PRIMARY_STATE_UNCONFIGURED,
    LifecycleStage.CONFIGURED: State.PRIMARY_STATE_INACTIVE,
    LifecycleStage.ACTIVE: State.PRIMARY_STATE_ACTIVE,
    LifecycleStage.FINALIZED: State.PRIMARY_STATE_FINALIZED,
}


# See https://design.ros2.org/articles/node_lifecycle.html
_transition_map = {
    State.PRIMARY_STATE_UNCONFIGURED: [
        (Transition.TRANSITION_CONFIGURE, State.PRIMARY_STATE_INACTIVE),
        (Transition.TRANSITION_UNCONFIGURED_SHUTDOWN, State.PRIMARY_STATE_FINALIZED),
    ],
    State.PRIMARY_STATE_INACTIVE: [
        (Transition.TRANSITION_INACTIVE_SHUTDOWN, State.PRIMARY_STATE_FINALIZED),
        (Transition.TRANSITION_CLEANUP, State.PRIMARY_STATE_UNCONFIGURED),
        (Transition.TRANSITION_ACTIVATE, State.PRIMARY_STATE_ACTIVE),
    ],
    State.PRIMARY_STATE_ACTIVE: [
        (Transition.TRANSITION_DEACTIVATE, State.PRIMARY_STATE_INACTIVE),
        (Transition.TRANSITION_ACTIVE_SHUTDOWN, State.PRIMARY_STATE_FINALIZED),
    ],
    State.PRIMARY_STATE_FINALIZED: [
        (Transition.TRANSITION_DESTROY, State.PRIMARY_STATE_UNKNOWN)
    ],
}


# Forward declaration to avoid cyclic imports
class AbstractNode:
    ...


class LifecycleManager:
    @classmethod
    def is_lifecycle(cls, node: AbstractNode) -> bool:
        # Whether a node supports lifecycle management can only be seen once the process has
        # started by checking the services it provides.
        if not node.is_ros2_connected:
            return None

        from better_launch import BetterLaunch

        bl = BetterLaunch.instance()
        if not bl:
            return None

        # Check if the node provides one of the key lifecycle services
        services = get_service_names_and_types(
            node=bl.shared_node, include_hidden_services=True
        )
        for srv_name, srv_types in services:
            if (
                srv_name == f"{node.fullname}/get_state"
                and "lifecycle_msgs/srv/GetState" in srv_types
            ):
                return True

        return False

    @classmethod
    def find_transition_path(cls, start_ros_state: int, goal_ros_state: int) -> list[int]:
        if start_ros_state == goal_ros_state:
            return []

        # Queue for BFS: (current_state, path_to_state)
        queue = deque([(start_ros_state, [])])
        visited = set()

        while queue:
            current_state, path = queue.popleft()

            if current_state == goal_ros_state:
                return path

            if current_state in visited:
                continue

            visited.add(current_state)

            # Explore transitions from the current state
            for transition, next_state in _transition_map.get(current_state, []):
                if next_state not in visited:
                    queue.append((next_state, path + [transition]))

        # No path found
        return []

    def __init__(self, node: AbstractNode):
        self._current_stage = LifecycleStage.PRISTINE
        self._current_ros_state: int = State.PRIMARY_STATE_UNCONFIGURED
        self._node = node

        from better_launch import BetterLaunch

        launcher = BetterLaunch.instance()
        self._state_sub = launcher.ros_adapter.ros_node.create_subscription(
            TransitionEvent,
            f"{self._node.fullname}/transition_event",
            self._on_transition_event,
            10,
        )

        self._transition_client = launcher.ros_adapter.ros_node.create_client(
            ChangeLifecycleState, f"{self._node.fullname}/change_state"
        )
        if not self._transition_client.wait_for_service(5.0):
            raise RuntimeError("Could not connect to lifecycle transition service")

    @property
    def current_stage(self) -> LifecycleStage:
        return self._current_stage

    @property
    def ros_state(self) -> int:
        return self._current_ros_state

    def transition(self, target_stage: LifecycleStage) -> bool:
        if target_stage == self.current_stage:
            return True

        # Figure out if and how we can get from our current state to the target state
        target_ros_state = _stage_to_ros_state[target_stage]
        transition_path = LifecycleManager.find_transition_path(
            self._current_ros_state, target_ros_state
        )

        if not transition_path:
            raise ValueError(
                f"Could not find a valid transition sequence for {self._current_stage}->{target_stage}"
            )

        for transition in transition_path:
            if not self._do_transition(transition):
                self.logger.error(
                    f"Lifecycle transition {transition} for {self.node.name} towards stage {target_stage} failed"
                )
                return False

        # TODO we may not have received the transition update yet
        return True

    def _on_transition_event(self, evt: TransitionEvent) -> None:
        self._current_ros_state = evt.goal_state.id
        for key, val in _stage_to_ros_state.items():
            if val == evt.goal_state.id:
                self._current_stage = key
                break
        else:
            self._current_stage = -1

    def _do_transition(self, transition_id: int) -> bool:
        req = ChangeLifecycleState.Request()
        req.transition.id = transition_id

        res = self._transition_client.call(req)
        return res.success
