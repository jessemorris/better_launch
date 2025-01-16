from typing import Any, Callable
from logging import Logger
from enum import IntEnum
from collections import deque

from lifecycle_msgs.msg import TransitionEvent, State, Transition
from lifecycle_msgs.srv import ChangeState as ChangeLifecycleState

from .node import Node


class LifecycleStage(IntEnum):
    PRISTINE = 0
    CONFIGURED = 1
    ACTIVE = 2
    FINALIZED = 3


_stage_to_state_map = {
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
    ]
}


class LifecycleNode(Node):
    def __init__(
        self,
        launcher,
        executable: str,
        name: str,
        node_args: dict[str, Any] = None,
        target_stage: int = LifecycleStage.PRISTINE,
        *,
        logger: Logger = None,
        remap: dict[str, str] = None,
        env: dict[str, str] = None,
        on_exit: Callable = None,
        max_respawns: int = 0,
        respawn_delay: float = 0.0,
        use_shell: bool = False,
        emulate_tty: bool = False,
        stderr_to_stdout: bool = False,
    ):
        super().__init__(
            launcher,
            executable,
            name,
            node_args,
            logger=logger,
            remap=remap,
            env=env,
            on_exit=on_exit,
            max_respawns=max_respawns,
            respawn_delay=respawn_delay,
            use_shell=use_shell,
            emulate_tty=emulate_tty,
            stderr_to_stdout=stderr_to_stdout,
            start_immediately=True,
        )

        self.current_stage = LifecycleStage.PRISTINE
        self.current_state_id = State.PRIMARY_STATE_UNCONFIGURED

        self._state_sub = launcher.ros_adapter.ros_node.create_subscription(
            TransitionEvent,
            f"{self.fullname}/transition_event",
            self._on_transition_event,
            10,
        )

        # TODO these will fail until the node is truly started, but the async loop runs later
        self._transition_client = launcher.ros_adapter.ros_node.create_client(
            ChangeLifecycleState, f"{self.fullname}/change_state"
        )
        if not self._transition_client.wait_for_service(5.0):
            raise RuntimeError("Could not connect to lifecycle transition service")

        if target_stage > LifecycleStage.PRISTINE:
            self.transition(target_stage)

    def _on_transition_event(self, evt: TransitionEvent):
        self.current_state_id = evt.goal_state.id
        for key, val in _stage_to_state_map.items():
            if val == evt.goal_state.id:
                self.current_stage = key
                break
        else:
            self.current_stage = -1

    def _find_transition_path(start_state, goal_state):
        # Queue for BFS: (current_state, path_to_state)
        queue = deque([(start_state, [])])
        visited = set()

        while queue:
            current_state, path = queue.popleft()

            if current_state == goal_state:
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

    def _do_transition(self, transition_id: int) -> bool:
        req = ChangeLifecycleState.Request()
        req.transition.id = transition_id

        res = self._transition_client.call(req)
        return res.success

    def transition(self, target_stage: LifecycleStage) -> bool:
        # Figure out if and how we can get from our current state to the target state
        target_state_id = _stage_to_state_map[target_stage]
        transition_path = self._find_transition_path(self.current_state_id, target_state_id)

        if not transition_path:
            raise ValueError(f"Could not find a valid transition sequence for {self.current_stage}->{target_stage}")

        for transition in transition_path:
            if not self._do_transition(transition):
                self.logger.error(
                    f"Lifecycle transition {transition} for {self.name} towards stage {target_stage} failed"
                )
                return False

        # TODO we may have not received the transition update yet
        return True
