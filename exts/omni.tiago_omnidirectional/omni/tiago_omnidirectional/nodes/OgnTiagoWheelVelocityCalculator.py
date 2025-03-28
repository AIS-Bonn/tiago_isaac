import numpy as np
from omni.isaac.core_nodes import BaseResetNode
from omni.tiago_omnidirectional.TiagoWheelVelocityCalculator import calculate_tiago_wheel_velocities

class OgnTiagoWheelVelocityCalculatorInternalState(BaseResetNode):
    def __init__(self):
        self.wheel_radius = 0.0762
        self.wheel_positions = np.array([0.244, 0.22317])
        self.node = None
        self.graph_id = None
        super().__init__(initialize=False)

    def initialize_controller(self) -> None:
        
        self.initialized = True

    def forward(self, command: np.ndarray):
        velocities = calculate_tiago_wheel_velocities(
            vx = command[0],
            vy = command[1],
            omega = command[2],
            wheel_radius = self.wheel_radius,
            wheel_positions = self.wheel_positions
        )
        return velocities



class OgnTiagoWheelVelocityCalculator:
    """
    nodes for moving an articulated robot with joint commands
    """

    @staticmethod
    def internal_state():
        return OgnTiagoWheelVelocityCalculatorInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state

        try:
            if not state.initialized:
                state.wheel_radius = db.inputs.wheelRadius
                state.wheel_positions = db.inputs.wheelPositions

                state.initialize_controller()

            joint_velocities = state.forward(np.array(db.inputs.inputVelocity))
            db.outputs.jointVelocityCommand = joint_velocities

        except Exception as error:
            db.log_warning(str(error))
            return False

        return True
