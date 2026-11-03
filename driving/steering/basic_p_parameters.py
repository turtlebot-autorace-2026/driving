"""basic_p의 좌우 normal_angular_divisor만 ROS 파라미터로 관리한다."""

from math import isfinite
from typing import Any, List

from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter

from driving.interfaces import SteeringController
from driving.steering.basic_p import AngularDivisors, Basic_P


PARAMETER_NAMES = {
    'left': 'basic_p.left.normal_angular_divisor',
    'right': 'basic_p.right.normal_angular_divisor',
}


class BasicPParameters:
    def __init__(self, node: Node):
        self.node = node
        defaults = Basic_P.default_angular_divisors()
        for side, name in PARAMETER_NAMES.items():
            node.declare_parameter(name, getattr(defaults, side), ParameterDescriptor(
                description='Angular velocity divisor for the ' + side + ' lane in basic_p.',
                additional_constraints='Finite double > 0.',
            ))
        # 시작 시 --ros-args/YAML로 덮어쓴 값도 검사한다.
        self.snapshot()
        node.add_on_set_parameters_callback(self.validate)

    @staticmethod
    def _check(name: str, value: Any):
        if type(value) is not float or not isfinite(value) or value <= 0:
            raise ValueError(name + ' must be a finite double > 0.')

    def snapshot(self) -> AngularDivisors:
        values = {}
        for side, name in PARAMETER_NAMES.items():
            value = self.node.get_parameter(name).value
            self._check(name, value)
            values[side] = value
        return AngularDivisors(**values)

    def validate(self, parameters: List[Parameter]) -> SetParametersResult:
        try:
            for parameter in parameters:
                if parameter.name in PARAMETER_NAMES.values():
                    self._check(parameter.name, parameter.value)
        except ValueError as error:
            return SetParametersResult(successful=False, reason=str(error))
        # 검증 후 다른 콜백이 거부할 수 있으므로 여기서는 객체를 변경하지 않는다.
        return SetParametersResult(successful=True)

    def configure_controller(
        self, controller: SteeringController[Any],
    ) -> SteeringController[Any]:
        # 다른 팀 클래스(상속한 별도 구현 포함)의 설정/동작을 덮어쓰지 않는다.
        if type(controller) is not Basic_P:
            return controller
        divisors = self.snapshot()
        if controller.angular_divisors == divisors:
            return controller
        return Basic_P(lane_mode=controller.lane_mode, angular_divisors=divisors)
