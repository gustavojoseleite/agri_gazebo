"""Camada ROS da degradação: os parâmetros de um nó, o nó de um sensor e o executável.

`DegradationParameters` liga os parâmetros do nó ao sensor do núcleo; `SensorNode` é o nó de
um sensor; `main` sobe os nós `gnss`, `imu` e `odometry` num só processo.
"""
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from agri_sensor_degradation.core import MODES, Trigger
from agri_sensor_degradation.sensors import GnssSensor, ImuSensor, OdometrySensor


class DegradationParameters:
    """Declara os parâmetros de degradação de um nó e os aplica ao sensor."""

    def __init__(self, node, sensor):
        self.node = node
        self.sensor = sensor
        self.position = None
        self._truth_sub = None
        self._truth_topic = ''

        fraction_range = FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)
        node.declare_parameter('seed', int(sensor.seed))
        node.declare_parameter('loss.enabled', False)
        node.declare_parameter('loss.fraction', 1.0, ParameterDescriptor(
            description='Fração das mensagens perdidas, de 0 a 1.',
            floating_point_range=[fraction_range]))
        node.declare_parameter('loss.trigger', 'always')
        node.declare_parameter('loss.window_start', 0.0)
        node.declare_parameter('loss.window_end', 0.0)
        node.declare_parameter('loss.zone', Parameter.Type.DOUBLE_ARRAY, ParameterDescriptor(
            description='Polígono da zona: x0, y0, x1, y1, ...'))
        node.declare_parameter('ground_truth_topic', '')

        self._state_pub = node.create_publisher(DiagnosticArray, 'degradation/state', 10)
        node.add_on_set_parameters_callback(self._validate)
        node.add_post_set_parameters_callback(self._on_changed)
        self._refresh()
        node.create_timer(1.0, self.publish_state)

    def _value(self, name):
        return self.node.get_parameter(name).value

    def _validate(self, parameters):
        """Recusa fração fora de [0, 1] e gatilho desconhecido."""
        for parameter in parameters:
            if parameter.name == 'loss.trigger' and parameter.value not in MODES:
                return SetParametersResult(
                    successful=False, reason=f'gatilho desconhecido: {parameter.value!r}')
            if parameter.name == 'loss.fraction' and not 0.0 <= parameter.value <= 1.0:
                return SetParametersResult(
                    successful=False, reason='a fração deve estar entre 0 e 1')
        return SetParametersResult(successful=True)

    def _on_changed(self, parameters):
        """Reaplica os parâmetros ao sensor depois de uma mudança em execução."""
        previous_seed = self.sensor.seed
        self._refresh()
        if any(p.name == 'seed' for p in parameters) and self.sensor.seed != previous_seed:
            self.sensor.reseed(self.sensor.seed)
        self.publish_state()

    def _refresh(self):
        """Lê todos os parâmetros e os copia para o sensor."""
        loss = self.sensor.loss
        loss.enabled = bool(self._value('loss.enabled'))
        loss.fraction = float(self._value('loss.fraction'))
        # Sem valor, o parâmetro da zona fica não inicializado: vale como zona vazia.
        empty = Parameter('loss.zone', Parameter.Type.DOUBLE_ARRAY, [])
        flat = list(self.node.get_parameter_or('loss.zone', empty).value)
        zone = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]
        if len(zone) < 3:
            zone = []
        window = (float(self._value('loss.window_start')), float(self._value('loss.window_end')))
        loss.trigger = Trigger(self._value('loss.trigger'), window=window, zone=zone)
        seed = int(self._value('seed'))
        if seed != self.sensor.seed:
            self.sensor.reseed(seed)
        self._subscribe_truth(self._value('ground_truth_topic'))

    def _subscribe_truth(self, topic):
        """Assina a pose real do veículo, se houver tópico."""
        if topic == self._truth_topic:
            return
        if self._truth_sub is not None:
            self.node.destroy_subscription(self._truth_sub)
            self._truth_sub = None
        self._truth_topic = topic
        if topic:
            self._truth_sub = self.node.create_subscription(
                PoseStamped, topic, self._on_truth, 10)

    def _on_truth(self, msg):
        self.position = (msg.pose.position.x, msg.pose.position.y)

    def apply(self, msg):
        """Degrada a mensagem usando o carimbo dela como instante; `None` se ela se perdeu."""
        stamp = msg.header.stamp
        return self.sensor.apply(msg, stamp.sec + stamp.nanosec * 1e-9, self.position)

    def publish_state(self):
        """Publica o estado do sensor em `degradation/state`."""
        status = DiagnosticStatus(name=self.node.get_fully_qualified_name(),
                                  level=DiagnosticStatus.OK)
        status.values = [KeyValue(key=k, value=v) for k, v in self.sensor.state().items()]
        message = DiagnosticArray()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.status = [status]
        self._state_pub.publish(message)


class SensorNode(Node):
    """Nó de um sensor: lê o tópico ideal e publica o normal, degradado conforme os parâmetros."""

    def __init__(self, sensor, node_name=None, **kwargs):
        super().__init__(node_name or sensor.name, **kwargs)
        self.sensor = sensor
        self._publisher = self.create_publisher(sensor.msg_type, sensor.output_topic, 10)
        self.degradation = DegradationParameters(self, sensor)
        self.create_subscription(sensor.msg_type, sensor.input_topic, self._on_message,
                                 qos_profile_sensor_data)

    def _on_message(self, msg):
        result = self.degradation.apply(msg)
        if result is not None:
            self._publisher.publish(result)


def main(args=None):
    """Sobe os nós `gnss`, `imu` e `odometry` num só processo."""
    rclpy.init(args=args)
    nodes = [SensorNode(GnssSensor()), SensorNode(ImuSensor()), SensorNode(OdometrySensor())]
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
