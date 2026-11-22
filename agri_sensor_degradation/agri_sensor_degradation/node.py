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

from agri_sensor_degradation import profiles
from agri_sensor_degradation.core import MODES, SCALAR_VALUES, Trigger
from agri_sensor_degradation.sensors import GnssSensor, ImuSensor, OdometrySensor


# Faixa dos valores que não são por grandeza.
SCALAR_RANGES = {
    ('jumps', 'rate'): (0.0, 1.0),
    ('jumps', 'duration'): (0.0, 30.0),
    ('jumps', 'min_fraction'): (0.0, 1.0),
    ('jumps', 'duration_min'): (0.0, 30.0),
    ('markov', 'tau'): (1.0, 3600.0),
    ('rate', 'max_hz'): (0.0, 100.0),
    ('delay', 'seconds'): (0.0, 2.0),
}


def _channel_range(effect, full_scale):
    """Faixa do valor de um efeito para uma grandeza, a partir do fundo de escala."""
    return {
        'scale': (0.5, 1.5),
        'bias': (-full_scale, full_scale),
        'drift': (-full_scale / 10.0, full_scale / 10.0),
        'markov': (0.0, full_scale),
        'noise': (0.0, full_scale),
        'jumps': (0.0, 10.0 * full_scale),
    }[effect]


def _polygon(flat):
    """Converte a lista plana `x0, y0, x1, y1, ...` em pontos; com menos de três, zona vazia."""
    zone = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]
    return zone if len(zone) >= 3 else []


class DegradationParameters:
    """Declara os parâmetros de degradação de um nó e os aplica ao sensor."""

    def __init__(self, node, sensor):
        self.node = node
        self.sensor = sensor
        self.position = None
        self._truth_sub = None
        self._truth_topic = ''
        self._defaults = {}
        self._profile_timer = None

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
        # Os efeitos gerais exigem grandezas; os próprios do sensor valem sempre.
        self._effects = {name: effect for name, effect in sensor.effects.items()
                         if sensor.channels or name in sensor.extra_effects}
        if self._effects:
            self._declare_effects()
            if hasattr(sensor, 'vertical_factor'):
                node.declare_parameter('vertical_factor', float(sensor.vertical_factor),
                                       self._range(1.0, 5.0, 'Peso da altura nos efeitos.'))
            node.declare_parameter('profile', '', ParameterDescriptor(
                description='Perfil de degradação por nome; vazio deixa os padrões.'))

        self._state_pub = node.create_publisher(DiagnosticArray, 'degradation/state', 10)
        node.add_on_set_parameters_callback(self._validate)
        node.add_post_set_parameters_callback(self._on_changed)
        self._refresh()
        node.create_timer(1.0, self.publish_state)
        if self._effects and self._value('profile'):
            self._apply_profile()

    @staticmethod
    def _range(low, high, description=''):
        """Descritor de um valor real com faixa, que vira slider no `rqt_reconfigure`."""
        return ParameterDescriptor(description=description, floating_point_range=[
            FloatingPointRange(from_value=float(low), to_value=float(high), step=0.0)])

    def _declare_effects(self):
        """Declara, para cada efeito, o gatilho e os valores, com a faixa de cada um."""
        node = self.node
        for name, effect in self._effects.items():
            self._defaults[f'{name}.enabled'] = False
            node.declare_parameter(f'{name}.enabled', False)
            node.declare_parameter(f'{name}.trigger', 'always')
            node.declare_parameter(f'{name}.window_start', 0.0)
            node.declare_parameter(f'{name}.window_end', 0.0)
            node.declare_parameter(f'{name}.zone', Parameter.Type.DOUBLE_ARRAY, ParameterDescriptor(
                description='Polígono da zona: x0, y0, x1, y1, ...'))
            for key, default in effect.values.items():
                self._defaults[f'{name}.{key}'] = float(default)
                extra = getattr(self.sensor, 'extra_ranges', {})
                if (name, key) in extra:
                    low, high = extra[(name, key)]
                elif key in SCALAR_VALUES.get(name, {}):
                    low, high = SCALAR_RANGES[(name, key)]
                else:
                    full_scale = self.sensor.channels[key].full_scale
                    low, high = _channel_range(name, full_scale)
                node.declare_parameter(f'{name}.{key}', float(default),
                                       self._range(low, high))

    def _value(self, name):
        return self.node.get_parameter(name).value

    def _validate(self, parameters):
        """Recusa fração fora de [0, 1], gatilho e perfil desconhecidos."""
        for parameter in parameters:
            if parameter.name == 'profile' and parameter.value and \
                    parameter.value not in profiles.names(self.sensor.name):
                return SetParametersResult(
                    successful=False, reason=f'perfil desconhecido: {parameter.value!r}')
            if parameter.name.endswith('.trigger') and parameter.value not in MODES:
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
        if any(p.name == 'profile' for p in parameters) and self._profile_timer is None:
            # Gravar parâmetros dentro do próprio callback não é seguro: fica para um timer.
            self._profile_timer = self.node.create_timer(0.01, self._apply_profile_once)

    def _apply_profile_once(self):
        """Aplica o perfil escolhido e destrói o timer que a chamou."""
        self.node.destroy_timer(self._profile_timer)
        self._profile_timer = None
        self._apply_profile()

    def _apply_profile(self):
        """Grava os padrões de todos os efeitos e, por cima, os valores do perfil."""
        name = self._value('profile')
        values = dict(self._defaults)
        if name:
            values.update(profiles.load(self.sensor.name, name))
        self.node.set_parameters([Parameter(key, value=value) for key, value in values.items()])

    def _refresh(self):
        """Lê todos os parâmetros e os copia para o sensor."""
        loss = self.sensor.loss
        loss.enabled = bool(self._value('loss.enabled'))
        loss.fraction = float(self._value('loss.fraction'))
        loss.trigger = self._trigger('loss')
        if self._effects:
            self._refresh_effects()
        seed = int(self._value('seed'))
        if seed != self.sensor.seed:
            self.sensor.reseed(seed)
        self._subscribe_truth(self._value('ground_truth_topic'))

    def _trigger(self, prefix):
        """Monta o gatilho de `<prefix>.trigger`, `.window_*` e `.zone`."""
        # Sem valor, o parâmetro da zona fica não inicializado: vale como zona vazia.
        empty = Parameter(f'{prefix}.zone', Parameter.Type.DOUBLE_ARRAY, [])
        flat = list(self.node.get_parameter_or(f'{prefix}.zone', empty).value)
        window = (float(self._value(f'{prefix}.window_start')),
                  float(self._value(f'{prefix}.window_end')))
        return Trigger(self._value(f'{prefix}.trigger'), window=window, zone=_polygon(flat))

    def _refresh_effects(self):
        """Copia os parâmetros dos efeitos para o sensor."""
        for name, effect in self._effects.items():
            effect.enabled = bool(self._value(f'{name}.enabled'))
            effect.trigger = self._trigger(name)
            for key in effect.values:
                effect.values[key] = float(self._value(f'{name}.{key}'))
        if hasattr(self.sensor, 'vertical_factor'):
            self.sensor.vertical_factor = float(self._value('vertical_factor'))

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
        self.create_timer(0.01, self._release)

    def _release(self):
        """Publica as mensagens que o atraso reteve e cuja hora chegou."""
        now = self.get_clock().now().nanoseconds * 1e-9
        for msg in self.sensor.release(now):
            self._publisher.publish(msg)

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
