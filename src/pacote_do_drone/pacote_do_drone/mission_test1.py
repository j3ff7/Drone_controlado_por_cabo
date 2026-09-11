import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleCommand,
    TrajectorySetpoint,
    VehicleStatus,
    VehicleLocalPosition,
)
import matplotlib.pyplot as plt
import numpy as np


class Browser(Node):
    def __init__(self):
        super().__init__('browser')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscriber da posição real do drone (feedback do PX4)
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            qos_profile
        )
        self.current_position = None  # [x, y, z] mais recente recebido do PX4

        self.waypoints = [
            [0.0, 0.0, -5.0],
            [40.0, 0.0, -5.0],
            [40.0, 30.0, -5.0],
            [-20.0, 30.0, -5.0],
            [-20.0, 15.0, -5.0],
            [0.0, 15.0, -5.0],
            [0.0, 0.0, -5.0],
        ]

        self.current_waypoint_index = 0
        self.cycle_count = 0
        self.counter_star = 0

        # --- Log de dados para o gráfico de erro ---
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.log_time = []
        self.log_setpoint = []   # [x, y, z] alvo em cada instante
        self.log_position = []   # [x, y, z] posição real em cada instante
        self.log_error = []      # erro escalar (norma euclidiana) em cada instante

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Nó 'browser' iniciado. Publicando comandos de voo para o PX4.")

    def position_callback(self, msg):
        # VehicleLocalPosition já vem em NED, mesmo referencial dos waypoints
        self.current_position = [msg.x, msg.y, msg.z]

    def timer_callback(self):
        self.publish_offboard_mode()

        current_waypoint = self.waypoints[self.current_waypoint_index]
        self.publish_trajectory_setpoint(current_waypoint)

        # Registra o erro só depois que já recebemos ao menos uma posição real
        if self.current_position is not None:
            t = self.get_clock().now().nanoseconds / 1e9 - self.start_time
            error_vec = np.array(current_waypoint) - np.array(self.current_position)
            error_norm = float(np.linalg.norm(error_vec))

            self.log_time.append(t)
            self.log_setpoint.append(list(current_waypoint))
            self.log_position.append(list(self.current_position))
            self.log_error.append(error_norm)

        if self.counter_star < 10:
            self.counter_star += 1
        elif self.counter_star == 10:
            self.get_logger().info("Iniciando a missão: entrando em modo OFFBOARD e armando.")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.counter_star += 1
            self.get_logger().info(f"Missão iniciada. Indo para o ponto 1: {current_waypoint}")

        if self.counter_star > 10:
            self.cycle_count += 1

            if self.cycle_count >= 150:
                self.cycle_count = 0

                if self.current_waypoint_index < len(self.waypoints) - 1:
                    self.current_waypoint_index += 1
                    next_waypoint = self.waypoints[self.current_waypoint_index]
                    self.get_logger().info(f"Indo para o próximo ponto: {next_waypoint}")
                else:
                    self.get_logger().info("Missão concluída. Último ponto alcançado.")
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
                    self.get_logger().info("Motores desligados. Encerrando o nó.")
                    self.timer.cancel()
                    self.plot_results()
                    rclpy.shutdown()

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, waypoint):
        msg = TrajectorySetpoint()
        msg.position = [float(waypoint[0]), float(waypoint[1]), float(waypoint[2])]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def plot_results(self):
        if len(self.log_time) == 0:
            self.get_logger().warn("Nenhum dado de posição foi registrado, gráfico não gerado.")
            return

        t = np.array(self.log_time)
        setpoints = np.array(self.log_setpoint)   # shape (N, 3)
        positions = np.array(self.log_position)   # shape (N, 3)
        error = np.array(self.log_error)          # shape (N,)

        fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

        # Gráfico 1: posição real vs setpoint, por eixo
        labels = ['X (Norte)', 'Y (Leste)', 'Z (Baixo)']
        for i in range(3):
            axs[0].plot(t, setpoints[:, i], '--', label=f'Setpoint {labels[i]}')
            axs[0].plot(t, positions[:, i], '-', label=f'Real {labels[i]}')
        axs[0].set_ylabel('Posição (m)')
        axs[0].set_title('Posição real vs. Setpoint')
        axs[0].legend(loc='upper right', fontsize=8, ncol=3)
        axs[0].grid(True)

        # Gráfico 2: erro (norma euclidiana) ao longo do tempo
        axs[1].plot(t, error, color='red')
        axs[1].set_xlabel('Tempo (s)')
        axs[1].set_ylabel('Erro de posição (m)')
        axs[1].set_title('Erro de posição (norma euclidiana) ao longo do tempo')
        axs[1].grid(True)

        fig.tight_layout()
        output_path = 'grafico_erro_posicao.png'
        fig.savefig(output_path, dpi=150)
        self.get_logger().info(f"Gráfico salvo em: {output_path}")


def main(args=None):
    rclpy.init(args=args)
    node = Browser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nó 'browser' encerrado pelo usuário.")
        node.plot_results()  # gera o gráfico mesmo se interrompido manualmente
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()