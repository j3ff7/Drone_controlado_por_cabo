
import math
import threading

import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64MultiArray

from gz.transport13 import Node as GazeboNode
from gz.msgs10.model_pb2 import Model


class TelemetriaCabo(Node):

    def __init__(self):
        super().__init__('telemetria_cabo')

        # ==================================================
        # CONFIGURAÇÃO
        # ==================================================

        # Índices das poses identificados no teste anterior
        self.IDX_RAIZ = 9
        self.IDX_SEGMENTO_1 = 10

        # Nome da junta na ponta do cabo
        self.JUNTA_PONTA = 'joint_ponta'

        # Proteção para acesso simultâneo aos dados
        self.lock = threading.Lock()

        # Dados recebidos
        self.quaternion_joint_1 = None
        self.angulos_ponta = None

        # ==================================================
        # ASSINATURA DAS POSES DO GAZEBO
        # ==================================================

        self.subscription_poses = self.create_subscription(
            TFMessage,
            '/world/default/pose/info',
            self.callback_poses,
            10
        )

        # ==================================================
        # ASSINATURA DO GAZEBO TRANSPORT
        # ==================================================

        self.gz_node = GazeboNode()

        self.TOPICO_JOINT_STATE = (
            '/world/default/model/x500_cabo/joint_state'
        )

        self.subscription_juntas = self.gz_node.subscribe(
            Model,
            self.TOPICO_JOINT_STATE,
            self.callback_juntas
        )

        if not self.subscription_juntas:
            raise RuntimeError(
                'Não foi possível assinar o tópico joint_state.'
            )

        # ==================================================
        # PUBLICADOR ROS 2
        # ==================================================

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/angulos_cabo',
            10
        )

        # Publicação a 50 Hz
        self.timer = self.create_timer(
            0.02,
            self.publicar_telemetria
        )

        self.get_logger().info(
            'Nó de telemetria do cabo iniciado.'
        )

        self.get_logger().info(
            'Monitorando somente joint_1 e joint_ponta.'
        )

    # ======================================================
    # OPERAÇÕES COM QUATERNIONS
    # ======================================================

    def quaternion_conjugate(self, q):

        x, y, z, w = q

        return (-x, -y, -z, w)

    def quaternion_multiply(self, q1, q2):

        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2

        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2

        return (x, y, z, w)

    # ======================================================
    # CALLBACK DAS POSES
    # ======================================================

    def callback_poses(self, msg):

        transforms = msg.transforms

        if len(transforms) <= self.IDX_SEGMENTO_1:
            return

        # Orientação da raiz do cabo
        rot_raiz = transforms[
            self.IDX_RAIZ
        ].transform.rotation

        q_raiz = (
            rot_raiz.x,
            rot_raiz.y,
            rot_raiz.z,
            rot_raiz.w
        )

        # Orientação do primeiro segmento
        rot_segmento = transforms[
            self.IDX_SEGMENTO_1
        ].transform.rotation

        q_segmento = (
            rot_segmento.x,
            rot_segmento.y,
            rot_segmento.z,
            rot_segmento.w
        )

        # Quaternion relativo:
        # q_rel = conjugado(q_raiz) * q_segmento

        q_raiz_inv = self.quaternion_conjugate(q_raiz)

        q_rel = self.quaternion_multiply(
            q_raiz_inv,
            q_segmento
        )

        # Normalização do quaternion
        norma = math.sqrt(sum(v*v for v in q_rel))

        if norma < 1e-12:
            return

        q_rel = tuple(v / norma for v in q_rel)

        with self.lock:
            self.quaternion_joint_1 = q_rel

    # ======================================================
    # CALLBACK DOS ESTADOS DAS JUNTAS
    # ======================================================

    def callback_juntas(self, msg):

        for joint in msg.joint:

            if joint.name == self.JUNTA_PONTA:

                axis1 = joint.axis1.position
                axis2 = joint.axis2.position

                with self.lock:
                    self.angulos_ponta = (
                        axis1,
                        axis2
                    )

                break

    # ======================================================
    # PUBLICAÇÃO DA TELEMETRIA
    # ======================================================

    def publicar_telemetria(self):

        with self.lock:

            # Aguarda os dados das duas fontes
            if self.quaternion_joint_1 is None:
                return

            if self.angulos_ponta is None:
                return

            quaternion = self.quaternion_joint_1

            axis1, axis2 = self.angulos_ponta

        # Organização dos seis valores
        resultado = [
            quaternion[0],  # Quaternion X
            quaternion[1],  # Quaternion Y
            quaternion[2],  # Quaternion Z
            quaternion[3],  # Quaternion W
            axis1,          # Eixo 1 da joint_ponta
            axis2           # Eixo 2 da joint_ponta
        ]

        mensagem = Float64MultiArray()
        mensagem.data = resultado

        self.publisher.publish(mensagem)


# ==========================================================
# FUNÇÃO PRINCIPAL
# ==========================================================

def main(args=None):

    rclpy.init(args=args)

    node = TelemetriaCabo()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
