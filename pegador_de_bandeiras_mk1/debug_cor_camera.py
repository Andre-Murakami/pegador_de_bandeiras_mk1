#!/usr/bin/env python3
"""
Script de debug TEMPORÁRIO para identificar a cor exata (BGR) de um objeto
na câmera semântica /robot_cam/colored_map.

COMO USAR:
1. Rode este script em um terminal separado, com a simulação já rodando:
   $ python3 debug_cor_camera.py

2. Posicione o robô (manualmente, via teleop, ou pausando o controle_robo.py)
   de frente para o objeto que você quer identificar (ex: a base flag_deploy_zone),
   de forma que o objeto fique na parte de baixo e central da imagem da câmera
   (o pixel lido é o central na horizontal e o mais inferior na vertical -
   última linha da imagem -, que tende a capturar o chão mais próximo do robô).
   Aproxime bem o robô do objeto para garantir que ele preencha esse ponto.

3. O terminal vai imprimir continuamente a cor BGR desse pixel.
   Anote o valor quando estiver claramente apontado só para o objeto desejado.

4. Repita para cada objeto que precisar identificar.

5. Pressione Ctrl+C para encerrar.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DebugCorCamera(Node):
    def __init__(self):
        super().__init__('debug_cor_camera')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/robot_cam/colored_map', self.camera_callback, 10)
        self.get_logger().info("Aguardando imagens em /robot_cam/colored_map...")

    def camera_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[0], frame.shape[1]
        # Pixel central na horizontal, e mais inferior na vertical (última linha da imagem) -
        # útil para captar objetos no chão próximos ao robô, como a base flag_deploy_zone
        pixel_central_base = frame[h - 1, w // 2]
        b, g, r = int(pixel_central_base[0]), int(pixel_central_base[1]), int(pixel_central_base[2])
        self.get_logger().info(f"Pixel central-inferior -> BGR=({b}, {g}, {r}) | RGB=({r}, {g}, {b})")


def main(args=None):
    rclpy.init(args=args)
    node = DebugCorCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
