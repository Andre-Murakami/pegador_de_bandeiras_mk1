#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R

from cv_bridge import CvBridge
import cv2
import numpy as np
from enum import Enum
import random
import time


class Estados(Enum):
    EXPLORANDO = 1
    DESVIANDO_DE_OBSTACULO = 2
    NAVEGANDO_PARA_BANDEIRA = 3
    POSICIONANDO_PARA_COLETA = 4
    CAPTURANDO_BANDEIRA = 5
    RETORNANDO_PARA_BASE = 6
    RESETANDO_GARRA_BASE = 7
    SOLTANDO_BANDEIRA = 8
    DESVIANDO_DE_OBSTACULO_2 = 9

class Direcoes(Enum):
    ESQUERDA = 1
    DIREITA = -1


class ControleRobo(Node):

    def __init__(self):
        super().__init__('controle_robo')

        # Publisher para comando de velocidade
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Publisher para controle da garra
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom_gt', self.odom_callback, 10)
        self.create_subscription(Image, '/robot_cam/colored_map', self.camera_callback, 10)

        # Utilizado para converter imagens ROS -> OpenCV
        self.bridge = CvBridge()

        # Timer para enviar comandos continuamente
        self.timer = self.create_timer(0.1, self.move_robot)

        # Estado interno
        self.obstaculo_a_frente = False
        self.obstaculo_a_frente_esquerda = False
        self.obstaculo_a_frente_direita = False
        self.obstaculo_a_esquerda = False
        self.obstaculo_a_direita = False
        self.bandeira_a_frente = False

        self.distancias_frente_esquerda = []
        self.distancias_frente_direita = []
        self.distancias_esquerda = []
        self.distancias_direita = []
        
        self.porcentagem_bandeira_na_camera = 0
        self.pos_x_bandeira_camera = -1
        self.centro_x_camera = -1
        self.tempo_desviando = -1
        self.direcao_desvio = -1
        self.direcao_aleatoria = random.choice([Direcoes.DIREITA.value, Direcoes.ESQUERDA.value])
        self.direcao_bandeira = -1
        # Distância de detecção de obstáculos
        self.distancia_max_obstaculo_frente = 0.63
        self.distancia_max_obstaculo_lados = 0.50
        
        # Variável exclusiva para controle de velocidade (não afeta o desvio)
        self.distancia_limite_velocidade = 1.5

        # Range de detecção de obstáculos à frente (-30° a +30°)
        self.indices_frente_esquerda = list(range(0, 25))
        self.indices_frente_direita = list(range(335, 360))
        # Range de detecção de obstáculos à esquerda (30° a 90°)
        self.indices_esquerda = list(range(30, 90))
        # Range de detecção de obstáculos à diretia (-30° a -90°)
        self.indices_direita = list(range(270, 330))
        # Range de detecção de obstáculo atrás (usado na manobra de ré) - 150° a 210°
        self.indices_atras = list(range(150, 211))

        # Controle da posição atual da garra
        self.extensão_garra = 0.0
        self.junta_garra_direita = 0.0
        self.junta_garra_esquerda = 0.0
        self.junta_dedo_esquerdo = 0.0
        self.junta_dedo_direito = 0.0
        self.rotacao = 0.0
        self.bandeira_capturada = False
        # Controle explícito de sub-passo da captura (evita reler junta_garra/extensão logo após escrevê-las)
        self.passo_captura = 0
        # Contador de ciclos (0.1s cada) para substituir o time.sleep(4) dentro de CAPTURANDO_BANDEIRA,
        # evitando travar o timer/executor do ROS 2. 40 ciclos == ~4 segundos.
        # OBS: a duração efetiva por passo foi ampliada para 100 ciclos (~10s) e o comando passou a
        # ser reenviado a cada ciclo enquanto aguarda, para garantir a ordem correta dos movimentos.
        self.ciclos_espera_garra = 0
        # Distância aferida do mastro da bandeira via LiDAR no momento da captura (fallback: 4.2)
        self.distancia_extensao_garra = 4.2

        self.estado_atual = Estados.EXPLORANDO
        self.estado_anterior = None
        self.estado_origem_desvio = Estados.EXPLORANDO

        # Variáveis adicionadas para o monitoramento de odometria e retorno seguro
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.start_x = None
        self.start_y = None
        self.tempo_recuo = 0
        self.passo_soltura = 0
        self.tempo_recuo_base = 0 

        # Detecção de obstáculo atrás (usada na manobra de ré)
        self.distancia_max_obstaculo_tras = 0.50
        self.distancias_atras = []
        self.obstaculo_atras = False

        # Controle de oscilação no desvio de obstáculo e manobra de ré com curva
        self.contador_oscilacao = 0
        self.direcao_desvio_anterior = None
        self.x_ini_desvio2 = 0.0
        self.y_ini_desvio2 = 0.0
        self.yaw_ini_desvio2 = 0.0
        
        # Variáveis de oscilação na centralização
        self.contador_oscilacao_centralizacao = 0
        self.direcao_centralizacao_anterior = None

    def calcular_velocidade_dinamica(self):
        """Calcula velocidade baseada na variável exclusiva self.distancia_limite_velocidade."""
        caminho_livre = True
        for dist in self.distancias_frente_esquerda + self.distancias_frente_direita:
            if 0.1 < dist < self.distancia_limite_velocidade:
                caminho_livre = False
                break
        
        # Velocidade aumentada para 1.2 quando livre, 0.5 caso contrário
        return 1.5 if caminho_livre else 0.5

    def scan_callback(self, msg: LaserScan):
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        # DETECÇÃO DE OBSTACULOS À FRENTE - Índices de -35° a +35°
        self.distancias_frente_esquerda = [msg.ranges[i] for i in self.indices_frente_esquerda]
        self.obstaculo_a_frente_esquerda = self.distancias_frente_esquerda and min(self.distancias_frente_esquerda) < self.distancia_max_obstaculo_frente

        self.distancias_frente_direita = [msg.ranges[i] for i in self.indices_frente_direita]
        self.obstaculo_a_frente_direita = self.distancias_frente_direita and min(self.distancias_frente_direita) < self.distancia_max_obstaculo_frente

        self.obstaculo_a_frente = self.obstaculo_a_frente_direita or self.obstaculo_a_frente_esquerda

        # DETECÇÃO DE OBSTACULOS À ESQUERDA - Índices de +35° a +90°
        self.distancias_esquerda = [msg.ranges[i] for i in self.indices_esquerda]
        self.obstaculo_a_esquerda = self.distancias_esquerda and min(self.distancias_esquerda) < self.distancia_max_obstaculo_lados

        # DETECÇÃO DE OBSTACULOS À DIREITA - Índices de -35° a -90°
        self.distancias_direita = [msg.ranges[i] for i in self.indices_direita]
        self.obstaculo_a_direita = self.distancias_direita and min(self.distancias_direita) < self.distancia_max_obstaculo_lados

        # DETECÇÃO DE OBSTACULO ATRÁS - usada na manobra de ré do DESVIANDO_DE_OBSTACULO_2
        self.distancias_atras = [msg.ranges[i] for i in self.indices_atras]
        self.obstaculo_atras = self.distancias_atras and min(self.distancias_atras) < self.distancia_max_obstaculo_tras


    def imu_callback(self, msg: Imu):
        return


    def odom_callback(self, msg: Odometry):
        # Registra a posição linear atual
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Converte o quaternion para obter o ângulo de orientação (Yaw)
        q = msg.pose.pose.orientation
        rot = R.from_quat([q.x, q.y, q.z, q.w])
        self.current_yaw = rot.as_euler('xyz')[2]
        
        # Define a coordenada onde o robô "nasceu" (posição inicial da base)
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y


    def camera_callback(self, msg: Image):
        # Converte mensagem ROS para imagem OpenCV (BGR)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Calcula centro da camera (x)
        h, w = frame.shape[0], frame.shape[1]
        self.centro_x_camera = w//2

        # Verifica se tem bandeira na câmera:
        target_color = np.array([227, 73, 0])   # Cor da bandeira na câmera semantica
        mask_bandeira = cv2.inRange(frame, target_color, target_color)
        
        contours_bandeira, _ = cv2.findContours(mask_bandeira, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # zera a parte superior da mascara para ver apenas o mastro da bandeira
        mask_mastro = mask_bandeira.copy()
        limite_corte = int(h * 0.5)
        mask_mastro[0:limite_corte, :] = 0
        contours_mastro, _ = cv2.findContours(mask_mastro, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.bandeira_a_frente = len(contours_bandeira) > 0
        if self.bandeira_a_frente:
            cnt = max(contours_bandeira, key=cv2.contourArea)
            M_bandeira = cv2.moments(cnt)
            # usa mascara inteira para calcular porcentagem
            if M_bandeira['m00'] != 0:
                # calcula área da camera ocupada pela bandeira
                area_total = w*h
                area_bandeira = cv2.contourArea(cnt)
                self.porcentagem_bandeira_na_camera = (area_bandeira / area_total) * 100.0
            # usa mascara cortada (apenas mastro) com prioridade para encontrar a posição X da bandeira na câmera
            if len(contours_mastro) > 0:
                cnt_mastro = max(contours_mastro, key=cv2.contourArea)
                M_mastro = cv2.moments(cnt_mastro)
                if M_mastro['m00'] != 0:
                    self.pos_x_bandeira_camera = int(M_mastro['m10'] / M_mastro['m00'])
            # se nao encontrou mastro na imagem, usa a máscara inteira (mastro + bandeira)
            else:
                if M_bandeira['m00'] != 0:
                    self.pos_x_bandeira_camera = int(M_bandeira['m10'] / M_bandeira['m00'])
        else:
            self.bandeira_a_frente = False
            self.porcentagem_bandeira_na_camera = 0.0


    def estender_garra(self, bloqueante=True):
        msg = Float64MultiArray()
        self.extensão_garra = self.distancia_extensao_garra
        msg.data = [self.extensão_garra, self.junta_garra_direita, self.junta_garra_esquerda, self.junta_dedo_esquerdo, self.junta_dedo_direito, self.rotacao]
        self.gripper_pub.publish(msg)
        if bloqueante:
            time.sleep(4)

    def estender_garra_parcial(self): 
        msg = Float64MultiArray()
        # 30% da extensão original
        self.extensão_garra = 0.096
        msg.data = [self.extensão_garra, self.junta_garra_direita, self.junta_garra_esquerda, self.junta_dedo_esquerdo, self.junta_dedo_direito, self.rotacao]
        self.gripper_pub.publish(msg)
        time.sleep(4)

    def retrair_garra(self, bloqueante=True):
        msg = Float64MultiArray()
        self.extensão_garra = 0.02
        msg.data = [self.extensão_garra, self.junta_garra_direita, self.junta_garra_esquerda, self.junta_dedo_esquerdo, self.junta_dedo_direito, self.rotacao]
        self.gripper_pub.publish(msg)
        if bloqueante:
            time.sleep(4)

    def abrir_garra(self, bloqueante=True):
        msg = Float64MultiArray()
        self.junta_garra_direita = 0.5
        self.junta_garra_esquerda = 0.5
        self.junta_dedo_esquerdo = -0.4
        self.junta_dedo_direito = -0.4
        msg.data = [self.extensão_garra, self.junta_garra_direita, self.junta_garra_esquerda, self.junta_dedo_esquerdo, self.junta_dedo_direito, self.rotacao]
        self.gripper_pub.publish(msg)
        if bloqueante:
            time.sleep(4)

    def fechar_garra(self, bloqueante=True):
        msg = Float64MultiArray()
        self.junta_garra_direita = 0.3
        self.junta_garra_esquerda = 0.3
        self.junta_dedo_esquerdo = -0.8
        self.junta_dedo_direito = -0.8
        msg.data = [self.extensão_garra, self.junta_garra_direita, self.junta_garra_esquerda, self.junta_dedo_esquerdo, self.junta_dedo_direito, self.rotacao]
        self.gripper_pub.publish(msg)
        if bloqueante:
            time.sleep(4)

    def rotacionar_garra(self, bloqueante=True):
        msg = Float64MultiArray()
        self.rotacao = -0.15
        msg.data = [self.extensão_garra, self.junta_garra_direita, self.junta_garra_esquerda, self.junta_dedo_esquerdo, self.junta_dedo_direito, self.rotacao]
        self.gripper_pub.publish(msg)
        if bloqueante:
            time.sleep(4)

    def resetar_rotacao_garra(self):
        msg = Float64MultiArray()
        self.rotacao = 0.0
        msg.data = [self.extensão_garra, self.junta_garra_direita, self.junta_garra_esquerda, self.junta_dedo_esquerdo, self.junta_dedo_direito, self.rotacao]
        self.gripper_pub.publish(msg)
        time.sleep(4) 


    def move_robot(self):
        if self.estado_anterior != self.estado_atual:
            self.get_logger().info(f"## {self.estado_atual.name} ##")
            self.estado_anterior = self.estado_atual

        twist = Twist()
        dx = 15
        base_vel_angular = 0.3
        # Inserção da velocidade dinâmica baseada na nova variável exclusiva
        base_vel_linear = self.calcular_velocidade_dinamica()
        bandeira_centralizada = self.pos_x_bandeira_camera <= self.centro_x_camera + dx and self.pos_x_bandeira_camera >= self.centro_x_camera - dx
        self.direcao_bandeira = 1 if (self.pos_x_bandeira_camera < self.centro_x_camera - dx) else -1

        ## EXPLORANDO: o robô vai para frente
        if self.estado_atual == Estados.EXPLORANDO:
            if self.bandeira_a_frente:
                self.get_logger().info("Bandeira detectada! Iniciando navegação em direção à bandeira.")
                self.estado_atual = Estados.NAVEGANDO_PARA_BANDEIRA
            elif self.obstaculo_a_frente:
                self.estado_origem_desvio = Estados.EXPLORANDO
                self.contador_oscilacao = 0
                self.direcao_desvio_anterior = None
                self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO       
            else:
                twist.linear.x = base_vel_linear
            

        ## DESVIANDO DE OBSTACULO: o robô vira de forma severa e anda para frente limpando o caminho
        elif self.estado_atual == Estados.DESVIANDO_DE_OBSTACULO:
            if self.estado_origem_desvio == Estados.RETORNANDO_PARA_BASE:
                leituras_filtradas_esq = [r for r in self.distancias_frente_esquerda if r > 0.40]
                leituras_filtradas_dir = [r for r in self.distancias_frente_direita if r > 0.40]
                obstaculo_frente_esq = leituras_filtradas_esq and min(leituras_filtradas_esq) < self.distancia_max_obstaculo_frente
                obstaculo_frente_dir = leituras_filtradas_dir and min(leituras_filtradas_dir) < self.distancia_max_obstaculo_frente
                ha_obstaculo_frente = obstaculo_frente_esq or obstaculo_frente_dir
                multiplicador_severidade = 1.8  #key  
            else:
                obstaculo_frente_esq = self.obstaculo_a_frente_esquerda
                obstaculo_frente_dir = self.obstaculo_a_frente_direita
                ha_obstaculo_frente = self.obstaculo_a_frente
                multiplicador_severidade = 0.75

            if ha_obstaculo_frente:
                obstaculo_lados = self.obstaculo_a_direita or self.obstaculo_a_esquerda
                if obstaculo_frente_dir and not obstaculo_frente_esq and not obstaculo_lados:
                    self.direcao_desvio = Direcoes.ESQUERDA.value
                elif obstaculo_frente_esq and not obstaculo_frente_dir and not obstaculo_lados:
                    self.direcao_desvio = Direcoes.DIREITA.value
                elif not obstaculo_lados:
                    if self.bandeira_a_frente and self.estado_origem_desvio != Estados.RETORNANDO_PARA_BASE:
                        self.direcao_desvio = self.direcao_bandeira
                    else:
                        self.direcao_desvio = self.direcao_aleatoria
                elif self.obstaculo_a_direita:
                    self.direcao_desvio = Direcoes.ESQUERDA.value
                elif self.obstaculo_a_esquerda:
                    self.direcao_desvio = Direcoes.DIREITA.value

                # Detecta oscilação: o robô trocou de lado de desvio em relação ao ciclo anterior
                if self.direcao_desvio_anterior is not None and self.direcao_desvio != self.direcao_desvio_anterior:
                    self.contador_oscilacao += 1
                self.direcao_desvio_anterior = self.direcao_desvio

                # Quebra o ciclo de oscilação: dá ré com leve curva ao invés de continuar oscilando
                if self.contador_oscilacao >= 8:
                    self.get_logger().info("Oscilação no desvio de obstáculo detectada! Iniciando manobra de ré.")
                    self.contador_oscilacao = 0
                    self.direcao_desvio_anterior = None
                    self.x_ini_desvio2 = self.current_x
                    self.y_ini_desvio2 = self.current_y
                    self.yaw_ini_desvio2 = self.current_yaw
                    self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO_2
                else:
                    twist.angular.z = base_vel_angular * self.direcao_desvio * multiplicador_severidade
                self.tempo_desviando = 30 if self.estado_origem_desvio == Estados.RETORNANDO_PARA_BASE else 5 

            elif self.tempo_desviando > 0:
                twist.angular.z = base_vel_angular * self.direcao_desvio * (0.4 if self.estado_origem_desvio == Estados.RETORNANDO_PARA_BASE else 0.75)
                twist.linear.x = base_vel_linear
                self.tempo_desviando -= 1

            else:
                self.direcao_aleatoria = random.choice([Direcoes.DIREITA.value, Direcoes.ESQUERDA.value])
                self.estado_atual = self.estado_origem_desvio

        ## DESVIANDO DE OBSTACULO 2: dá ré por 1 metro com uma leve curva de 20°, para quebrar o ciclo de oscilação
        elif self.estado_atual == Estados.DESVIANDO_DE_OBSTACULO_2:
            # Verifica a todo instante se há obstáculo atrás; se houver, aborta a manobra imediatamente
            if self.obstaculo_atras:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("Obstáculo detectado atrás durante a ré! Abortando manobra e retomando exploração.")
                self.estado_atual = self.estado_origem_desvio
            else:
                distancia_percorrida = np.hypot(self.current_x - self.x_ini_desvio2, self.current_y - self.y_ini_desvio2)

                if distancia_percorrida >= 1.0:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info("Manobra de ré concluída. Retomando exploração.")
                    self.estado_atual = Estados.EXPLORANDO # Alterado para retornar a EXPLORANDO
                else:
                    twist.linear.x = -0.25
                    # Mantém a leve curva até completar ~20° de giro; depois disso segue reto de ré
                    yaw_delta = np.arctan2(np.sin(self.current_yaw - self.yaw_ini_desvio2), np.cos(self.current_yaw - self.yaw_ini_desvio2))
                    if abs(yaw_delta) < np.radians(20):
                        twist.angular.z = base_vel_angular * 0.5 * self.direcao_desvio
                    else:
                        twist.angular.z = 0.0

        ## NAVEGANDO PARA BANDEIRA: o robô anda na direção da bandeira
        elif self.estado_atual == Estados.NAVEGANDO_PARA_BANDEIRA:
            if not self.bandeira_a_frente:
                self.get_logger().info("Bandeira perdida, voltando para o estado de exploração.")
                self.estado_atual = Estados.EXPLORANDO
            elif self.porcentagem_bandeira_na_camera >= 3.0:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info("Bandeira alcançada! Iniciando alinhamento para coleta.")
                    self.estado_atual = Estados.POSICIONANDO_PARA_COLETA
                    self.contador_oscilacao_centralizacao = 0 # Reset na entrada do estado
            elif self.obstaculo_a_frente and self.porcentagem_bandeira_na_camera < 3.0:
                self.estado_origem_desvio = Estados.NAVEGANDO_PARA_BANDEIRA
                self.contador_oscilacao = 0
                self.direcao_desvio_anterior = None
                self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO
            else:
                if (self.direcao_bandeira == Direcoes.ESQUERDA.value and self.obstaculo_a_esquerda) or (self.direcao_bandeira == Direcoes.DIREITA.value and self.obstaculo_a_direita):
                    twist.angular.z = base_vel_angular * 0.50 * (self.direcao_bandeira if not bandeira_centralizada else 0)
                else:
                    twist.angular.z = base_vel_angular * (self.direcao_bandeira if not bandeira_centralizada else 0)
                twist.linear.x = base_vel_linear * (0.5 if (self.porcentagem_bandeira_na_camera >= 2.0) else 1)

        ## POSICIONANDO PARA COLETA: o robô se alinha com o mastro da bandeira
        elif self.estado_atual == Estados.POSICIONANDO_PARA_COLETA:
            dx = 1
            bandeira_centralizada = self.pos_x_bandeira_camera <= self.centro_x_camera + dx and self.pos_x_bandeira_camera >= self.centro_x_camera - dx
            self.direcao_bandeira = 1 if (self.pos_x_bandeira_camera < self.centro_x_camera - dx) else -1
            
            if not self.bandeira_a_frente:
                self.get_logger().info("Bandeira perdida, voltando para o estado de exploração.")
                self.estado_atual = Estados.EXPLORANDO
            elif not bandeira_centralizada:
                # Lógica para detectar oscilação de centralização
                direcao_atual = 1 if self.pos_x_bandeira_camera < self.centro_x_camera else -1
                if self.direcao_centralizacao_anterior is not None and direcao_atual != self.direcao_centralizacao_anterior:
                    self.contador_oscilacao_centralizacao += 1
                self.direcao_centralizacao_anterior = direcao_atual

                if self.contador_oscilacao_centralizacao >= 15:
                    self.get_logger().info("Oscilação excessiva na centralização! Dando ré de 1 metro e retomando exploração.")
                    self.contador_oscilacao_centralizacao = 0
                    self.x_ini_desvio2 = self.current_x
                    self.y_ini_desvio2 = self.current_y
                    self.yaw_ini_desvio2 = self.current_yaw
                    self.direcao_desvio = direcao_atual
                    self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO_2
                else:
                    self.get_logger().info("Centralizando garra com o mastro da bandeira...")
                    twist.angular.z = (base_vel_angular * 0.25) * self.direcao_bandeira
                    if self.obstaculo_a_esquerda or self.obstaculo_a_direita:
                        twist.linear.x = base_vel_linear * 0.5
            else:
                self.contador_oscilacao_centralizacao = 0 # Reseta se centralizado
                # Aferição precisa da distância ao mastro
                leituras_frente = [d for d in (self.distancias_frente_esquerda + self.distancias_frente_direita) if np.isfinite(d) and d > 0.05]
                dist_atual = min(leituras_frente) if leituras_frente else 0.47
                alvo = 0.67
                
                if abs(dist_atual - alvo) > 0.02:
                    self.get_logger().info(f"Ajustando distância: {dist_atual:.2f}m -> {alvo}m")
                    twist.linear.x = 0.1 if dist_atual > alvo else -0.1
                else:
                    self.get_logger().info(f"Distância correta ({dist_atual:.2f}m). Aferindo para extensão...")
                    self.distancia_extensao_garra = dist_atual - 0.23
                    self.estado_atual = Estados.CAPTURANDO_BANDEIRA

        ## CAPTURANDO BANDEIRA
        elif self.estado_atual == Estados.CAPTURANDO_BANDEIRA:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            # Solução "à prova de falhas" (intencionalmente bruta): em vez de publicar o comando
            # uma única vez e apenas contar ciclos, o MESMO comando do passo atual é reenviado a
            # CADA ciclo enquanto self.ciclos_espera_garra > 0. Isso garante que (a) uma eventual
            # publicação perdida não trave a sequência e (b) o movimento físico tenha tempo de
            # sobra para realmente terminar antes de avançar para o próximo passo. Só quando o
            # contador zera é que self.passo_captura avança — então a ordem abrir -> estender ->
            # fechar -> retrair -> rotacionar fica garantida, custe o tempo que custar.
            duracao_passo_garra = 100  # ~10s (100 ciclos de 0.1s) por passo, com reenvio contínuo
            if self.ciclos_espera_garra > 0:
                # Aguarda a garra concluir o movimento anterior sem bloquear o timer/executor
                if self.passo_captura == 0:
                    self.abrir_garra(bloqueante=False)
                elif self.passo_captura == 1:
                    self.estender_garra(bloqueante=False)
                elif self.passo_captura == 2:
                    self.fechar_garra(bloqueante=False)
                elif self.passo_captura == 3:
                    self.retrair_garra(bloqueante=False)
                elif self.passo_captura == 4:
                    self.rotacionar_garra(bloqueante=False)
                self.ciclos_espera_garra -= 1
                if self.ciclos_espera_garra == 0:
                    self.passo_captura += 1
            elif self.passo_captura == 0:
                self.get_logger().info("Abrindo garra...")
                self.abrir_garra(bloqueante=False)
                self.ciclos_espera_garra = duracao_passo_garra
            elif self.passo_captura == 1:
                self.get_logger().info("Estendendo garra...")
                self.estender_garra(bloqueante=False)
                self.bandeira_capturada = True
                self.ciclos_espera_garra = duracao_passo_garra
            elif self.passo_captura == 2:
                self.get_logger().info("Fechando garra...")
                self.fechar_garra(bloqueante=False)
                self.ciclos_espera_garra = duracao_passo_garra
            elif self.passo_captura == 3:
                self.get_logger().info("Retraindo garra...")
                self.retrair_garra(bloqueante=False)
                self.ciclos_espera_garra = duracao_passo_garra
            elif self.passo_captura == 4:
                self.get_logger().info("Rotacionando garra...")
                self.rotacionar_garra(bloqueante=False)
                self.ciclos_espera_garra = duracao_passo_garra
            elif self.passo_captura == 5:
                self.get_logger().info("Bandeira capturada com sucesso! Retornando para a base")
                self.estado_atual = Estados.RETORNANDO_PARA_BASE
                self.tempo_recuo = 90 #key
                self.tempo_recuo_base = 0 
                self.passo_captura = 0

        ## RETORNANDO PARA BASE: Controle de posição e parada exata a 0.3 metros
        elif self.estado_atual == Estados.RETORNANDO_PARA_BASE:
            # Aumenta a distância de detecção de segurança durante o retorno
            self.distancia_max_obstaculo_frente = 0.65  #key
            
            if self.tempo_recuo > 0:
                if self.obstaculo_atras:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info("Obstáculo detectado atrás! Suspendendo ré pós-captura.")
                    self.tempo_recuo = 0
                else:
                    twist.linear.x = -0.25
                    twist.angular.z = 0.0
                    self.tempo_recuo -= 1
            else:
                if self.start_x is None:
                    self.start_x = 0.0
                    self.start_y = 0.0

                dx_base = self.start_x - self.current_x
                dy_base = self.start_y - self.current_y
                distancia_base = np.hypot(dx_base, dy_base)

                # Chegada no centro (limiar de 0.2m)
                if distancia_base <= 0.2: 
                    if self.tempo_recuo_base < 80: #key
                        twist.linear.x = -0.3 
                        self.tempo_recuo_base += 1
                    else:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        # Restaura detecção padrão ao chegar
                        self.distancia_max_obstaculo_frente = 0.63
                        self.get_logger().info("Posição de depósito final alcançada. Preparando garra...")
                        self.estado_atual = Estados.RESETANDO_GARRA_BASE
                
                else:
                    obstaculo_real_frente = False
                    leituras_filtradas_esq = [r for r in self.distancias_frente_esquerda if r > 0.40]
                    leituras_filtradas_dir = [r for r in self.distancias_frente_direita if r > 0.40]
                    
                    if (leituras_filtradas_esq and min(leituras_filtradas_esq) < self.distancia_max_obstaculo_frente) or \
                       (leituras_filtradas_dir and min(leituras_filtradas_dir) < self.distancia_max_obstaculo_frente):
                        obstaculo_real_frente = True

                    if obstaculo_real_frente:
                        self.estado_origem_desvio = Estados.RETORNANDO_PARA_BASE
                        self.contador_oscilacao = 0
                        self.direcao_desvio_anterior = None
                        self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO
                    
                    else:
                        ang_objetivo = np.arctan2(dy_base, dx_base)
                        erro_ang = ang_objetivo - self.current_yaw
                        erro_ang = np.arctan2(np.sin(erro_ang), np.cos(erro_ang))

                        if abs(erro_ang) > 0.2:
                            twist.angular.z = base_vel_angular * (1.0 if erro_ang > 0 else -1.0)
                            twist.linear.x = 0.0
                        else:
                            twist.angular.z = base_vel_angular * 0.5 * erro_ang
                            # Velocidade linear proporcional ao erro angular para maior suavidade
                            twist.linear.x = (base_vel_linear * 1.4) * (1.0 - min(abs(erro_ang), 0.5))

        ## RESETANDO GARRA BASE
        elif self.estado_atual == Estados.RESETANDO_GARRA_BASE:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("[Alinhamento Base] Retornando rotação da garra para o ângulo inicial...")
            self.resetar_rotacao_garra()
            self.estado_atual = Estados.SOLTANDO_BANDEIRA
            self.passo_soltura = 0

        ## SOLTANDO BANDEIRA
        elif self.estado_atual == Estados.SOLTANDO_BANDEIRA:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            if self.passo_soltura == 0:
                self.get_logger().info("[Descarregamento] Estendendo a garra (30%) com a bandeira na base...")
                self.estender_garra_parcial() 
                self.passo_soltura = 1
            elif self.passo_soltura == 1:
                self.get_logger().info("[Descarregamento] Abrindo a garra para soltar a bandeira...")
                self.abrir_garra()
                self.passo_soltura = 2
            elif self.passo_soltura == 2:
                self.get_logger().info("[Descarregamento] Retraindo a garra completamente vazia...")
                self.retrair_garra()
                self.passo_soltura = 3
                self.get_logger().info("Desafio de Captura, Retorno Otimizado e Depósito Concluído com Sucesso!")
                self.timer.cancel()

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
