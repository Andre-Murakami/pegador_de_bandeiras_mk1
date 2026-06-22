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
    RE_PRE_DESVIO = 10
    POSICIONANDO_PARA_DEPOSITO = 11
    GIRANDO_360_RETORNO = 12
    AVANCANDO_PARA_DEPOSITO = 13
    NAVEGANDO_PARA_AREA_SOLO_AZUL = 14
    PERMANECENDO_AREA_SOLO_AZUL = 15
    GIRANDO_180_AREA_SOLO_AZUL = 16
    IDENTIFICANDO_ARENA = 17
    APROXIMANDO_PAREDE = 18
    GIRANDO_PARALELO_PAREDE = 19
    SEGUINDO_PAREDE = 20
    GIRANDO_90_NOVA_PAREDE = 21
    AVANCANDO_FIXO_NOVA_PAREDE = 22
    # Estado intermediário (Arena Paredes): avanço em linha reta de 0.8s ANTES do giro pós-parede
    AP_AVANCO_PRE_GIRO_NOVA_PAREDE = 23
    # Estado de retorno por odometria (Arena Paredes): após capturar a bandeira, navega de volta
    # ao ponto de nascimento usando as coordenadas de odometria armazenadas durante a ida
    AP_RETORNO_ODOMETRIA = 24

class Direcoes(Enum):
    ESQUERDA = 1
    DIREITA = -1


class ControleRobo(Node):

    def __init__(self):
        super().__init__('controle_robo')
        
        # Linha adicionada para indicar a versão no terminal
        self.get_logger().info("--- VERSAO 000121 ---")

        # === CRONÔMETRO DA MISSÃO ===
        self.tempo_inicio_missao = time.time()
        self.tempo_captura_bandeira = None
        self.tempo_inicio_retorno = None
        self.get_logger().info("Cronômetro da missão iniciado.")

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
        self.distancia_max_obstaculo_frente = 0.61
        self.distancia_max_obstaculo_lados = 0.27
        
        # Variável exclusiva para controle de velocidade (não afeta o desvio)
        self.distancia_limite_velocidade = 1.5

        # Distância (via LIDAR frontal) usada para o ajuste fino de distância já existente
        # dentro de POSICIONANDO_PARA_COLETA, antes de aferir a extensão da garra
        self.distancia_alvo_bandeira = 0.67  #key

        # Porcentagem da bandeira na câmera para considerar que ela foi "alcançada" e iniciar
        # a centralização da garra (saída de NAVEGANDO_PARA_BANDEIRA). AJUSTE FINO: altere o
        # valor abaixo para calibrar a distância em que a centralização da garra é iniciada.
        self.porcentagem_alvo_bandeira_coleta = 4.1  #key ANDRE

        # Range de detecção de obstáculos à frente (-30° a +30°)
        self.indices_frente_esquerda = list(range(0, 31))
        self.indices_frente_direita = list(range(329, 360))
        # Índices originais guardados para restaurar após o retorno à base
        self._indices_frente_esquerda_orig = list(self.indices_frente_esquerda)
        self._indices_frente_direita_orig = list(self.indices_frente_direita)
        # Zona cega frontal estreita usada no RETORNANDO_PARA_BASE para ignorar o mastro
        # capturado: descarta -15° a +15° (índices 0-15 e 345-360), mantendo apenas 16-24 e 335-344
        self._indices_frente_esquerda_retorno = list(range(6, 25))  # 6, 25
        self._indices_frente_direita_retorno = list(range(335, 354))  # 335, 354
        self._zona_cega_mastro_ativa = False
        # Range de detecção de obstáculos à esquerda (30° a 90°)
        self.indices_esquerda = list(range(30, 90))
        # Range de detecção de obstáculos à diretia (-30° a -90°)
        self.indices_direita = list(range(270, 330))
        # Range de detecção de obstáculo atrás (usado na manobra de ré) - 150° a 210°
        self.indices_atras = list(range(159, 202))

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

        self.estado_atual = Estados.IDENTIFICANDO_ARENA
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
        self.distancia_max_obstaculo_tras = 0.387 # key
        self.distancias_atras = []
        self.obstaculo_atras = False

        # Ré curta pré-desvio: se o obstáculo à frente estiver mais próximo que este limiar,
        # o robô recua um pouco antes de executar o desvio de obstáculo normal já implementado
        self.distancia_minima_re_pre_desvio = 0.39  #key
        self.distancia_re_pre_desvio = 0.21  #key
        self.x_ini_re_pre_desvio = 0.0
        self.y_ini_re_pre_desvio = 0.0

        # Variáveis para detecção da base e região da base (retorno)
        self.base_a_frente = False
        self.pos_x_base_camera = -1
        self.porcentagem_base_na_camera = 0.0
        self.regiao_base_a_frente = False
        self.pos_x_regiao_camera = -1
        self.dentro_da_regiao_base = False
        # Limiar de % de pixels da região na tela para considerar "dentro da região"
        self.limiar_dentro_regiao = 15.0
        self.direcao_base = -1

        # Controle de oscilação no desvio de obstáculo e manobra de ré com curva
        self.contador_oscilacao = 0
        self.direcao_desvio_anterior = None
        self.x_ini_desvio2 = 0.0
        self.y_ini_desvio2 = 0.0
        self.yaw_ini_desvio2 = 0.0

        # Redução progressiva da distância de detecção do LIDAR frontal: a cada 4 oscilações
        # (esquerda/direita/esquerda/direita...) sem ainda completar as 5 que disparam a ré,
        # a distância de detecção do obstáculo à frente é reduzida, tentando destravar o robô
        self.decremento_distancia_obstaculo_frente = 0.01  #key
        self.distancia_minima_obstaculo_frente = 0.35  #key (piso de segurança para não zerar/negativar) original 0.15

        # Redução progressiva da distância da manobra de ré disparada pela oscilação (5 vezes):
        # na 1ª vez que ocorre, a ré é a padrão (1.0m); a cada nova vez que a oscilação se repete,
        # a distância de ré diminui 0.2m, até chegar a 0 (quando o robô passa a apenas girar no
        # próprio eixo, sem recuar)
        self.distancia_re_padrao_obstaculo = 1.0  #key
        self.decremento_re_oscilacao = 0.2  #key
        self.contador_vezes_re_oscilacao = 0
        # Distância de ré efetivamente usada na manobra atual de DESVIANDO_DE_OBSTACULO_2
        # (definida no momento da entrada no estado; 1.0 = padrão da manobra vinda da centralização)
        self.distancia_re_atual = 0.3
        
        # Variáveis de oscilação na centralização
        self.contador_oscilacao_centralizacao = 0
        self.direcao_centralizacao_anterior = None

        # Variáveis de oscilação na centralização com a base (depósito), análogas às da bandeira
        self.contador_oscilacao_centralizacao_base = 0
        self.direcao_centralizacao_anterior_base = None

        # Controle de desvios de obstáculo consecutivos durante a centralização com a base (depósito):
        # se a centralização B for interrompida por desvio de obstáculo 4 vezes consecutivas, a partir
        # da 4ª a centralização é cancelada e a colocação da bandeira prossegue do jeito que está
        self.contador_desvios_centralizacao_deposito = 0
        self.centralizacao_deposito_cancelada = False
        # Indica que o avanço final e a extensão da garra no depósito devem usar os valores reduzidos
        # (0.11m de avanço e extensão completa da garra, igual à usada na captura da bandeira),
        # usados quando a centralização com a base foi cancelada por excesso de desvios de obstáculo
        self.avanco_deposito_reduzido = False

        # Controle do giro de 360° executado logo após a ré pós-captura, no RETORNANDO_PARA_BASE,
        # para varrer obstáculos ao redor do robô (frente, laterais e trás) antes de iniciar a busca
        self.giro_360_concluido = False
        self.angulo_acumulado_giro_360 = 0.0
        self.yaw_anterior_giro_360 = 0.0
        self.direcao_giro_360 = 1

        # Avanço de 0.47m seguido de giro de 149°, executados como a primeira coisa a ser feita
        # dentro de RETORNANDO_PARA_BASE, logo após a captura da bandeira.
        # TESTE DE DEBUG: nesta fase o sensor frontal fica desativado (não é verificado) e o
        # desvio de obstáculo passa a usar somente os sensores laterais, reconfigurados para
        # operar na mesma faixa angular que o sensor frontal usa. Antes de ajustar os sensores,
        # antes de iniciar o avanço e após o término do giro, o robô aguarda parado por
        # self.duracao_pausa_retorno (1.7s) em cada uma dessas etapas, para tentar isolar a causa do bug.
        # self.subfase_inicial_retorno: 0 = pausa inicial, 1 = pausa pós-ajuste de sensores,
        # 2 = avançando 0.47m, 3 = girando 149°, 4 = pausa pós-giro, concluído quando
        # fase_inicial_retorno_concluida=True
        self.fase_inicial_retorno_concluida = False
        self.subfase_inicial_retorno = 0
        self.tempo_inicio_subfase_retorno = 0.0
        self.duracao_pausa_retorno = 1.7  #key
        self.x_ini_avanco_inicial_retorno = 0.0
        self.y_ini_avanco_inicial_retorno = 0.0
        self.distancia_avanco_inicial_retorno = 0.57  #key
        self.yaw_ini_giro_inicial_retorno = 0.0
        self.direcao_giro_inicial_retorno = 1
        self._indices_esquerda_orig_retorno = []
        self._indices_direita_orig_retorno = []

        # Controle do avanço final de 0.51m em direção à base, executado após a centralização em
        # POSICIONANDO_PARA_DEPOSITO e antes de estender a garra para depositar a bandeira
        self.x_ini_avanco_deposito = 0.0
        self.y_ini_avanco_deposito = 0.0
        self.distancia_avanco_deposito = 0.51  #key

        # Avanço reduzido usado quando a centralização com a base é cancelada por 4 desvios de
        # obstáculo consecutivos: a bandeira é depositada com o robô avançando somente 0.11m
        self.distancia_avanco_deposito_reduzido = 0.11  #key

        # Controle da ré pós-captura por distância (antes era por tempo fixo): o robô recua
        # exatamente 0.76m antes de iniciar o giro de 360° e a busca pela base
        self.x_ini_re_pos_captura = 0.0
        self.y_ini_re_pos_captura = 0.0
        self.distancia_re_pos_captura = 0.76  #key

        # Variáveis para detecção da área de solo azul (zona de partida/permanência da bandeira azul)
        self.area_solo_azul_a_frente = False
        self.pos_x_area_solo_azul_camera = -1
        self.porcentagem_area_solo_azul_na_camera = 0.0
        self.dentro_area_solo_azul = False
        # Limiar de % de pixels da área de solo azul na tela para considerar "dentro da área"
        self.limiar_dentro_area_solo_azul = 15.0
        self.direcao_area_solo_azul = -1

        # Controle do giro de 180° executado quando o robô sai da área de solo azul sem ter
        # encontrado a bandeira azul, de forma a reorientá-lo de volta para a área (análogo ao
        # giro de 360° já existente em GIRANDO_360_RETORNO)
        self.angulo_acumulado_giro_180_area_azul = 0.0
        self.yaw_anterior_giro_180_area_azul = 0.0
        self.direcao_giro_180_area_azul = 1

        # Variáveis para caminhada de 2 metros antes do giro de reorientação
        self.x_ini_caminhada_reorientacao = 0.0
        self.y_ini_caminhada_reorientacao = 0.0
        self.distancia_caminhada_reorientacao = 2.0

        # === VARIÁVEIS PARA LÓGICA ALTERNADA DE RÉ NO DESVIANDO_DE_OBSTACULO_2 ===
        self.tipo_re_atual = 1  # 1 = giro puro no próprio eixo / 2 = micro deslocamentos
        self.contador_micro_desloc = 0
        self.micro_fase = 0   # alterna entre frente e trás nos micro movimentos
        # Ré de 0.33m executada ANTES do giro de 100° (tipo 1) em DESVIANDO_DE_OBSTACULO_2
        self.re_pre_giro_iniciada = False
        self.re_pre_giro_concluida = False
        self.x_ini_re_pre_giro = 0.0
        self.y_ini_re_pre_giro = 0.0
        self.distancia_re_pre_giro = 0.07  #key // mudei de 0.33 para 0.07

        # === IDENTIFICAÇÃO DE ARENA (CILINDROS vs PAREDES) ===
        # Tempo de espera (em segundos), logo após o robô nascer no cenário, antes de verificar
        # em qual arena ele está, para evitar qualquer bug de leitura de câmera ainda não estabilizada
        self.tempo_inicio_identificacao_arena = time.time()
        self.tempo_espera_identificacao_arena = 3.0  #key # parametro: tempo_espera_identificacao_arena
        # None enquanto não identificada; depois passa a ser 'CILINDROS' ou 'PAREDES'
        self.arena_identificada = None

        # Supressão da detecção de parede durante a navegação rumo à área de solo azul:
        # após self.tempo_suprimir_parede_rumo_solo_azul segundos de caminhada constante em
        # direção à área de solo azul (dentro de NAVEGANDO_PARA_AREA_SOLO_AZUL), a variável
        # self.parede_a_frente deixa de ser usada para decidir ir para APROXIMANDO_PAREDE,
        # evitando que o robô desvie da rota para a área ao ver uma parede lateral ou oblíqua.
        # A detecção de parede é reabilitada após self.tempo_reabilitar_parede_dentro_solo_azul
        # segundos depois de o robô confirmar que adentrou a área de solo azul.
        self.tempo_suprimir_parede_rumo_solo_azul = 6.1     #key # parametro: tempo_suprimir_parede_rumo_solo_azul
        self.tempo_reabilitar_parede_dentro_solo_azul = 3.1 #key # parametro: tempo_reabilitar_parede_dentro_solo_azul
        self.tempo_inicio_navegando_para_solo_azul = 0.0    # cronômetro armado ao entrar em NAVEGANDO_PARA_AREA_SOLO_AZUL
        self.tempo_inicio_dentro_solo_azul = 0.0            # cronômetro armado ao entrar em PERMANECENDO_AREA_SOLO_AZUL
        # Contador de ciclos consecutivos em que area_solo_azul_a_frente foi False durante a
        # navegação para a área. Só desiste e volta a EXPLORANDO quando esse contador atingir
        # ap_frames_perda_solo_azul ciclos seguidos sem ver a área — evita que um único frame
        # ruim (race condition camera_callback vs move_robot) derrube a navegação.
        self.ap_contador_perda_solo_azul = 0
        self.ap_frames_perda_solo_azul = 10  #key # parametro: ap_frames_perda_solo_azul (ciclos de 0.1s = 1s de tolerância)
        self.parede_suprimida_rumo_solo_azul = False        # True enquanto parede estiver suprimida

        # === VARIÁVEIS DE DETECÇÃO DA PAREDE (ARENA PAREDES), análogas às já existentes
        # para bandeira/base/área de solo azul ===
        self.parede_a_frente = False
        self.pos_x_parede_camera = -1
        self.porcentagem_parede_na_camera = 0.0
        # Percentual mínimo de pixels da cor da parede ocupando o "volume" total da câmera
        # para considerar que o robô nasceu na Arena Paredes
        self.limiar_identificacao_arena_paredes = 30.0  #key # parametro: limiar_identificacao_arena_paredes

        # Distância (via LiDAR frontal), em metros, na qual o robô deve parar de se aproximar
        # da parede e iniciar o giro de 90° para passar a segui-la tangencialmente
        self.distancia_aproximacao_parede = 0.67  #key # parametro: distancia_aproximacao_parede

        # Faixa de correção fina de distância (via LiDAR lateral) mantida durante o seguimento
        # tangencial à parede: dentro desta faixa o robô segue em frente sem corrigir o ângulo
        self.distancia_minima_seguindo_parede = 0.60  #key # parametro: distancia_minima_seguindo_parede
        self.distancia_maxima_seguindo_parede = 0.67  #key # parametro: distancia_maxima_seguindo_parede
        # Faixa de histerese (via LiDAR lateral), mais larga que a faixa de correção acima, usada
        # para considerar que o robô AINDA está tangenciando a MESMA parede mesmo que tenha se
        # afastado/aproximado um pouco mais. Enquanto a leitura permanecer dentro desta faixa, o
        # robô não procura/migra para outra parede. Assim que a leitura sair desta faixa (abaixo
        # do mínimo ou acima do máximo), considera-se que o contato com a parede foi perdido.
        self.distancia_minima_contato_parede = 0.4  #key # parametro: distancia_minima_contato_parede
        self.distancia_maxima_contato_parede = 1.1  #key # parametro: distancia_maxima_contato_parede

        # Fator de redução da velocidade linear aplicado durante o seguimento tangencial à
        # parede e durante o avanço fixo em busca de nova parede (fase mais delicada da
        # navegação, exige velocidade reduzida para corrigir melhor a distância)
        self.fator_velocidade_arena_paredes = 0.5  #key # parametro: fator_velocidade_arena_paredes

        # Lado (LiDAR lateral) escolhido para seguir a parede. É redefinido (de forma FIXA para
        # cada encontro com uma parede) no momento em que o robô para diante dela, com base em
        # qual lado do sensor frontal (esquerda/direita) está mais próximo dela; o valor abaixo
        # é apenas o valor inicial antes da primeira detecção.
        self.lado_seguindo_parede = self.direcao_aleatoria

        # Controle do giro de 90° executado antes de iniciar o seguimento tangencial à parede
        self.yaw_ini_giro_paralelo_parede = 0.0

        # Controle do giro de 90° executado ao perder contato lateral com a parede, antes de
        # seguir em frente à procura de uma nova parede
        self.yaw_ini_giro_nova_parede = 0.0

        # Avanço fixo (em linha reta) executado logo após o giro de 90° em busca de nova
        # parede, antes de retomar a procura "oficial" (reaproveitando o script de aproximação)
        self.distancia_avanco_fixo_nova_parede = 0.77  #key # parametro: distancia_avanco_fixo_nova_parede
        self.x_ini_avanco_fixo_nova_parede = 0.0
        self.y_ini_avanco_fixo_nova_parede = 0.0
        # Controla o ciclo do protocolo Giro+Avanço: 0 = 1º ciclo (55° + 0.5m),
        # 1 = 2º ciclo (27.5° + 0.25m, metade das métricas do 1º). Zera ao encontrar nova parede.
        self.contador_ciclos_nova_parede = 0

        # Avanço em linha reta de 0.8s ANTES do giro pós-parede (AP_AVANCO_PRE_GIRO_NOVA_PAREDE).
        # Inserido entre o momento em que o sensor lateral perde a parede e o início do giro,
        # para que o robô avance um pouco além da quina antes de virar.
        self.ap_duracao_avanco_pre_giro = 1.8   #key # parametro: ap_duracao_avanco_pre_giro (segundos)
        self.ap_tempo_ini_avanco_pre_giro = 0.0

        # Retorno por odometria (Arena Paredes): coordenadas do ponto de nascimento (start_x/start_y
        # já gravadas em odom_callback) usadas como alvo de retorno após a captura da bandeira.
        # O robô navega de volta até atingir ap_raio_chegada_retorno metros do ponto de nascimento
        # e, a partir daí, passa o controle para RETORNANDO_PARA_BASE (que já tem toda a lógica
        # de centralização e depósito da bandeira na base, igual à Arena Cilindros).
        self.ap_raio_chegada_retorno = 1.5   #key # parametro: ap_raio_chegada_retorno (metros — distância do start para considerar "chegou")
        self.ap_retorno_odometria_ativo = False  # ativado ao entrar em AP_RETORNO_ODOMETRIA

    def obstaculo_frente_oposto_parede(self):
        """Durante o seguimento tangencial da parede (Arena Paredes), o sensor frontal do MESMO
        lado escolhido para tangenciar a parede (self.lado_seguindo_parede) não deve ser
        considerado como detector de obstáculo: a própria correção de trajetória para manter o
        robô tangenciado pode aproximar momentaneamente esse sensor da parede sendo seguida,
        gerando falsos positivos. Apenas o sensor frontal do lado OPOSTO é considerado real."""
        if self.lado_seguindo_parede == Direcoes.ESQUERDA.value:
            return self.obstaculo_a_frente_direita
        else:
            return self.obstaculo_a_frente_esquerda

    def obstaculo_lateral_oposto_parede(self):
        """Análogo ao método acima, mas para o sensor LATERAL: o sensor lateral do lado que está
        sendo usado para medir a distância até a própria parede (self.lado_seguindo_parede) não
        deve ser considerado um detector de obstáculo. Apenas o lado OPOSTO é considerado real."""
        if self.lado_seguindo_parede == Direcoes.ESQUERDA.value:
            return self.obstaculo_a_direita
        else:
            return self.obstaculo_a_esquerda

    def calcular_velocidade_dinamica(self):
        """Calcula velocidade baseada na variável exclusiva self.distancia_limite_velocidade."""
        caminho_livre = True
        for dist in self.distancias_frente_esquerda + self.distancias_frente_direita:
            if 0.1 < dist < self.distancia_limite_velocidade:
                caminho_livre = False
                break
        
        return 1.4 if caminho_livre else 0.5

    def scan_callback(self, msg: LaserScan):
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        self.distancias_frente_esquerda = [msg.ranges[i] for i in self.indices_frente_esquerda]
        self.obstaculo_a_frente_esquerda = self.distancias_frente_esquerda and min(self.distancias_frente_esquerda) < self.distancia_max_obstaculo_frente

        self.distancias_frente_direita = [msg.ranges[i] for i in self.indices_frente_direita]
        self.obstaculo_a_frente_direita = self.distancias_frente_direita and min(self.distancias_frente_direita) < self.distancia_max_obstaculo_frente

        self.obstaculo_a_frente = self.obstaculo_a_frente_direita or self.obstaculo_a_frente_esquerda

        self.distancias_esquerda = [msg.ranges[i] for i in self.indices_esquerda]
        self.obstaculo_a_esquerda = self.distancias_esquerda and min(self.distancias_esquerda) < self.distancia_max_obstaculo_lados

        self.distancias_direita = [msg.ranges[i] for i in self.indices_direita]
        self.obstaculo_a_direita = self.distancias_direita and min(self.distancias_direita) < self.distancia_max_obstaculo_lados

        self.distancias_atras = [msg.ranges[i] for i in self.indices_atras]
        self.obstaculo_atras = self.distancias_atras and min(self.distancias_atras) < self.distancia_max_obstaculo_tras


    def imu_callback(self, msg: Imu):
        return


    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        rot = R.from_quat([q.x, q.y, q.z, q.w])
        self.current_yaw = rot.as_euler('xyz')[2]
        
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y


    def camera_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        h, w = frame.shape[0], frame.shape[1]
        self.centro_x_camera = w//2

        target_color = np.array([227, 73, 0])
        mask_bandeira = cv2.inRange(frame, target_color, target_color)
        
        contours_bandeira, _ = cv2.findContours(mask_bandeira, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        mask_mastro = mask_bandeira.copy()
        limite_corte = int(h * 0.5)
        mask_mastro[0:limite_corte, :] = 0
        contours_mastro, _ = cv2.findContours(mask_mastro, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.bandeira_a_frente = len(contours_bandeira) > 0
        if self.bandeira_a_frente:
            cnt = max(contours_bandeira, key=cv2.contourArea)
            M_bandeira = cv2.moments(cnt)
            if M_bandeira['m00'] != 0:
                area_total = w*h
                area_bandeira = cv2.contourArea(cnt)
                self.porcentagem_bandeira_na_camera = (area_bandeira / area_total) * 100.0
            if len(contours_mastro) > 0:
                cnt_mastro = max(contours_mastro, key=cv2.contourArea)
                M_mastro = cv2.moments(cnt_mastro)
                if M_mastro['m00'] != 0:
                    self.pos_x_bandeira_camera = int(M_mastro['m10'] / M_mastro['m00'])
            else:
                if M_bandeira['m00'] != 0:
                    self.pos_x_bandeira_camera = int(M_bandeira['m10'] / M_bandeira['m00'])
        else:
            self.bandeira_a_frente = False
            self.porcentagem_bandeira_na_camera = 0.0

        cor_base = np.array([40, 174, 0])
        mask_base = cv2.inRange(frame, cor_base, cor_base)
        contours_base, _ = cv2.findContours(mask_base, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.base_a_frente = len(contours_base) > 0
        if self.base_a_frente:
            cnt_b = max(contours_base, key=cv2.contourArea)
            M_base = cv2.moments(cnt_b)
            area_total = w * h
            self.porcentagem_base_na_camera = (cv2.contourArea(cnt_b) / area_total) * 100.0
            if M_base['m00'] != 0:
                self.pos_x_base_camera = int(M_base['m10'] / M_base['m00'])
        else:
            self.porcentagem_base_na_camera = 0.0

        cor_regiao = np.array([142, 80, 0])
        mask_regiao = cv2.inRange(frame, cor_regiao, cor_regiao)
        contours_regiao, _ = cv2.findContours(mask_regiao, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.regiao_base_a_frente = len(contours_regiao) > 0
        area_total = w * h
        if self.regiao_base_a_frente:
            cnt_r = max(contours_regiao, key=cv2.contourArea)
            M_regiao = cv2.moments(cnt_r)
            area_regiao = cv2.contourArea(cnt_r)
            porcentagem_regiao = (area_regiao / area_total) * 100.0
            self.dentro_da_regiao_base = porcentagem_regiao >= self.limiar_dentro_regiao
            if M_regiao['m00'] != 0:
                self.pos_x_regiao_camera = int(M_regiao['m10'] / M_regiao['m00'])
        else:
            self.dentro_da_regiao_base = False

        cor_area_solo_azul = np.array([85, 249, 0])
        mask_area_solo_azul = cv2.inRange(frame, cor_area_solo_azul, cor_area_solo_azul)
        contours_area_solo_azul, _ = cv2.findContours(mask_area_solo_azul, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.area_solo_azul_a_frente = len(contours_area_solo_azul) > 0
        if self.area_solo_azul_a_frente:
            cnt_asa = max(contours_area_solo_azul, key=cv2.contourArea)
            M_asa = cv2.moments(cnt_asa)
            area_asa = cv2.contourArea(cnt_asa)
            self.porcentagem_area_solo_azul_na_camera = (area_asa / area_total) * 100.0
            self.dentro_area_solo_azul = self.porcentagem_area_solo_azul_na_camera >= self.limiar_dentro_area_solo_azul
            if M_asa['m00'] != 0:
                self.pos_x_area_solo_azul_camera = int(M_asa['m10'] / M_asa['m00'])
        else:
            self.porcentagem_area_solo_azul_na_camera = 0.0
            self.dentro_area_solo_azul = False

        # Detecção da parede (Arena Paredes) - BGR=(171, 242, 0), informado via debug da câmera
        # (equivalente a RGB=(0, 242, 171))
        cor_parede = np.array([171, 242, 0])
        mask_parede = cv2.inRange(frame, cor_parede, cor_parede)
        contours_parede, _ = cv2.findContours(mask_parede, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.parede_a_frente = len(contours_parede) > 0
        # Percentual calculado sobre o TOTAL de pixels da cor na imagem (e não apenas o maior
        # contorno), pois é esse o critério usado para identificar a Arena Paredes (a cor deve
        # ocupar 30% do "volume" da câmera)
        pixels_parede = cv2.countNonZero(mask_parede)
        self.porcentagem_parede_na_camera = (pixels_parede / area_total) * 100.0
        if self.parede_a_frente:
            cnt_p = max(contours_parede, key=cv2.contourArea)
            M_p = cv2.moments(cnt_p)
            if M_p['m00'] != 0:
                self.pos_x_parede_camera = int(M_p['m10'] / M_p['m00'])


    def estender_garra(self, bloqueante=True):
        msg = Float64MultiArray()
        self.extensão_garra = self.distancia_extensao_garra
        msg.data = [self.extensão_garra, self.junta_garra_direita, self.junta_garra_esquerda, self.junta_dedo_esquerdo, self.junta_dedo_direito, self.rotacao]
        self.gripper_pub.publish(msg)
        if bloqueante:
            time.sleep(4)

    def estender_garra_parcial(self): 
        msg = Float64MultiArray()
        self.extensão_garra = 0.066
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
        self.rotacao = -0.19
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
        base_vel_linear = self.calcular_velocidade_dinamica()
        bandeira_centralizada = self.pos_x_bandeira_camera <= self.centro_x_camera + dx and self.pos_x_bandeira_camera >= self.centro_x_camera - dx
        self.direcao_bandeira = 1 if (self.pos_x_bandeira_camera < self.centro_x_camera - dx) else -1

        dx_parede = 15
        parede_centralizada = self.pos_x_parede_camera <= self.centro_x_camera + dx_parede and self.pos_x_parede_camera >= self.centro_x_camera - dx_parede
        direcao_parede = 1 if (self.pos_x_parede_camera < self.centro_x_camera - dx_parede) else -1

        if self.estado_atual == Estados.IDENTIFICANDO_ARENA:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            tempo_decorrido_identificacao_arena = time.time() - self.tempo_inicio_identificacao_arena
            if tempo_decorrido_identificacao_arena < self.tempo_espera_identificacao_arena:
                pass
            elif self.porcentagem_parede_na_camera >= self.limiar_identificacao_arena_paredes:
                self.arena_identificada = 'PAREDES'
                self.get_logger().info(f"=== ARENA IDENTIFICADA: PAREDES (parede ocupando {self.porcentagem_parede_na_camera:.1f}% da câmera) ===")
                self.estado_atual = Estados.APROXIMANDO_PAREDE
            else:
                self.arena_identificada = 'CILINDROS'
                self.get_logger().info(f"=== ARENA IDENTIFICADA: CILINDROS (parede ocupando apenas {self.porcentagem_parede_na_camera:.1f}% da câmera) ===")
                self.estado_atual = Estados.EXPLORANDO

        elif self.estado_atual == Estados.EXPLORANDO:
            if self.bandeira_a_frente:
                self.get_logger().info("Bandeira detectada! Iniciando navegação em direção à bandeira.")
                self.estado_atual = Estados.NAVEGANDO_PARA_BANDEIRA
            elif self.area_solo_azul_a_frente:
                self.get_logger().info(">>> ÁREA DE SOLO AZUL DETECTADA durante exploração! Navegando imediatamente para a área de solo azul. <<<")
                self.tempo_inicio_navegando_para_solo_azul = time.time()
                self.parede_suprimida_rumo_solo_azul = False
                self.estado_atual = Estados.NAVEGANDO_PARA_AREA_SOLO_AZUL
            elif self.obstaculo_a_frente:
                self.estado_origem_desvio = Estados.EXPLORANDO
                self.contador_oscilacao = 0
                self.direcao_desvio_anterior = None
                self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO       
            else:
                twist.linear.x = base_vel_linear

        elif self.estado_atual == Estados.DESVIANDO_DE_OBSTACULO:
            if self.estado_origem_desvio == Estados.RETORNANDO_PARA_BASE:
                leituras_filtradas_esq = [r for r in self.distancias_frente_esquerda if r > 0.40]
                leituras_filtradas_dir = [r for r in self.distancias_frente_direita if r > 0.40]
                obstaculo_frente_esq = leituras_filtradas_esq and min(leituras_filtradas_esq) < 0.9
                obstaculo_frente_dir = leituras_filtradas_dir and min(leituras_filtradas_dir) < 0.9
                ha_obstaculo_frente = obstaculo_frente_esq or obstaculo_frente_dir
                multiplicador_severidade = 1.8
            else:
                obstaculo_frente_esq = self.obstaculo_a_frente_esquerda
                obstaculo_frente_dir = self.obstaculo_a_frente_direita
                ha_obstaculo_frente = self.obstaculo_a_frente
                multiplicador_severidade = 0.75

            if ha_obstaculo_frente:
                dist_esq = min(self.distancias_frente_esquerda) if self.distancias_frente_esquerda else 10.0
                dist_dir = min(self.distancias_frente_direita) if self.distancias_frente_direita else 10.0

                if dist_esq < dist_dir and self.obstaculo_a_frente_esquerda:
                    self.direcao_desvio = Direcoes.DIREITA.value
                elif dist_dir < dist_esq and self.obstaculo_a_frente_direita:
                    self.direcao_desvio = Direcoes.ESQUERDA.value
                else:
                    obstaculo_lados = self.obstaculo_a_direita or self.obstaculo_a_esquerda
                    if self.obstaculo_a_frente_direita and not self.obstaculo_a_frente_esquerda and not obstaculo_lados:
                        self.direcao_desvio = Direcoes.ESQUERDA.value
                    elif self.obstaculo_a_frente_esquerda and not self.obstaculo_a_frente_direita and not obstaculo_lados:
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

                if self.direcao_desvio_anterior is not None and self.direcao_desvio != self.direcao_desvio_anterior:
                    self.contador_oscilacao += 1
                self.direcao_desvio_anterior = self.direcao_desvio

                if self.contador_oscilacao == 4:
                    self.distancia_max_obstaculo_frente = max(
                        self.distancia_max_obstaculo_frente - self.decremento_distancia_obstaculo_frente,
                        self.distancia_minima_obstaculo_frente
                    )
                    self.get_logger().info(f"4 oscilações detectadas! Reduzindo distância de detecção frontal do LIDAR para {self.distancia_max_obstaculo_frente:.3f}m.")

                if self.contador_oscilacao >= 5:
                    self.get_logger().info("Oscilação no desvio de obstáculo detectada! Iniciando manobra de ré.")
                    self.contador_oscilacao = 0
                    self.direcao_desvio_anterior = None
                    self.x_ini_desvio2 = self.current_x
                    self.y_ini_desvio2 = self.current_y
                    self.yaw_ini_desvio2 = self.current_yaw
                    self.distancia_re_atual = max(
                        self.distancia_re_padrao_obstaculo - (self.contador_vezes_re_oscilacao * self.decremento_re_oscilacao),
                        0.0
                    )
                    self.get_logger().info(f"Distância da manobra de ré definida para {self.distancia_re_atual:.2f}m (ocorrência nº {self.contador_vezes_re_oscilacao + 1}).")
                    self.contador_vezes_re_oscilacao += 1
                    self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO_2
                else:
                    twist.angular.z = base_vel_angular * self.direcao_desvio * multiplicador_severidade
                self.tempo_desviando = 6 if self.estado_origem_desvio == Estados.RETORNANDO_PARA_BASE else 6.5    # key define quanto tempo o robo desvio do obstaculo

            elif self.tempo_desviando > 0:
                twist.angular.z = base_vel_angular * self.direcao_desvio * (0.4 if self.estado_origem_desvio == Estados.RETORNANDO_PARA_BASE else 0.75)
                twist.linear.x = base_vel_linear
                self.tempo_desviando -= 1

            else:
                self.direcao_aleatoria = random.choice([Direcoes.DIREITA.value, Direcoes.ESQUERDA.value])
                self.estado_atual = self.estado_origem_desvio

        elif self.estado_atual == Estados.DESVIANDO_DE_OBSTACULO_2:
            # Nova lógica alternada de ré conforme solicitado pelo usuário
            if self.obstaculo_atras or self.obstaculo_a_esquerda or self.obstaculo_a_direita:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("Obstáculo detectado (traseiro ou lateral) durante a ré! Suspendendo manobra e retornando ao estado anterior.")
                self.estado_atual = self.estado_origem_desvio
                self.contador_micro_desloc = 0
                self.re_pre_giro_iniciada = False
                self.re_pre_giro_concluida = False
            else:
                if self.tipo_re_atual == 1:
                    # Tipo 1: Ré de 0.33m seguida de giro puro no próprio eixo (100 graus no sentido oposto)
                    if not self.re_pre_giro_concluida:
                        if not self.re_pre_giro_iniciada:
                            self.x_ini_re_pre_giro = self.current_x
                            self.y_ini_re_pre_giro = self.current_y
                            self.re_pre_giro_iniciada = True
                            self.get_logger().info("Iniciando ré de 0.33m antes do giro...")
                        distancia_re_pre_giro_percorrida = np.hypot(self.current_x - self.x_ini_re_pre_giro, self.current_y - self.y_ini_re_pre_giro)
                        if distancia_re_pre_giro_percorrida < self.distancia_re_pre_giro:
                            twist.linear.x = -0.25
                            twist.angular.z = 0.0
                        else:
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                            self.get_logger().info("Ré de 0.33m concluída. Iniciando giro de 100°...")
                            self.re_pre_giro_concluida = True
                    else:
                        yaw_delta = np.arctan2(np.sin(self.current_yaw - self.yaw_ini_desvio2), np.cos(self.current_yaw - self.yaw_ini_desvio2))
                        if abs(yaw_delta) < np.radians(100):
                            twist.linear.x = 0.0
                            twist.angular.z = base_vel_angular * 1.6 * (-self.direcao_desvio)  # sentido oposto
                        else:
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                            self.get_logger().info("Giro no próprio eixo de 100° concluído.")
                            self.estado_atual = self.estado_origem_desvio
                            self.tipo_re_atual = 2  # alterna para micro deslocamentos
                            self.re_pre_giro_iniciada = False
                            self.re_pre_giro_concluida = False

                else:
                    # Tipo 2: Micro deslocamentos (10 ciclos)
                    if self.contador_micro_desloc >= 10:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.get_logger().info("Sequência de 10 micro deslocamentos concluída. Retomando estado anterior.")
                        self.estado_atual = self.estado_origem_desvio
                        self.contador_micro_desloc = 0
                        self.tipo_re_atual = 1  # alterna de volta
                    else:
                        self.contador_micro_desloc += 1
                        if self.micro_fase == 0:  # frente + curva para o lado oposto
                            twist.linear.x = 0.1
                            twist.angular.z = base_vel_angular * 0.8 * (-self.direcao_desvio)
                            self.micro_fase = 1
                        else:  # trás + curva para o lado do obstáculo
                            twist.linear.x = -0.1
                            twist.angular.z = base_vel_angular * 0.8 * self.direcao_desvio
                            self.micro_fase = 0

        elif self.estado_atual == Estados.RE_PRE_DESVIO:
            distancia_percorrida_re_pre_desvio = np.hypot(self.current_x - self.x_ini_re_pre_desvio, self.current_y - self.y_ini_re_pre_desvio)

            if distancia_percorrida_re_pre_desvio >= self.distancia_re_pre_desvio:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("Ré curta pré-desvio concluída. Iniciando desvio de obstáculo.")
                self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO
            else:
                twist.linear.x = -0.25
                twist.angular.z = 0.0

        elif self.estado_atual == Estados.NAVEGANDO_PARA_BANDEIRA:
            if not self.bandeira_a_frente:
                self.get_logger().info("Bandeira perdida, voltando para o estado de exploração.")
                self.estado_atual = Estados.EXPLORANDO
            elif self.porcentagem_bandeira_na_camera >= self.porcentagem_alvo_bandeira_coleta:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("Bandeira alcançada! Iniciando alinhamento para coleta.")
                self.estado_atual = Estados.POSICIONANDO_PARA_COLETA
                self.contador_oscilacao_centralizacao = 0
            elif self.obstaculo_a_frente and self.porcentagem_bandeira_na_camera < self.porcentagem_alvo_bandeira_coleta:
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

        elif self.estado_atual == Estados.POSICIONANDO_PARA_COLETA:
            dx = 1
            bandeira_centralizada = self.pos_x_bandeira_camera <= self.centro_x_camera + dx and self.pos_x_bandeira_camera >= self.centro_x_camera - dx
            self.direcao_bandeira = 1 if (self.pos_x_bandeira_camera < self.centro_x_camera - dx) else -1
            
            if not self.bandeira_a_frente:
                self.get_logger().info("Bandeira perdida, voltando para o estado de exploração.")
                self.estado_atual = Estados.EXPLORANDO
            elif self.obstaculo_a_frente:
                self.get_logger().info("Obstáculo detectado durante centralização para captura! Cancelando e desviando.")
                self.estado_origem_desvio = Estados.POSICIONANDO_PARA_COLETA
                self.contador_oscilacao = 0
                self.direcao_desvio_anterior = None
                self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO
            elif not bandeira_centralizada:
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
                    self.distancia_re_atual = 1.0
                    self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO_2
                else:
                    self.get_logger().info("Centralizando garra com o mastro da bandeira...")
                    twist.angular.z = (base_vel_angular * 0.25) * self.direcao_bandeira
                    if self.obstaculo_a_esquerda or self.obstaculo_a_direita:
                        twist.linear.x = base_vel_linear * 0.5
            else:
                self.contador_oscilacao_centralizacao = 0
                leituras_frente = [d for d in (self.distancias_frente_esquerda + self.distancias_frente_direita) if np.isfinite(d) and d > 0.05]
                dist_atual = min(leituras_frente) if leituras_frente else 0.47
                alvo = self.distancia_alvo_bandeira
                
                if abs(dist_atual - alvo) > 0.02:
                    self.get_logger().info(f"Ajustando distância: {dist_atual:.2f}m -> {alvo}m")
                    twist.linear.x = 0.1 if dist_atual > alvo else -0.1
                else:
                    self.get_logger().info(f"Distância correta ({dist_atual:.2f}m). Aferindo para extensão...")
                    self.distancia_extensao_garra = dist_atual - 0.23
                    self.estado_atual = Estados.CAPTURANDO_BANDEIRA

        elif self.estado_atual == Estados.CAPTURANDO_BANDEIRA:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            duracao_passo_garra = 35
            if self.ciclos_espera_garra > 0:
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
                self.tempo_captura_bandeira = time.time()
                self.ciclos_espera_garra = duracao_passo_garra
            elif self.passo_captura == 2:
                self.get_logger().info("Fechar garra...")
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
                if self.arena_identificada == 'PAREDES':
                    # Arena Paredes: retorna ao ponto de nascimento usando odometria, depois
                    # entrega o controle a RETORNANDO_PARA_BASE para centralização e depósito.
                    self.get_logger().info(f"[AP] Retorno por odometria ativado. Alvo: start=({self.start_x:.2f}, {self.start_y:.2f}). Raio de chegada: {self.ap_raio_chegada_retorno}m.")
                    self.ap_retorno_odometria_ativo = True
                    self.estado_atual = Estados.AP_RETORNO_ODOMETRIA
                else:
                    self.estado_atual = Estados.RETORNANDO_PARA_BASE
                self.passo_captura = 0
                self.indices_frente_esquerda = self._indices_frente_esquerda_retorno
                self.indices_frente_direita = self._indices_frente_direita_retorno
                self._zona_cega_mastro_ativa = True
                self.get_logger().info("Zona cega frontal ativada: ignorando mastro capturado no LiDAR.")
                self.fase_inicial_retorno_concluida = False
                self.subfase_inicial_retorno = 0
                self.tempo_inicio_subfase_retorno = time.time()

        elif self.estado_atual == Estados.NAVEGANDO_PARA_AREA_SOLO_AZUL:
            # Arma o cronômetro na primeira vez que entra neste estado
            if self.tempo_inicio_navegando_para_solo_azul == 0.0:
                self.tempo_inicio_navegando_para_solo_azul = time.time()
                self.parede_suprimida_rumo_solo_azul = False
                self.ap_contador_perda_solo_azul = 0
                self.get_logger().info("Iniciando navegação para a área de solo azul. Cronômetro de supressão de parede armado.")

            # Após tempo_suprimir_parede_rumo_solo_azul segundos de caminhada, suprime a
            # detecção de parede para não desviar da rota rumo ao solo azul
            tempo_navegando_solo_azul = time.time() - self.tempo_inicio_navegando_para_solo_azul
            if not self.parede_suprimida_rumo_solo_azul and tempo_navegando_solo_azul >= self.tempo_suprimir_parede_rumo_solo_azul:  # parametro: tempo_suprimir_parede_rumo_solo_azul
                self.parede_suprimida_rumo_solo_azul = True
                self.get_logger().info(f"Parede suprimida para tangenciamento ({self.tempo_suprimir_parede_rumo_solo_azul:.1f}s de rota ao solo azul decorridos).")

            dx_asa = 15
            area_solo_azul_centralizada = (
                self.pos_x_area_solo_azul_camera <= self.centro_x_camera + dx_asa and
                self.pos_x_area_solo_azul_camera >= self.centro_x_camera - dx_asa
            )
            self.direcao_area_solo_azul = 1 if (self.pos_x_area_solo_azul_camera < self.centro_x_camera - dx_asa) else -1

            if self.bandeira_a_frente:
                self.get_logger().info("Bandeira detectada durante navegação para a área de solo azul! Priorizando a bandeira.")
                self.tempo_inicio_navegando_para_solo_azul = 0.0
                self.parede_suprimida_rumo_solo_azul = False
                self.ap_contador_perda_solo_azul = 0
                self.estado_atual = Estados.NAVEGANDO_PARA_BANDEIRA
            elif not self.area_solo_azul_a_frente:
                # Debounce: só desiste após ap_frames_perda_solo_azul ciclos consecutivos sem ver
                # a área. Evita que um único frame ruim (race condition câmera/timer) derrube a
                # navegação. Enquanto aguarda, o robô continua avançando na última direção conhecida.
                self.ap_contador_perda_solo_azul += 1
                if self.ap_contador_perda_solo_azul >= self.ap_frames_perda_solo_azul:  # parametro: ap_frames_perda_solo_azul
                    self.get_logger().info(f"Área de solo azul perdida por {self.ap_frames_perda_solo_azul} ciclos consecutivos. Voltando para exploração.")
                    self.tempo_inicio_navegando_para_solo_azul = 0.0
                    self.parede_suprimida_rumo_solo_azul = False
                    self.ap_contador_perda_solo_azul = 0
                    self.estado_atual = Estados.EXPLORANDO
                else:
                    # Mantém o movimento na última direção conhecida enquanto aguarda confirmação
                    if (self.direcao_area_solo_azul == Direcoes.ESQUERDA.value and self.obstaculo_a_esquerda) or (self.direcao_area_solo_azul == Direcoes.DIREITA.value and self.obstaculo_a_direita):
                        twist.angular.z = base_vel_angular * 0.50 * (self.direcao_area_solo_azul if not area_solo_azul_centralizada else 0)
                    else:
                        twist.angular.z = base_vel_angular * (self.direcao_area_solo_azul if not area_solo_azul_centralizada else 0)
                    twist.linear.x = base_vel_linear
            elif self.dentro_area_solo_azul:
                self.ap_contador_perda_solo_azul = 0
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info(">>> ADENTROU A ÁREA DE SOLO AZUL! Permanecendo na área até localizar a bandeira azul. <<<")
                self.tempo_inicio_navegando_para_solo_azul = 0.0
                self.parede_suprimida_rumo_solo_azul = False
                self.tempo_inicio_dentro_solo_azul = time.time()
                self.estado_atual = Estados.PERMANECENDO_AREA_SOLO_AZUL
            elif self.obstaculo_a_frente:
                self.ap_contador_perda_solo_azul = 0
                self.estado_origem_desvio = Estados.NAVEGANDO_PARA_AREA_SOLO_AZUL
                self.contador_oscilacao = 0
                self.direcao_desvio_anterior = None
                self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO
            else:
                self.ap_contador_perda_solo_azul = 0
                if (self.direcao_area_solo_azul == Direcoes.ESQUERDA.value and self.obstaculo_a_esquerda) or (self.direcao_area_solo_azul == Direcoes.DIREITA.value and self.obstaculo_a_direita):
                    twist.angular.z = base_vel_angular * 0.50 * (self.direcao_area_solo_azul if not area_solo_azul_centralizada else 0)
                else:
                    twist.angular.z = base_vel_angular * (self.direcao_area_solo_azul if not area_solo_azul_centralizada else 0)
                twist.linear.x = base_vel_linear

        elif self.estado_atual == Estados.PERMANECENDO_AREA_SOLO_AZUL:
            # Aguarda tempo_reabilitar_parede_dentro_solo_azul segundos após adentrar a área
            # antes de voltar a usar self.parede_a_frente para ir tangenciar paredes.
            # Durante esse período, a supressão iniciada em NAVEGANDO_PARA_AREA_SOLO_AZUL ainda está ativa.
            tempo_dentro_solo_azul = time.time() - self.tempo_inicio_dentro_solo_azul
            parede_ainda_suprimida = (tempo_dentro_solo_azul < self.tempo_reabilitar_parede_dentro_solo_azul)  # parametro: tempo_reabilitar_parede_dentro_solo_azul
            if not parede_ainda_suprimida and self.parede_suprimida_rumo_solo_azul:
                self.parede_suprimida_rumo_solo_azul = False
                self.get_logger().info(f"Detecção de parede para tangenciamento REABILITADA ({self.tempo_reabilitar_parede_dentro_solo_azul:.1f}s dentro da área de solo azul).")

            if self.bandeira_a_frente:
                self.get_logger().info("Bandeira azul localizada dentro da área de solo azul! Iniciando navegação até a bandeira.")
                self.estado_atual = Estados.NAVEGANDO_PARA_BANDEIRA
            elif self.obstaculo_a_frente:
                if self.arena_identificada == 'PAREDES':
                    if parede_ainda_suprimida:
                        # Parede ainda suprimida: aguarda o tempo de reabilitação antes de tangenciar
                        self.get_logger().info(f"Obstáculo detectado, mas detecção de parede ainda suprimida ({self.tempo_reabilitar_parede_dentro_solo_azul - tempo_dentro_solo_azul:.1f}s restantes). Aguardando...")
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                    else:
                        # Comportamento idêntico ao início da simulação (APROXIMANDO_PAREDE)
                        # sem mais procurar área azul
                        self.get_logger().info("Obstáculo detectado na área de solo azul (Arena Paredes). Detecção de parede reabilitada. Iniciando busca por parede RGB (igual ao início da simulação)...")
                        self.estado_atual = Estados.APROXIMANDO_PAREDE
                else:
                    self.estado_origem_desvio = Estados.PERMANECENDO_AREA_SOLO_AZUL
                    self.contador_oscilacao = 0
                    self.direcao_desvio_anterior = None
                    self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO
            elif not self.area_solo_azul_a_frente:
                self.get_logger().info("Robô saiu da área de solo azul sem encontrar a bandeira! Iniciando caminhada de 2m seguida de giro de 700°.")
                self.x_ini_caminhada_reorientacao = self.current_x
                self.y_ini_caminhada_reorientacao = self.current_y
                self.estado_atual = Estados.GIRANDO_180_AREA_SOLO_AZUL
            else:
                self.get_logger().info("Permanecendo na área de solo azul, andando em linha reta procurando pela bandeira azul...")
                twist.linear.x = base_vel_linear
                twist.angular.z = 0.0

        elif self.estado_atual == Estados.GIRANDO_180_AREA_SOLO_AZUL:
            if self.bandeira_a_frente:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("Bandeira avistada durante o giro! Cessando o giro e navegando até ela.")
                self.estado_atual = Estados.NAVEGANDO_PARA_BANDEIRA
            elif self.area_solo_azul_a_frente:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("Área de solo azul avistada durante o giro! Cessando o giro e navegando até ela.")
                self.estado_atual = Estados.NAVEGANDO_PARA_AREA_SOLO_AZUL
            elif self.obstaculo_a_frente or self.obstaculo_a_esquerda or self.obstaculo_a_direita or self.obstaculo_atras:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("Obstáculo detectado durante o giro! Abortando giro e retomando exploração.")
                self.estado_atual = Estados.EXPLORANDO
            else:
                distancia_caminhada = np.hypot(self.current_x - self.x_ini_caminhada_reorientacao, self.current_y - self.y_ini_caminhada_reorientacao)
                if distancia_caminhada < self.distancia_caminhada_reorientacao:
                    twist.linear.x = base_vel_linear * 0.8
                    twist.angular.z = 0.0
                else:
                    delta_yaw = self.current_yaw - self.yaw_anterior_giro_180_area_azul
                    delta_yaw = np.arctan2(np.sin(delta_yaw), np.cos(delta_yaw))
                    self.angulo_acumulado_giro_180_area_azul += abs(delta_yaw)
                    self.yaw_anterior_giro_180_area_azul = self.current_yaw

                    if self.angulo_acumulado_giro_180_area_azul >= np.radians(700):
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.get_logger().info("Giro de 700° concluído. Retomando exploração.")
                        self.estado_atual = Estados.EXPLORANDO
                    else:
                        twist.linear.x = 0.0
                        twist.angular.z = base_vel_angular * self.direcao_giro_180_area_azul

        ## APROXIMANDO DA PAREDE (ARENA PAREDES)
        elif self.estado_atual == Estados.APROXIMANDO_PAREDE:
            if self.bandeira_a_frente:
                self.get_logger().info("Bandeira detectada durante a aproximação da parede! Priorizando a bandeira.")
                self.estado_atual = Estados.NAVEGANDO_PARA_BANDEIRA
            # Removido priorização de área azul aqui para evitar loops quando já está em permanência
            elif not self.parede_a_frente:
                if self.obstaculo_a_frente:
                    self.estado_origem_desvio = Estados.APROXIMANDO_PAREDE
                    self.contador_oscilacao = 0
                    self.direcao_desvio_anterior = None
                    self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO
                else:
                    self.get_logger().info("Procurando a parede (cor de referência) em linha reta...")
                    twist.linear.x = base_vel_linear
                    twist.angular.z = 0.0
            else:
                leituras_frente_parede = [d for d in (self.distancias_frente_esquerda + self.distancias_frente_direita) if np.isfinite(d) and d > 0.05]
                dist_atual_parede = min(leituras_frente_parede) if leituras_frente_parede else 10.0

                if dist_atual_parede <= self.distancia_aproximacao_parede:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    # Determina, de forma FIXA para este encontro com a parede, qual lado do
                    # sensor frontal (esquerda/direita) está mais próximo dela. O robô gira para
                    # o lado OPOSTO e passa a tangenciar a parede por esse mesmo lado mais
                    # próximo (ex.: parede mais próxima à esquerda -> gira para a direita e
                    # tangencia usando o sensor lateral esquerdo).
                    leituras_frente_esq_lado_parede = [d for d in self.distancias_frente_esquerda if np.isfinite(d) and d > 0.05]
                    leituras_frente_dir_lado_parede = [d for d in self.distancias_frente_direita if np.isfinite(d) and d > 0.05]
                    dist_frente_esq_lado_parede = min(leituras_frente_esq_lado_parede) if leituras_frente_esq_lado_parede else 10.0
                    dist_frente_dir_lado_parede = min(leituras_frente_dir_lado_parede) if leituras_frente_dir_lado_parede else 10.0
                    self.lado_seguindo_parede = Direcoes.ESQUERDA.value if dist_frente_esq_lado_parede <= dist_frente_dir_lado_parede else Direcoes.DIREITA.value
                    lado_txt_parede = 'ESQUERDA' if self.lado_seguindo_parede == Direcoes.ESQUERDA.value else 'DIREITA'
                    self.get_logger().info(f"Parede alcançada a {dist_atual_parede:.2f}m! Lado mais próximo (sensor frontal): {lado_txt_parede}. Girando 90° para o lado oposto e tangenciando por esse lado...")
                    self.yaw_ini_giro_paralelo_parede = self.current_yaw
                    self.estado_atual = Estados.GIRANDO_PARALELO_PAREDE
                elif self.obstaculo_a_frente:
                    self.estado_origem_desvio = Estados.APROXIMANDO_PAREDE
                    self.contador_oscilacao = 0
                    self.direcao_desvio_anterior = None
                    self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO
                else:
                    if (direcao_parede == Direcoes.ESQUERDA.value and self.obstaculo_a_esquerda) or (direcao_parede == Direcoes.DIREITA.value and self.obstaculo_a_direita):
                        twist.angular.z = base_vel_angular * 0.50 * (direcao_parede if not parede_centralizada else 0)
                    else:
                        twist.angular.z = base_vel_angular * (direcao_parede if not parede_centralizada else 0)
                    twist.linear.x = base_vel_linear

        ## GIRANDO PARA FICAR PARALELO/TANGENTE À PAREDE (ARENA PAREDES)
        elif self.estado_atual == Estados.GIRANDO_PARALELO_PAREDE:
            if self.bandeira_a_frente:
                self.get_logger().info("Bandeira detectada durante o giro para ficar paralelo à parede! Priorizando a bandeira.")
                self.estado_atual = Estados.NAVEGANDO_PARA_BANDEIRA
            else:
                # O giro de 90° é OBRIGATÓRIO e executado por completo: o sensor de desvio de
                # obstáculo (frontal e lateral) permanece DESATIVADO durante todo o giro.
                delta_yaw_parede = self.current_yaw - self.yaw_ini_giro_paralelo_parede
                delta_yaw_parede = np.arctan2(np.sin(delta_yaw_parede), np.cos(delta_yaw_parede))
                if abs(delta_yaw_parede) < np.radians(90):
                    twist.linear.x = 0.0
                    twist.angular.z = base_vel_angular * (-self.lado_seguindo_parede)
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info("Giro de 90° concluído. Iniciando caminhada tangenciando a parede...")
                    self.estado_atual = Estados.SEGUINDO_PAREDE

        ## SEGUINDO A PAREDE TANGENCIALMENTE (ARENA PAREDES)
        elif self.estado_atual == Estados.SEGUINDO_PAREDE:
            if self.bandeira_a_frente:
                self.get_logger().info("Bandeira detectada durante o seguimento da parede! Priorizando a bandeira.")
                self.estado_atual = Estados.NAVEGANDO_PARA_BANDEIRA
            elif self.obstaculo_frente_oposto_parede():
                self.get_logger().info("Obstáculo real (lado oposto à parede) detectado à frente durante o seguimento da parede! Desviando...")
                self.estado_origem_desvio = Estados.SEGUINDO_PAREDE
                self.contador_oscilacao = 0
                self.direcao_desvio_anterior = None
                self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO
            else:
                distancias_lado_seguido_parede = self.distancias_esquerda if self.lado_seguindo_parede == Direcoes.ESQUERDA.value else self.distancias_direita
                leituras_lado_seguido_parede = [d for d in distancias_lado_seguido_parede if np.isfinite(d) and d > 0.05]
                dist_lateral_parede = min(leituras_lado_seguido_parede) if leituras_lado_seguido_parede else 999.0

                # Velocidade linear reduzida (fase de seguimento exige correção mais fina)
                velocidade_seguindo_parede = base_vel_linear * self.fator_velocidade_arena_paredes  # parametro: fator_velocidade_arena_paredes

                if dist_lateral_parede < self.distancia_minima_contato_parede or dist_lateral_parede > self.distancia_maxima_contato_parede:
                    # Fora da faixa de histerese de tangenciamento: contato com a parede perdido.
                    # Antes de iniciar o giro, avança 0.8s em linha reta para ultrapassar a quina.
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info("Contato lateral com a parede perdido! Avançando 0.8s antes do giro pós-parede...")
                    self.ap_tempo_ini_avanco_pre_giro = time.time()
                    self.estado_atual = Estados.AP_AVANCO_PRE_GIRO_NOVA_PAREDE
                elif dist_lateral_parede < self.distancia_minima_seguindo_parede:
                    # Muito perto da parede (mas ainda dentro da faixa de tangenciamento): afasta
                    # um pouco, virando para o lado oposto, sempre mantendo-se em movimento
                    twist.linear.x = velocidade_seguindo_parede
                    twist.angular.z = base_vel_angular * 0.5 * (-self.lado_seguindo_parede)
                elif dist_lateral_parede > self.distancia_maxima_seguindo_parede:
                    # Muito longe da parede (mas ainda dentro da faixa de tangenciamento):
                    # aproxima um pouco, sempre mantendo-se em movimento
                    twist.linear.x = velocidade_seguindo_parede
                    twist.angular.z = base_vel_angular * 0.5 * self.lado_seguindo_parede
                else:
                    # Dentro da faixa de correção fina: segue reto
                    twist.linear.x = velocidade_seguindo_parede
                    twist.angular.z = 0.0

        ## AVANÇO EM LINHA RETA DE 0.8s ANTES DO GIRO PÓS-PAREDE (ARENA PAREDES)
        ## Inserido entre a perda de contato lateral e o giro, para ultrapassar a quina da parede.
        ## Só a bandeira pode interromper este avanço (área solo azul e parede: desativadas).
        elif self.estado_atual == Estados.AP_AVANCO_PRE_GIRO_NOVA_PAREDE:
            if self.bandeira_a_frente:
                self.get_logger().info("Bandeira detectada durante o avanço pré-giro! Priorizando a bandeira.")
                self.estado_atual = Estados.NAVEGANDO_PARA_BANDEIRA
            else:
                decorrido_pre_giro = time.time() - self.ap_tempo_ini_avanco_pre_giro
                if decorrido_pre_giro < self.ap_duracao_avanco_pre_giro:
                    # Avança em linha reta durante 0.8s  #key parametro: ap_duracao_avanco_pre_giro
                    twist.linear.x = base_vel_linear * self.fator_velocidade_arena_paredes
                    twist.angular.z = 0.0
                else:
                    # Tempo esgotado: inicia o protocolo Giro+Avanço
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info(f"Avanço pré-giro de {self.ap_duracao_avanco_pre_giro}s concluído. Iniciando giro pós-parede...")
                    self.yaw_ini_giro_nova_parede = self.current_yaw
                    self.contador_ciclos_nova_parede = 0  # inicia o protocolo de 2 ciclos
                    self.estado_atual = Estados.GIRANDO_90_NOVA_PAREDE

        ## GIRO DE 90° APÓS PERDER A PAREDE, À PROCURA DE UMA NOVA (ARENA PAREDES)
        elif self.estado_atual == Estados.GIRANDO_90_NOVA_PAREDE:
            # Giro OBRIGATÓRIO: parede e área de solo azul estão DESATIVADAS durante todo
            # este estado. Só a bandeira (prioridade máxima) pode interromper.
            # A detecção de solo azul e de parede é verificada apenas ao fim do protocolo
            # completo (ao final de AVANCANDO_FIXO_NOVA_PAREDE ciclo 2/2).
            if self.bandeira_a_frente:
                self.get_logger().info("Bandeira detectada durante o giro pós-parede! Priorizando a bandeira.")
                self.estado_atual = Estados.NAVEGANDO_PARA_BANDEIRA
            else:
                # Sensor de desvio de obstáculo (frontal e lateral) permanece DESATIVADO durante
                # este giro de 90° para o lado da parede: só é reativado após o avanço fixo de
                # 0.5m em AVANCANDO_FIXO_NOVA_PAREDE.
                delta_yaw_nova_parede = self.current_yaw - self.yaw_ini_giro_nova_parede
                delta_yaw_nova_parede = np.arctan2(np.sin(delta_yaw_nova_parede), np.cos(delta_yaw_nova_parede))
                # Ciclo 0: giro de 49°; Ciclo 1: giro de 52.5° (metade)
                angulo_giro_ciclo = 65.0 if self.contador_ciclos_nova_parede == 0 else 42.5  #key
                if abs(delta_yaw_nova_parede) < np.radians(angulo_giro_ciclo):
                    twist.linear.x = 0.0
                    twist.angular.z = base_vel_angular * self.lado_seguindo_parede
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info(f"Giro de {angulo_giro_ciclo}° concluído (ciclo {self.contador_ciclos_nova_parede + 1}/2). Iniciando avanço fixo...")
                    self.x_ini_avanco_fixo_nova_parede = self.current_x
                    self.y_ini_avanco_fixo_nova_parede = self.current_y
                    self.estado_atual = Estados.AVANCANDO_FIXO_NOVA_PAREDE

        ## AVANÇO FIXO EM LINHA RETA APÓS O GIRO, ANTES DE RETOMAR A BUSCA POR NOVA PAREDE (ARENA PAREDES)
        elif self.estado_atual == Estados.AVANCANDO_FIXO_NOVA_PAREDE:
            # Avanço OBRIGATÓRIO: parede e área de solo azul estão DESATIVADAS durante todo
            # este estado. Só a bandeira (prioridade máxima) pode interromper.
            # Ao fim do protocolo completo (ciclo 2/2), a hierarquia é verificada: bandeira >
            # solo azul > parede, a partir de uma posição e orientação estáveis.
            if self.bandeira_a_frente:
                self.get_logger().info("Bandeira detectada durante o avanço fixo pós-parede! Priorizando a bandeira.")
                self.estado_atual = Estados.NAVEGANDO_PARA_BANDEIRA
            else:
                # Sensor de desvio de obstáculo (frontal e lateral) ainda DESATIVADO durante este
                # avanço fixo; só volta a ser considerado em APROXIMANDO_PAREDE logo abaixo, ao
                # retomar oficialmente a busca por uma nova parede.
                # Ciclo 0: avança 0.5m; Ciclo 1: avança 0.25m (metade)
                distancia_alvo_ciclo = self.distancia_avanco_fixo_nova_parede if self.contador_ciclos_nova_parede == 0 else self.distancia_avanco_fixo_nova_parede / 2.0
                distancia_avancada_fixa_parede = np.hypot(self.current_x - self.x_ini_avanco_fixo_nova_parede, self.current_y - self.y_ini_avanco_fixo_nova_parede)
                if distancia_avancada_fixa_parede < distancia_alvo_ciclo:
                    twist.linear.x = base_vel_linear * self.fator_velocidade_arena_paredes  # parametro: fator_velocidade_arena_paredes
                    twist.angular.z = 0.0
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    if self.contador_ciclos_nova_parede == 0:
                        # 1º ciclo concluído: prepara o 2º
                        self.get_logger().info(f"Avanço de {distancia_alvo_ciclo}m concluído (ciclo 1/2). Iniciando 2º ciclo: giro de 27.5° + avanço de {self.distancia_avanco_fixo_nova_parede / 2.0}m...")
                        self.contador_ciclos_nova_parede = 1
                        self.yaw_ini_giro_nova_parede = self.current_yaw
                        self.estado_atual = Estados.GIRANDO_90_NOVA_PAREDE
                    else:
                        # 2º ciclo concluído: protocolo completo. Agora verifica a hierarquia
                        # completa a partir de uma posição estável antes de retomar o tangenciamento.
                        self.contador_ciclos_nova_parede = 0  # zera para o próximo uso
                        if self.bandeira_a_frente:
                            self.get_logger().info("Protocolo Giro+Avanço concluído. Bandeira detectada! Priorizando.")
                            self.estado_atual = Estados.NAVEGANDO_PARA_BANDEIRA
                        elif self.area_solo_azul_a_frente:
                            self.get_logger().info(">>> Protocolo Giro+Avanço concluído. ÁREA DE SOLO AZUL DETECTADA! Navegando para a área. <<<")
                            self.ap_contador_perda_solo_azul = 0
                            self.tempo_inicio_navegando_para_solo_azul = time.time()
                            self.parede_suprimida_rumo_solo_azul = False
                            self.estado_atual = Estados.NAVEGANDO_PARA_AREA_SOLO_AZUL
                        else:
                            self.get_logger().info(f"Avanço de {distancia_alvo_ciclo}m concluído (ciclo 2/2). Sensores de obstáculo reativados. Retomando a busca por uma nova parede...")
                            self.estado_atual = Estados.APROXIMANDO_PAREDE

        ## RETORNANDO PARA BASE
        elif self.estado_atual == Estados.RETORNANDO_PARA_BASE:
            self.distancia_max_obstaculo_frente = 0.79

            if not self.fase_inicial_retorno_concluida:
                # Primeira coisa a ser feita ao entrar em RETORNANDO_PARA_BASE: avança 0.47m e
                # em seguida gira 149°.
                # TESTE DE DEBUG: nesta fase o sensor frontal fica desativado (não é verificado) e o
                # desvio de obstáculo passa a usar somente os sensores laterais, reconfigurados para
                # operar na mesma faixa angular que o sensor frontal usa. Antes de ajustar os sensores,
                # antes de iniciar o avanço e após o término do giro, o robô aguarda parado por
                # self.duracao_pausa_retorno (1.7s) em cada uma dessas etapas, para tentar isolar a causa do bug.
                # self.subfase_inicial_retorno: 0 = pausa inicial, 1 = pausa pós-ajuste de sensores,
                # 2 = avançando 0.47m, 3 = girando 149°, 4 = pausa pós-giro, concluído quando
                # fase_inicial_retorno_concluida=True
                if self.subfase_inicial_retorno == 0:
                    # Pausa de 1.7s logo ao ativar o RETORNANDO_PARA_BASE, antes de qualquer ajuste de sensor
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    if time.time() - self.tempo_inicio_subfase_retorno >= self.duracao_pausa_retorno:
                        self.get_logger().info("Pausa inicial de 1.7s concluída. Ajustando sensores do LiDAR (frontal desativado, laterais na faixa frontal)...")
                        self._indices_esquerda_orig_retorno = list(self.indices_esquerda)
                        self._indices_direita_orig_retorno = list(self.indices_direita)
                        self.indices_esquerda = list(self.indices_frente_esquerda)
                        self.indices_direita = list(self.indices_frente_direita)
                        self.subfase_inicial_retorno = 1
                        self.tempo_inicio_subfase_retorno = time.time()
                elif self.subfase_inicial_retorno == 1:
                    # Pausa de 1.7s após o ajuste dos sensores, antes de iniciar o avanço
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    if time.time() - self.tempo_inicio_subfase_retorno >= self.duracao_pausa_retorno:
                        self.get_logger().info("Pausa pós-ajuste de sensores concluída. Iniciando avanço de 0.47m...")
                        self.x_ini_avanco_inicial_retorno = self.current_x
                        self.y_ini_avanco_inicial_retorno = self.current_y
                        self.subfase_inicial_retorno = 2
                elif self.subfase_inicial_retorno == 2:
                    if self.obstaculo_a_esquerda or self.obstaculo_a_direita:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.get_logger().info("Obstáculo detectado pelos sensores laterais (faixa frontal) durante o avanço inicial do retorno! Suspendendo manobra, restaurando sensores e prosseguindo com a busca normal pela base.")
                        self.indices_esquerda = self._indices_esquerda_orig_retorno
                        self.indices_direita = self._indices_direita_orig_retorno
                        self.fase_inicial_retorno_concluida = True
                    else:
                        distancia_avancada_inicial_retorno = np.hypot(self.current_x - self.x_ini_avanco_inicial_retorno, self.current_y - self.y_ini_avanco_inicial_retorno)
                        if distancia_avancada_inicial_retorno < self.distancia_avanco_inicial_retorno:
                            self.get_logger().info(f"Andando para frente {self.distancia_avanco_inicial_retorno}m antes de buscar a base...")
                            twist.linear.x = base_vel_linear
                            twist.angular.z = 0.0
                        else:
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                            self.get_logger().info("Avanço de 0.47m concluído. Girando 149°...")
                            self.yaw_ini_giro_inicial_retorno = self.current_yaw
                            self.subfase_inicial_retorno = 3
                elif self.subfase_inicial_retorno == 3:
                    if self.obstaculo_a_esquerda or self.obstaculo_a_direita:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        self.get_logger().info("Obstáculo detectado pelos sensores laterais (faixa frontal) durante o giro inicial do retorno! Suspendendo manobra, restaurando sensores e prosseguindo com a busca normal pela base.")
                        self.indices_esquerda = self._indices_esquerda_orig_retorno
                        self.indices_direita = self._indices_direita_orig_retorno
                        self.fase_inicial_retorno_concluida = True
                    else:
                        delta_yaw_inicial_retorno = self.current_yaw - self.yaw_ini_giro_inicial_retorno
                        delta_yaw_inicial_retorno = np.arctan2(np.sin(delta_yaw_inicial_retorno), np.cos(delta_yaw_inicial_retorno))
                        if abs(delta_yaw_inicial_retorno) < np.radians(149):
                            self.get_logger().info("Girando 149°...")
                            twist.linear.x = 0.0
                            twist.angular.z = base_vel_angular * self.direcao_giro_inicial_retorno
                        else:
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                            self.get_logger().info("Giro de 149° concluído. Restaurando sensores do LiDAR. Aguardando 1.7s antes de iniciar a busca pela base...")
                            self.indices_esquerda = self._indices_esquerda_orig_retorno
                            self.indices_direita = self._indices_direita_orig_retorno
                            self.subfase_inicial_retorno = 4
                            self.tempo_inicio_subfase_retorno = time.time()
                else:
                    # Pausa de 1.7s após o giro, com os sensores já restaurados, antes de
                    # prosseguir com a busca normal pela base
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    if time.time() - self.tempo_inicio_subfase_retorno >= self.duracao_pausa_retorno:
                        self.get_logger().info("Pausa pós-giro de 1.7s concluída. Iniciando busca pela base...")
                        self.fase_inicial_retorno_concluida = True

            elif self.base_a_frente and self.porcentagem_base_na_camera >= 7.1:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.distancia_max_obstaculo_frente = 0.63
                self.get_logger().info("Base alcançada! Iniciando centralização para o depósito...")
                self.estado_atual = Estados.POSICIONANDO_PARA_DEPOSITO
                self.contador_oscilacao_centralizacao_base = 0
                self.tempo_inicio_retorno = time.time()

            elif self.obstaculo_a_frente:
                self.estado_origem_desvio = Estados.RETORNANDO_PARA_BASE
                self.contador_oscilacao = 0
                self.direcao_desvio_anterior = None
                self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO

            elif self.base_a_frente:
                base_centralizada = (
                    self.pos_x_base_camera <= self.centro_x_camera + dx and
                    self.pos_x_base_camera >= self.centro_x_camera - dx
                )
                self.direcao_base = 1 if self.pos_x_base_camera < self.centro_x_camera - dx else -1
                self.get_logger().info(f"Base visível ({self.porcentagem_base_na_camera:.1f}%). Navegando...")
                if (self.direcao_base == Direcoes.ESQUERDA.value and self.obstaculo_a_esquerda) or \
                   (self.direcao_base == Direcoes.DIREITA.value and self.obstaculo_a_direita):
                    twist.angular.z = base_vel_angular * 0.50 * (self.direcao_base if not base_centralizada else 0)
                else:
                    twist.angular.z = base_vel_angular * (self.direcao_base if not base_centralizada else 0)
                twist.linear.x = base_vel_linear * (0.5 if self.porcentagem_base_na_camera >= 1.5 else 1.0)

            elif self.dentro_da_regiao_base:
                self.get_logger().info("Dentro da região da base. Avançando em linha reta...")
                twist.linear.x = base_vel_linear
                twist.angular.z = 0.0

            elif self.regiao_base_a_frente:
                regiao_centralizada = (
                    self.pos_x_regiao_camera <= self.centro_x_camera + dx and
                    self.pos_x_regiao_camera >= self.centro_x_camera - dx
                )
                direcao_regiao = 1 if self.pos_x_regiao_camera < self.centro_x_camera - dx else -1
                self.get_logger().info("Região da base visível. Navegando para a região...")
                if (direcao_regiao == Direcoes.ESQUERDA.value and self.obstaculo_a_esquerda) or \
                   (direcao_regiao == Direcoes.DIREITA.value and self.obstaculo_a_direita):
                    twist.angular.z = base_vel_angular * 0.50 * (direcao_regiao if not regiao_centralizada else 0)
                else:
                    twist.angular.z = base_vel_angular * (direcao_regiao if not regiao_centralizada else 0)
                twist.linear.x = base_vel_linear

            else:
                self.get_logger().info("Sem referência visual da base. Explorando aleatoriamente...")
                twist.linear.x = base_vel_linear
                twist.angular.z = base_vel_angular * self.direcao_aleatoria * 0.3

        ## POSICIONANDO PARA DEPÓSITO
        elif self.estado_atual == Estados.POSICIONANDO_PARA_DEPOSITO:
            dx_base = 1
            base_centralizada_deposito = self.pos_x_base_camera <= self.centro_x_camera + dx_base and self.pos_x_base_camera >= self.centro_x_camera - dx_base
            direcao_base_deposito = 1 if (self.pos_x_base_camera < self.centro_x_camera - dx_base) else -1

            if self.centralizacao_deposito_cancelada:
                self.get_logger().info("Centralização com a base cancelada após 4 desvios de obstáculo consecutivos. Prosseguindo com a colocação da bandeira do jeito que está...")
                self.centralizacao_deposito_cancelada = False
                self.contador_desvios_centralizacao_deposito = 0
                self.avanco_deposito_reduzido = True
                self.x_ini_avanco_deposito = self.current_x
                self.y_ini_avanco_deposito = self.current_y
                self.estado_atual = Estados.AVANCANDO_PARA_DEPOSITO
            elif not self.base_a_frente:
                self.get_logger().info("Base perdida durante a centralização, retomando navegação para a base.")
                self.estado_atual = Estados.RETORNANDO_PARA_BASE
            elif self.obstaculo_a_frente:
                self.contador_desvios_centralizacao_deposito += 1
                if self.contador_desvios_centralizacao_deposito >= 4:
                    self.get_logger().info("Centralização com a base interrompida por desvio de obstáculo pela 4ª vez consecutiva! Centralização será cancelada após este desvio.")
                    self.centralizacao_deposito_cancelada = True
                else:
                    self.get_logger().info("Obstáculo detectado durante centralização para depósito! Cancelando e desviando.")
                self.estado_origem_desvio = Estados.POSICIONANDO_PARA_DEPOSITO
                self.contador_oscilacao = 0
                self.direcao_desvio_anterior = None
                self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO
            elif not base_centralizada_deposito:
                direcao_atual_base = 1 if self.pos_x_base_camera < self.centro_x_camera else -1
                if self.direcao_centralizacao_anterior_base is not None and direcao_atual_base != self.direcao_centralizacao_anterior_base:
                    self.contador_oscilacao_centralizacao_base += 1
                self.direcao_centralizacao_anterior_base = direcao_atual_base

                if self.contador_oscilacao_centralizacao_base >= 5:
                    self.get_logger().info("Oscilação excessiva na centralização com a base! Prosseguindo para o depósito mesmo assim.")
                    self.contador_oscilacao_centralizacao_base = 0
                    self.contador_desvios_centralizacao_deposito = 0
                    self.get_logger().info("Base alcançada! Avançando os 0.51m finais antes de depositar...")
                    self.x_ini_avanco_deposito = self.current_x
                    self.y_ini_avanco_deposito = self.current_y
                    self.estado_atual = Estados.AVANCANDO_PARA_DEPOSITO
                else:
                    self.get_logger().info("Centralizando com a base para o depósito...")
                    twist.angular.z = (base_vel_angular * 0.25) * direcao_base_deposito
            else:
                self.contador_oscilacao_centralizacao_base = 0
                self.contador_desvios_centralizacao_deposito = 0
                self.get_logger().info("Base centralizada! Avançando os 0.51m finais antes de depositar...")
                self.x_ini_avanco_deposito = self.current_x
                self.y_ini_avanco_deposito = self.current_y
                self.estado_atual = Estados.AVANCANDO_PARA_DEPOSITO

        ## AVANCANDO PARA DEPÓSITO
        elif self.estado_atual == Estados.AVANCANDO_PARA_DEPOSITO:
            distancia_alvo_avanco_deposito = self.distancia_avanco_deposito_reduzido if self.avanco_deposito_reduzido else self.distancia_avanco_deposito
            if self.obstaculo_a_frente:
                self.get_logger().info("Obstáculo detectado antes do avanço final para depósito! Cancelando depósito e desviando.")
                self.estado_origem_desvio = Estados.AVANCANDO_PARA_DEPOSITO
                self.contador_oscilacao = 0
                self.direcao_desvio_anterior = None
                self.estado_atual = Estados.DESVIANDO_DE_OBSTACULO
            else:
                distancia_avancada_deposito = np.hypot(self.current_x - self.x_ini_avanco_deposito, self.current_y - self.y_ini_avanco_deposito)
                if distancia_avancada_deposito >= distancia_alvo_avanco_deposito:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    if self.avanco_deposito_reduzido:
                        self.get_logger().info("Avanço reduzido de 0.11m concluído (centralização cancelada por desvios). Preparando garra para depósito...")
                    else:
                        self.get_logger().info("Avanço final de 0.51m concluído. Preparando garra para depósito...")
                    self.estado_atual = Estados.RESETANDO_GARRA_BASE
                else:
                    twist.linear.x = 0.25
                    twist.angular.z = 0.0

        ## RESETANDO GARRA BASE
        elif self.estado_atual == Estados.RESETANDO_GARRA_BASE:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if self._zona_cega_mastro_ativa:
                self.indices_frente_esquerda = self._indices_frente_esquerda_orig
                self.indices_frente_direita = self._indices_frente_direita_orig
                self._zona_cega_mastro_ativa = False
                self.get_logger().info("Zona cega frontal desativada: LiDAR frontal restaurado.")
            self.get_logger().info("[Alinhamento Base] Retornando rotação da garra para o ângulo inicial...")
            self.resetar_rotacao_garra()
            self.estado_atual = Estados.SOLTANDO_BANDEIRA
            self.passo_soltura = 0

        ## SOLTANDO BANDEIRA
        elif self.estado_atual == Estados.SOLTANDO_BANDEIRA:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            if self.passo_soltura == 0:
                if self.avanco_deposito_reduzido:
                    self.get_logger().info("[Descarregamento] Estendendo a garra (mesma extensão usada na captura da bandeira) com a bandeira na base...")
                    self.estender_garra()
                    self.avanco_deposito_reduzido = False
                else:
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
                
                # === RELATÓRIO DE TEMPO ===
                tempo_total = time.time() - self.tempo_inicio_missao
                tempo_captura = self.tempo_captura_bandeira - self.tempo_inicio_missao if self.tempo_captura_bandeira else 0
                tempo_retorno = time.time() - self.tempo_inicio_retorno if self.tempo_inicio_retorno else 0

                hours_t = int(tempo_total // 3600)
                minutes_t = int((tempo_total % 3600) // 60)
                seconds_t = int(tempo_total % 60)

                self.get_logger().info(f"🎉 MISSÃO CONCLUÍDA COM SUCESSO!")
                self.get_logger().info(f"⏱️ Tempo até captura da bandeira: {int(tempo_captura//60)}m {int(tempo_captura%60)}s")
                self.get_logger().info(f"⏱️ Tempo de retorno após captura: {int(tempo_retorno//60)}m {int(tempo_retorno%60)}s")
                self.get_logger().info(f"⏱️ Tempo total da simulação: {hours_t:02d}h {minutes_t:02d}m {seconds_t:02d}s")
                self.get_logger().info("Desafio de Captura, Retorno Otimizado e Depósito Concluído com Sucesso!")
                self.timer.cancel()

        ## AP_RETORNO_ODOMETRIA (ARENA PAREDES): navega de volta ao ponto de nascimento
        ## usando odometria. Quando chegar dentro do raio ap_raio_chegada_retorno metros do
        ## start, entrega o controle a RETORNANDO_PARA_BASE (lógica de base/centralização/depósito
        ## idêntica à Arena Cilindros, já implementada). A todo instante, se a base já for visível,
        ## pode ir direto para RETORNANDO_PARA_BASE sem esperar chegar ao ponto de nascimento.
        elif self.estado_atual == Estados.AP_RETORNO_ODOMETRIA:
            # Se a base já estiver à vista, entrega imediatamente ao RETORNANDO_PARA_BASE
            if self.base_a_frente:
                self.get_logger().info("[AP] Base avistada durante retorno por odometria! Entregando controle a RETORNANDO_PARA_BASE.")
                self.ap_retorno_odometria_ativo = False
                self.estado_atual = Estados.RETORNANDO_PARA_BASE
            else:
                # Calcula vetor do robô até o ponto de nascimento
                dx_ret = self.start_x - self.current_x
                dy_ret = self.start_y - self.current_y
                dist_ret = np.hypot(dx_ret, dy_ret)
                self.get_logger().info(f"[AP] Retorno odometria: distância ao start = {dist_ret:.2f}m (alvo ≤ {self.ap_raio_chegada_retorno}m)")

                if dist_ret <= self.ap_raio_chegada_retorno:
                    # Chegou à vizinhança do ponto de nascimento: entrega ao RETORNANDO_PARA_BASE
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info(f"[AP] Chegou ao ponto de nascimento ({dist_ret:.2f}m ≤ {self.ap_raio_chegada_retorno}m). Entregando controle a RETORNANDO_PARA_BASE.")
                    self.ap_retorno_odometria_ativo = False
                    self.estado_atual = Estados.RETORNANDO_PARA_BASE
                else:
                    # Orienta o robô em direção ao ponto de nascimento e avança
                    angulo_alvo_ret = np.arctan2(dy_ret, dx_ret)
                    erro_yaw_ret = angulo_alvo_ret - self.current_yaw
                    erro_yaw_ret = np.arctan2(np.sin(erro_yaw_ret), np.cos(erro_yaw_ret))
                    # Desvio de obstáculo durante o retorno: se tiver algo à frente, gira
                    if self.obstaculo_a_frente:
                        twist.linear.x = 0.0
                        # Gira para o lado com mais espaço
                        leit_esq_ret = [d for d in self.distancias_esquerda if np.isfinite(d) and d > 0.05]
                        leit_dir_ret = [d for d in self.distancias_direita if np.isfinite(d) and d > 0.05]
                        dist_esq_ret = min(leit_esq_ret) if leit_esq_ret else 999.0
                        dist_dir_ret = min(leit_dir_ret) if leit_dir_ret else 999.0
                        twist.angular.z = base_vel_angular * (1 if dist_esq_ret >= dist_dir_ret else -1)
                    else:
                        # Proporcional: quanto maior o erro de yaw, mais gira; quanto menor, mais avança
                        twist.linear.x = base_vel_linear * max(0.3, 1.0 - abs(erro_yaw_ret) / np.pi)
                        twist.angular.z = base_vel_angular * np.clip(erro_yaw_ret / (np.pi / 4), -1.0, 1.0)

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
