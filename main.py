import cv2
import numpy as np
import time
import os

try:
    from gpiozero import PWMOutputDevice
except ImportError:
    PWMOutputDevice = None

WINDOW_NAME = "Seguidor de Linha"
THRESHOLD_NAME = "Threshold"

# Objetos dos motores
motor_left = None
motor_right = None

velocity = 0.20     # Variável global para armazenar a velocidade do robô (0 a 1)

last_error = 0.0    # Variável global para armazenar o último erro
last_time = 0       # Variável global para armazenar o último tempo

def setup_motors():
    # Configura os pinos dos motores usando gpiozero com PWM.
    global motor_left, motor_right

    if PWMOutputDevice is None:
        print("gpiozero não encontrado. Modo simulado ativo.")
        return

    try:

        # GPIO18 -> IN1 (pino físico 12)
        # GPIO19 -> IN2 (pino físico 35)
        # GPIO12 -> IN3 (pino físico 32)
        # GPIO13 -> IN4 (pino físico 33)
        # Motor esquerdo (IN1 e IN2)
        motor_left = {
            'forward': PWMOutputDevice(18, initial_value=0, frequency=30),  # IN1
            'backward': PWMOutputDevice(19, initial_value=0)  # IN2
        }

        # Motor direito (IN3 e IN4)
        motor_right = {
            'forward': PWMOutputDevice(12, initial_value=0, frequency=30),  # IN3
            'backward': PWMOutputDevice(13, initial_value=0)  # IN4
        }

        # Garante estado inicial desligado
        stop_motors()
    except Exception as e:
        print(f"Erro ao configurar motores: {e}")
        motor_left = None
        motor_right = None


def stop_motors():
    # Para os motores e desliga as saídas PWM.
    if motor_left is None or motor_right is None:
        return

    motor_left['forward'].value = 0
    motor_left['backward'].value = 0
    motor_right['forward'].value = 0
    motor_right['backward'].value = 0


def drive_robot(cx, frame_width):
    # Controla o movimento do robô com base na posição da linha na imagem.
    if motor_left is None or motor_right is None:
        return

    center = frame_width // 2       # Calcula o centro da imagem
    threshold = 5                  # Margem de erro de 21

    global last_error  # Declara que vamos usar a variável global last_error
    global last_time   # Declara que vamos usar a variável global last_time


    if cx is None:                  # Caso não encontre o valor da linha no eixo x
        print("Não vi a linha")
        motor_left["forward"].value = velocity      
        motor_right["forward"].value = velocity
        return

    error = cx - center             # Calcula o erro entre o centro da linha e o centro da imagem
    if abs(error) >= 40:
        error = error * 1.5

    kp = 0.0007                     # Constante proporcional
    proportional = kp * error       # Variavel da correção proporcional em relação ao erro
    
    Kd = 0.0005

    now = time.time()                 # Pega o tempo atual
    dt = now - last_time
    derivative = Kd * (error - last_error) / dt if dt > 0 else 0  # Variavel da correção derivativa em relação ao erro
    last_time = now
    correction = proportional + derivative     # Variavel de correção. OBS: esta variavel foi adicionada pensando em colocar um controlador derivativo somando com o proporcional

    max_correction = 0.30
    correction = max(-max_correction, min(max_correction, correction))

    if abs(error) < threshold:     # Caso o erro seja menor que a tolerância, zera a correção para manter o robô andando reto
        correction = 0

    left_speed = velocity + correction      # Velocidade do motor esquerdo
    right_speed = velocity - correction     # Velocidade do motor direito

    left_speed = max(0, min(1, left_speed))     # Garante que a velocidade do motor esquerdo esteja entre 0 e 1. Evitando valores PWM negativos ou acima de 1
    right_speed = max(0, min(1, right_speed))

    motor_left["forward"].value = left_speed    # Isso é lindo cara, a matemática faz tudo pela gente, sem precisar de condicionais
    motor_left["backward"].value = 0

    motor_right["forward"].value = right_speed  
    motor_right["backward"].value = 0

    last_error = error  # Atualiza o último erro para a próxima iteração

    # if error > threshold:     # Se o valor da linha no eixo x for maior que o centro da imagem + tolerância...
    #     print("Virar para a esquerda")
    #     # Motor esquerdo avançado, motor direito reverso
    #     motor_left['forward'].value = left_speed
    #     motor_left['backward'].value = 0
    #     motor_right['forward'].value = 0
    #     motor_right['backward'].value = right_speed
    # elif error < -threshold:   # Se o valor da linha no eixo x for menor que o centro da imagem - tolerância...
    #     print("Virar para a direita")
    #     # Motor esquerdo reverso, motor direito avançado
    #     motor_left['forward'].value = 0
    #     motor_left['backward'].value = left_speed
    #     motor_right['forward'].value = right_speed
    #     motor_right['backward'].value = 0
    # else:                           # Caso esteja dentro da tolerância
    #     print("Seguindo a linha")
    #     # Ambos os motores avançados com PWM em velocidade lenta
    #     motor_left['forward'].value = left_speed
    #     motor_left['backward'].value = 0
    #     motor_right['forward'].value = right_speed
    #     motor_right['backward'].value = 0


def process_frame(frame):
    """Processa o frame, aplica a região de interesse e detecta a linha."""
    height, width = frame.shape[:2]

    # Região de interesse: metade inferior da imagem
    roi = frame[int(height*0.4):height, :]

    # Converte para escala de cinza e binariza para destacar a linha
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)

    # Procura os contornos encontrados na máscara
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cx = None           # Centro da imagem no eixo x
    cy = None           # Centro da imagem no eixo y

    if contours:        # Caso haja contorno
        bigger = max(contours, key=cv2.contourArea)     # Seleciona a maior mancha branca
        M = cv2.moments(bigger)                         # Calcula os momentos

        if M["m00"] != 0:                               # Se o número de pixels for diferente de zero
            cx = int(M["m10"] / M["m00"])               # Média aritmética das posições X dos pixels em razão do número de pixels, resultando no centro do eixo X
            cy = int(M["m01"] / M["m00"])               # O mesmo que a variavel passada, mas em relação ao eixo y. Os dois juntos resultam no centro da linha

            # Desenha o contorno e o centro da linha na região de interesse
            cv2.drawContours(roi, [bigger], -1, (0, 255, 0), 2)
            cv2.circle(roi, (cx, cy), 5, (0, 0, 255), -1)

            # Desenha a linha central da imagem para referência
            center_x = width // 2
            cv2.line(frame, (center_x, 0), (center_x, height), (255, 0, 0), 2)

    return frame, roi, binary, cx, cy


def main():
    """Função principal do programa."""
    # Tenta abrir a câmera com várias estratégias (fallbacks)
    attempts = [
        ("default index", lambda: cv2.VideoCapture(0)),
        ("v4l2 index", lambda: cv2.VideoCapture(0, cv2.CAP_V4L2)),
        ("v4l2 device", lambda: cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)),
        ("gstreamer index", lambda: cv2.VideoCapture(0, cv2.CAP_GSTREAMER)),
    ]

    cap = None
    for name, opener in attempts:
        try:
            print(f"Tentando abrir câmera ({name})...")
            cap = opener()
            # Ajuste de resolução para câmera USB ou CSI na Raspberry Pi 4
            try:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
            except Exception:
                pass

            # Pequena espera para backend inicializar
            time.sleep(0.2)
            if cap is not None and cap.isOpened():
                print(f"Câmera aberta com sucesso usando: {name}")
                break
            else:
                print(f"Falha ao abrir com: {name}")
                try:
                    cap.release()
                except Exception:
                    pass
                cap = None
        except Exception as e:
            print(f"Erro tentando abrir câmera ({name}): {e}")

    if cap is None or not cap.isOpened():
        print("Não foi possível abrir a câmera. Verifique: \n- Se a câmera está conectada;\n- Se o usuário pertence ao grupo 'video';\n- Se outro processo (libcamera, vlc, etc.) não está usando /dev/video0;")
        # Lista rápida de dispositivos encontrados para ajudar diagnóstico
        try:
            devs = sorted([d for d in os.listdir('/dev') if d.startswith('video')])
            print("/dev contém:", devs)
        except Exception:
            pass
        return

    setup_motors()                      # Configura os motores

    try:
        while True:
            ret, frame = cap.read()     # Lê a câmera e vê se conseguiu conectar
            if not ret:                 # Se não conectou, quebra o loop
                break

            frame, roi, mask, cx, cy = process_frame(frame)
            drive_robot(cx, frame.shape[1])

            cv2.imshow(WINDOW_NAME, frame)      # Mostra a imagem renderizada
            cv2.imshow("Mscara", mask)

            key = cv2.waitKey(1) & 0xFF         # Caso o usuário aperte "q" de "quit", encerre o loop
            if key == ord("q"):
                break
    finally:
        stop_motors()
        # gpiozero limpa automaticamente os recursos
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":      # Execute o programa se tudo esta correto
    main()

