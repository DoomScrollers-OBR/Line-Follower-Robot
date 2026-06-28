import cv2
import numpy as np

try:
    from gpiozero import DigitalOutputDevice
except ImportError:
    DigitalOutputDevice = None

WINDOW_NAME = "Seguidor de Linha"

# Pinos utilizados (BCM) conforme informado
# GPIO18 -> IN1 (pino físico 12)
# GPIO19 -> IN2 (pino físico 35)
# GPIO12 -> IN3 (pino físico 32)
# GPIO13 -> IN4 (pino físico 33)

# Objetos dos motores
motor_left = None
motor_right = None


def setup_motors():
    """Configura os pinos dos motores usando gpiozero."""
    global motor_left, motor_right
    
    if DigitalOutputDevice is None:
        print("gpiozero não encontrado. Modo simulado ativo.")
        return

    try:
        # Motor esquerdo (IN1 e IN2)
        motor_left = {
            'forward': DigitalOutputDevice(18),  # IN1
            'backward': DigitalOutputDevice(19)  # IN2
        }
        
        # Motor direito (IN3 e IN4)
        motor_right = {
            'forward': DigitalOutputDevice(12),  # IN3
            'backward': DigitalOutputDevice(13)  # IN4
        }
        
        # Garante estado inicial desligado
        stop_motors()
    except Exception as e:
        print(f"Erro ao configurar motores: {e}")
        motor_left = None
        motor_right = None


def stop_motors():
    """Para os motores e desliga as saídas."""
    if motor_left is None or motor_right is None:
        return

    motor_left['forward'].off()
    motor_left['backward'].off()
    motor_right['forward'].off()
    motor_right['backward'].off()


def drive_robot(cx, frame_width):
    """Controla o movimento do robô com base na posição da linha na imagem."""
    if motor_left is None or motor_right is None:
        return

    center = frame_width // 2       # Calcula o centro da imagem
    threshold = 20                  # Margem de erro de 20

    if cx is None:                  # Caso não encontre o valor da linha no eixo x
        print("Não vi a linha")
        stop_motors()
        return

    if cx > center + threshold:     # Se o valor da linha no eixo x for maior que o centro da imagem + tolerância...
        print("Virar para a esquerda")
        # Motor esquerdo avançado, motor direito reverso
        motor_left['forward'].on()
        motor_left['backward'].off()
        motor_right['forward'].off()
        motor_right['backward'].on()
    elif cx < center - threshold:   # Se o valor da linha no eixo x for menor que o centro da imagem - tolerância...
        print("Virar para a direita")
        # Motor esquerdo reverso, motor direito avançado
        motor_left['forward'].off()
        motor_left['backward'].on()
        motor_right['forward'].on()
        motor_right['backward'].off()
    else:                           # Caso esteja dentro da tolerância
        print("Seguindo a linha")
        # Ambos os motores avançados
        motor_left['forward'].on()
        motor_left['backward'].off()
        motor_right['forward'].on()
        motor_right['backward'].off()


def process_frame(frame):
    """Processa o frame, aplica a região de interesse e detecta a linha."""
    height, width = frame.shape[:2]

    # Região de interesse: metade inferior da imagem
    roi = frame[height // 2:height, :]

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
    cap = cv2.VideoCapture(0)

    # Ajuste de resolução para câmera USB ou CSI na Raspberry Pi 4
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    if not cap.isOpened():
        print("Não foi possível abrir a câmera.")
        return

    setup_motors()                      # Configura os motores
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    try:
        while True:
            ret, frame = cap.read()     # Lê a câmera e vê se conseguiu conectar
            if not ret:                 # Se não conectou, quebra o loop
                break

            frame, roi, mask, cx, cy = process_frame(frame)
            drive_robot(cx, frame.shape[1])

            cv2.imshow(WINDOW_NAME, frame)      # Mostra a imagem renderizada
            cv2.imshow("Máscara", mask)

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

