import cv2
import numpy as np

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

WINDOW_NAME = "Seguidor de Linha"

# Pinos utilizados (BCM) conforme informado
# GPIO18 -> IN1 (pino físico 12)
# GPIO19 -> IN2 (pino físico 35)
# GPIO12 -> IN3 (pino físico 32)
# GPIO13 -> IN4 (pino físico 33)
# ENA/ENB (EN1/EN2) não estão sendo usados
IN1 = 18
IN2 = 19
IN3 = 12
IN4 = 13


def setup_motors():
    """Configura os pinos dos motores e inicia o PWM."""
    if GPIO is None:
        print("RPi.GPIO não encontrado. Modo simulado ativo.")
        return None

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    # Configura apenas as entradas de controle (IN1..IN4). Sem pinos ENA/ENB.
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    GPIO.setup(IN3, GPIO.OUT)
    GPIO.setup(IN4, GPIO.OUT)

    # Garante estado inicial desligado
    stop_motors()
    return None


def stop_motors():
    """Para os motores e desliga as saídas."""
    if GPIO is None:
        return

    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)


def drive_robot(cx, frame_width):
    """Controla o movimento do robô com base na posição da linha na imagem."""
    if GPIO is None:
        return

    center = frame_width // 2
    threshold = 20

    if cx is None:
        print("Não vi a linha")
        stop_motors()
        return

    if cx > center + threshold:
        print("Virar para a esquerda")
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
    elif cx < center - threshold:
        print("Virar para a direita")
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
    else:
        print("Seguindo a linha")
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)


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

    cx = None
    cy = None

    if contours:
        bigger = max(contours, key=cv2.contourArea)
        M = cv2.moments(bigger)

        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

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

    setup_motors()
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frame, roi, mask, cx, cy = process_frame(frame)
            drive_robot(cx, frame.shape[1])

            cv2.imshow(WINDOW_NAME, frame)
            cv2.imshow("Máscara", mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    finally:
        stop_motors()
        if GPIO is not None:
            GPIO.cleanup()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

