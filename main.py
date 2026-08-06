"""
Hardware:
  - Raspberry Pi 4B (2GB RAM)
  - Ponte H DRV8833: cada motor usa 2 pinos (INx1/INx2), sem pino de
    "enable" separado. A gpiozero ja modela isso com a classe Motor:
    motor.value vai de -1 (re, velocidade maxima) a 1 (frente,
    velocidade maxima), aplicando PWM no pino certo e deixando o
    outro em LOW - exatamente o modo "fast decay" da DRV8833.
  - nSLEEP da DRV8833 precisa estar em nivel alto (ligado direto em
    VM ou com pull-up no modulo) pra ponte funcionar; sem isso os
    motores nao se movem.
  - 1 camera USB, so a de seguir linha.

"""

import cv2
import numpy as np
import time
from gpiozero import Motor

# =========================================================================
# CONFIGURACAO DE PILOTAGEM
# =========================================================================
Kp = 1.8
Kd = 0.7
BASE_SPEED = 25       # velocidade de cruzeiro (escala -50..50)
MAX_SPEED = 50
MIN_SPEED = -MAX_SPEED
DEADZONE = 5            # erro abaixo disso e tratado como "reto"
THRESHOLD = 80          # limiar de binarizacao (preto vs fundo)
MIN_AREA = 11000        # area minima do contorno pra considerar "linha valida"

# =========================================================================
# MOTORES - DRV8833 via gpiozero.Motor
# Ajuste os pinos conforme sua fiacao real com a ponte H.
# =========================================================================
left_motor = Motor(forward=17, backward=18)
right_motor = Motor(forward=12, backward=13)


def move(left_speed, right_speed):
    # left_speed/right_speed ja chegam recortados em [MIN_SPEED, MAX_SPEED]
    # vindos de control()
    left_motor.value = left_speed / MAX_SPEED
    right_motor.value = right_speed / MAX_SPEED


def stop():
    left_motor.stop()
    right_motor.stop()


# =========================================================================
# CAMERA
# =========================================================================

CAMERA_WIDTH, CAMERA_HEIGHT = 160, 120


def start_camera():
    attempts = 0
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    while not cap.isOpened():
        attempts += 1
        print(f"[Camera] Tentando abrir... (tentativa {attempts})")
        time.sleep(1)
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(3, CAMERA_WIDTH)
    cap.set(4, CAMERA_HEIGHT)
    print("[Camera] Pronta")
    return cap


# =========================================================================
# VISAO - acha o centro (line_center_x) da linha preta
# =========================================================================
# Kernel da "abertura" morfologica (erosao seguida de dilatacao): limpa
# ruidos pequenos da mascara binaria antes de procurar contornos, pra
# grãos de sujeira/reflexo na imagem nao virarem "linha" falsa.
kernel = np.ones((3, 3), np.uint8)


def find_line(frame):
    height, width = frame.shape[:2]
    roi = frame[int(height * 0.1):height, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, THRESHOLD, 255, cv2.THRESH_BINARY_INV)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    line_center_x = None
    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) >= MIN_AREA:
            moments = cv2.moments(largest)
            if moments["m00"] != 0:
                line_center_x = int(moments["m10"] / moments["m00"])

    return line_center_x, roi


# =========================================================================
# CONTROLE (PID)
# =========================================================================
last_error = 0
last_time = time.time()


def control(line_center_x, roi):
    global last_error, last_time

    width = roi.shape[1]
    now = time.time()
    dt = max(now - last_time, 0.0001)

    if line_center_x is not None:
        center = width // 2
        error = (line_center_x - center) / center * 100
    else:
        error = last_error  # perdeu a linha: mantem a ultima curva

    if abs(error) < DEADZONE:
        error = 0

    derivative = np.clip((error - last_error) / dt, -300, 300)
    correction = Kp * error + Kd * derivative

    left_speed = np.clip(BASE_SPEED + correction, MIN_SPEED, MAX_SPEED)
    right_speed = np.clip(BASE_SPEED - correction, MIN_SPEED, MAX_SPEED)
    move(left_speed, right_speed)

    last_error = error
    last_time = now


# =========================================================================
# LOOP PRINCIPAL
# =========================================================================
if __name__ == "__main__":
    cap = start_camera()
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            line_center_x, roi = find_line(frame)
            control(line_center_x, roi)
    except KeyboardInterrupt:
        print("Finalizado")
    finally:
        stop()
        cap.release()