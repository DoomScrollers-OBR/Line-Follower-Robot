"""
SEGUIDOR DE LINHA SIMPLES (baseado no ROBO_DOGO_2026)

So o essencial: le a camera, acha a linha preta, calcula um PID e
manda velocidade pros motores via ponte H DRV8833.

Hardware:
  - Raspberry Pi 4B (2GB RAM)
  - Ponte H DRV8833: cada motor usa 2 pinos (INx1/INx2), sem pino de
    "enable" separado. Cada pino e um PWMOutputDevice (gpiozero); a
    logica em _set_motor() aplica PWM num pino e deixa o outro em
    LOW - exatamente o modo "fast decay" da DRV8833. Trocamos o
    Motor (que nao deixa configurar frequencia) por PWMOutputDevice
    direto justamente pra poder ajustar PWM_FREQUENCY abaixo.
  - nSLEEP da DRV8833 precisa estar em nivel alto (ligado direto em
    VM ou com pull-up no modulo) pra ponte funcionar; sem isso os
    motores nao se movem.
  - 1 camera USB, so a de seguir linha.
"""

import cv2
import numpy as np
import time
from gpiozero import PWMOutputDevice

# =========================================================================
# CONFIGURACAO DE PILOTAGEM
# =========================================================================
Kp = 1.8
Kd = 0.1
BASE_SPEED = 25       # velocidade de cruzeiro (escala -50..50)
MAX_SPEED = 50
MIN_SPEED = -MAX_SPEED
DEADZONE = 5            # erro abaixo disso e tratado como "reto"
THRESHOLD = 80          # limiar de binarizacao (preto vs fundo)
MIN_AREA = 11000        # area minima do contorno pra considerar "linha valida"

# Filtro passa-baixa (media movel exponencial) aplicado na derivada -
# suaviza picos de ruido sem perder resposta a mudancas reais.
DERIVATIVE_FILTER = 0.2

# =========================================================================
# MOTORES - DRV8833 via PWMOutputDevice (gpiozero)
# Ajuste os pinos conforme sua fiacao real com a ponte H.
# =========================================================================
PWM_FREQUENCY = 200  # Hz - baixe mais (ex: 100, 50) se ainda tiver problema

left_forward = PWMOutputDevice(17, frequency=PWM_FREQUENCY)
left_backward = PWMOutputDevice(18, frequency=PWM_FREQUENCY)
right_forward = PWMOutputDevice(12, frequency=PWM_FREQUENCY)
right_backward = PWMOutputDevice(13, frequency=PWM_FREQUENCY)


def _set_motor(forward_pin, backward_pin, value):
    # value entre -1 (re, velocidade maxima) e 1 (frente, velocidade
    # maxima). PWM num pino, o outro em LOW.
    if value > 0:
        forward_pin.value = value
        backward_pin.value = 0
    elif value < 0:
        forward_pin.value = 0
        backward_pin.value = -value
    else:
        forward_pin.value = 0
        backward_pin.value = 0


def move(left_speed, right_speed):
    # left_speed/right_speed ja chegam recortados em [MIN_SPEED, MAX_SPEED]
    # vindos de control()
    _set_motor(left_forward, left_backward, left_speed / MAX_SPEED)
    _set_motor(right_forward, right_backward, right_speed / MAX_SPEED)


def stop():
    _set_motor(left_forward, left_backward, 0)
    _set_motor(right_forward, right_backward, 0)


# =========================================================================
# CAMERA
# =========================================================================
CAMERA_INDEX = 0   # indice do dispositivo de video (/dev/video0)
CAMERA_WIDTH, CAMERA_HEIGHT = 160, 120


def start_camera():
    attempts = 0
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    while not cap.isOpened():
        attempts += 1
        print(f"[Camera] Tentando abrir... (tentativa {attempts})")
        time.sleep(1)
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
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
# CONTROLE (PID com derivada filtrada)
# =========================================================================
last_error = 0
last_time = time.time()
filtered_derivative = 0.0


def control(line_center_x, roi):
    global last_error, last_time, filtered_derivative

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

    raw_derivative = np.clip((error - last_error) / dt, -300, 300)
    filtered_derivative = (DERIVATIVE_FILTER * raw_derivative
                            + (1 - DERIVATIVE_FILTER) * filtered_derivative)

    correction = Kp * error + Kd * filtered_derivative

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