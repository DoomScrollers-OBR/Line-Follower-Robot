"""
SEGUIDOR DE LINHA SIMPLES (baseado no ROBO_DOGO_2026)

So o essencial: le a camera, acha a linha preta, calcula um PD
direto (sem clip de derivada, sem deadzone, sem filtro) e manda
velocidade pros motores via ponte H DRV8833, usando lgpio direto.

Hardware:
  - Raspberry Pi 4B (2GB RAM)
  - Ponte H DRV8833: cada motor usa 2 pinos (INx1/INx2), sem pino de
    "enable" separado. apply_motor() aplica PWM num pino e deixa o
    outro em LOW - modo "fast decay" da DRV8833.
  - Fiacao REAL confirmada:
        GPIO18 -> IN1 (motor esquerdo, forward)
        GPIO19 -> IN2 (motor esquerdo, backward)
        GPIO12 -> IN3 (motor direito, forward)
        GPIO13 -> IN4 (motor direito, backward)
  - nSLEEP da DRV8833 precisa estar em nivel alto (ligado direto em
    VM ou com pull-up no modulo) pra ponte funcionar; sem isso os
    motores nao se movem.
  - 1 camera USB, so a de seguir linha.

Mecanica de curva: velocidade de cada roda = BASE_SPEED +-
correcao do PD, recortada em [MIN_SPEED, MAX_SPEED] (esse recorte
final e limite fisico do motor, nao faz parte do calculo do PD).
Erro pequeno -> so desequilibra as duas rodas pra frente (curva
suave). Erro grande o suficiente pra correcao passar de BASE_SPEED
-> uma roda vira negativa (anda pra tras), virando quase no proprio
eixo. Isso e automatico, so o resultado de somar/subtrair a
correcao.

PD sem extras: sem clip na derivada, sem deadzone, sem filtro. So
existe uma protecao minima contra dt=0 (nao e "tuning", e so pra
nao quebrar com divisao por zero).
"""

import cv2
import numpy as np
import lgpio as GPIO
import time

# =========================================================================
# CONFIGURACAO DE PILOTAGEM (PD puro)
# =========================================================================
Kp = 1.8
Kd = 0.15
BASE_SPEED = 25       # velocidade de cruzeiro (escala -50..50)
MAX_SPEED = 50
MIN_SPEED = -MAX_SPEED
DEADZONE = 5 
THRESHOLD = 80          # limiar de binarizacao (preto vs fundo) - visao, nao PD
MIN_AREA = 11000        # area minima do contorno pra considerar "linha valida" - visao, nao PD

# =========================================================================
# PINOS - DRV8833 (fiacao real confirmada, ver docstring)
# =========================================================================
LEFT_IN1 = 18    # motor esquerdo - forward
LEFT_IN2 = 19    # motor esquerdo - backward
RIGHT_IN1 = 12   # motor direito - forward
RIGHT_IN2 = 13   # motor direito - backward

PWM_FREQ = 30  # Hz - ajuste se precisar (baixe se o motor reagir mal)

h = GPIO.gpiochip_open(0)
for pin in (LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2):
    GPIO.gpio_claim_output(h, pin)


# =========================================================================
# MOTORES - PWM direto em INx1 ou INx2, o outro fica em 0
# =========================================================================
def apply_motor(pin_in1, pin_in2, speed):
    speed = max(min(speed, MAX_SPEED), MIN_SPEED)
    duty = min(abs(speed), 100)
    if speed > 0:
        GPIO.tx_pwm(h, pin_in1, PWM_FREQ, duty)
        GPIO.tx_pwm(h, pin_in2, PWM_FREQ, 0)
    elif speed < 0:
        GPIO.tx_pwm(h, pin_in1, PWM_FREQ, 0)
        GPIO.tx_pwm(h, pin_in2, PWM_FREQ, duty)
    else:
        GPIO.tx_pwm(h, pin_in1, PWM_FREQ, 0)
        GPIO.tx_pwm(h, pin_in2, PWM_FREQ, 0)


def move(left_speed, right_speed):
    apply_motor(LEFT_IN1, LEFT_IN2, left_speed)
    apply_motor(RIGHT_IN1, RIGHT_IN2, right_speed)


def stop():
    move(0, 0)


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
# CONTROLE - PD direto: correction = Kp*error + Kd*derivative
# =========================================================================
last_error = 0
last_time = time.time()


def control(line_center_x, roi):
    global last_error, last_time

    width = roi.shape[1]
    now = time.time()
    dt = now - last_time
    if dt <= 0:
        dt = 1e-6  # so evita divisao por zero, nao e ajuste de controle

    if line_center_x is not None:
        center = width // 2
        error = (line_center_x - center) / center * 100
    else:
        error = last_error  # perdeu a linha: mantem a ultima curva

    if abs(error) < DEADZONE:
        error = 0  # deadzone: ignora erros pequenos

    derivative = (error - last_error) / dt
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
        GPIO.gpiochip_close(h)
        cap.release()