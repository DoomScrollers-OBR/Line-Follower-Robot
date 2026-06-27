import numpy as np
import cv2 as cv
import RPi.GPIO as GPIO
import time

Window_name = "Seguidor de Linha"

def image_process():

    height, width = frame.shape[:2]     # Obtem a altura e largura da imagem
    roi = frame[height // 2:height, :]  # Utiliza a região de interesse, que é a metade inferior da imagem

    gray = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)  # Converte para escala de cinza
    _, binary = cv.threshold(
            gray,
            100,
            255,
            cv.THRESH_BINARY_INV
            )                           # Binariza a imagem, deixando em preto e branco de forma invertida (Branco vira preto e preto vira branco)

    contours, _ = cv.findContours(
            binary
            cv.RETR_EXTERNAL,
            cv.CHAIN_APPROX_SIMPLE
            )                           # Procura os contornos

                                        
    cx = None                           # Definem o centro da imagem no eixo X e Y
    cy = None

    if contours:                        # Se encontrou algum contorno 
        
        bigger = max(contours, key=cv.contourArea)   # Pega o maior contorno encontrado
         
        M = cv.moments(bigger)          # Calcula os momentos

        if M["m00"] != 0:               # Se o numero de pixels for diferente de zero

            cx = int(M["m10"] / M["m00"])           # Média aritmética das posições X dos pixels em razão do número de pixels, resultando no centro do eixo X
            cy = int(M["m01"] / M["m00"] )          # O mesmo que a variavel passada, mas em relação ao eixo y. Os dois juntos resultam no centro da linha

            cv.drawContours(
                    roi,
                    [maior],
                    -1,
                    (0,255,0),
                    2
                    )                               # Desenha os contornos
            cv.circle(
                    roi,
                    (cx, cy),
                    5,
                    (0, 0, 255),
                    -1
                    )                               # Desenha o centro da linha

            return frame, roi, binary, cx, cy, width, height


