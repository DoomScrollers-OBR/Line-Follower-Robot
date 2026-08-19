
"""
TESTE ISOLADO - motor direito
 
Roda so o motor direito, sem camera e sem PID, pra descobrir se o
problema e de fiacao/pino ou de logica no codigo principal.
"""
 
from gpiozero import Motor
from time import sleep
 
# mesmos pinos do codigo principal - troque aqui se for testar outros
right_motor = Motor(forward=12, backward=13)
 
print("Frente por 2s...")
right_motor.forward(0.5)   # 50% de velocidade
sleep(2)
right_motor.stop()
 
print("Re por 2s...")
right_motor.backward(0.5)
sleep(2)
right_motor.stop()
 
print("Fim do teste")
