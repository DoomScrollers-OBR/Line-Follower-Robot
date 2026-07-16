#!/usr/bin/env python3
"""
controller.py

Controlar via botão físico:

  - Quando a rasp liga o programa inicia sozinho
  - 1o clique no botão (com o seguidor rodando)  
	-> para o seguidor.
  - 2o clique no botão (seguidor parado):
        se tiver internet -> roda "git pull" para atualizar o código
        se não tiver  internet -> inicia o seguidor de linha de novo
"""

import os
import signal
import socket
import subprocess
from gpiozero import Button

# ----------------- CONFIGURAÇÃO (ajuste aqui) -----------------
DIR_PROJETO = "/home/serafim/Line-Follower-Robot"   # pasta do repositório
SCRIPT_SEGUIDOR = "main.py"           # arquivo do seguidor de linha
PINO_BOTAO = 17                           # GPIO
# ----------------------------------------------------------------

processo = None  # processo do seguidor de linha em execução


def tem_internet(host="8.8.8.8", porta=53, timeout=3):
    """Testa conexão com a internet"""
    try:
        socket.setdefaulttimeout(timeout)
        socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((host, porta))
        return True
    except OSError:
        return False


def seguidor_rodando():
    return processo is not None and processo.poll() is None


def iniciar_seguidor():
    global processo
    if seguidor_rodando():
        print("[controller] Seguidor já está rodando.")
        return
    print("[controller] Iniciando seguidor de linha...")
    processo = subprocess.Popen(
        ["python3", SCRIPT_SEGUIDOR],
        cwd=DIR_PROJETO,
        preexec_fn=os.setsid,  # grupo de processos próprio, facilita encerrar tudo depois
    )


def parar_seguidor():
    global processo
    if not seguidor_rodando():
        print("[controller] Seguidor já está parado.")
        return
    print("[controller] Parando seguidor de linha...")
    try:
        os.killpg(os.getpgid(processo.pid), signal.SIGTERM)
        processo.wait(timeout=5)
    except ProcessLookupError:
        pass
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(processo.pid), signal.SIGKILL)
        processo.wait()
    processo = None


def atualizar_codigo():
    print("[controller] Internet detectada. Rodando git pull...")
    try:
        resultado = subprocess.run(
            ["git", "pull"],
            cwd=DIR_PROJETO,
            capture_output=True,
            text=True,
            timeout=30,
            env={**os.environ, "GIT_TERMINAL_PROMPT": "0"},
        )
        saida = resultado.stdout.strip()
        if saida:
            print("[controller]", saida)
        if resultado.returncode != 0:
            print("[controller] Erro no git pull:", resultado.stderr.strip())
    except subprocess.TimeoutExpired:
        print("[controller] git pull demorou demais (verifique as credenciais do git).")


def ao_clicar():
    if seguidor_rodando():
        parar_seguidor()
        return

    if tem_internet():
        atualizar_codigo()
        iniciar_seguidor()
    else:
        print("[controller] Sem internet.")
        iniciar_seguidor()


def encerrar(*_args):
    print("\n[controller] Encerrando controlador...")
    parar_seguidor()
    raise SystemExit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, encerrar)
    signal.signal(signal.SIGINT, encerrar)

    botao = Button(PINO_BOTAO, pull_up=True, bounce_time=0.25)
    botao.when_pressed = ao_clicar

    iniciar_seguidor()  # início automático ao ligar
    signal.pause()
