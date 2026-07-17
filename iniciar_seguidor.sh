#!/bin/bash

DIR_PROJETO="/home/serafim/Documentos/Line-Follower-Robot"   # pasta do projeto
USAR_VENV=false                          # true se é  ambiente virtual
VENV_PATH="$DIR_PROJETO/venv"           # ambiente virtual

# -------------------------------------------------

cd "$DIR_PROJETO" || { echo "Pasta do projeto não encontrada: $DIR_PROJETO"; exit 1; }

if [ "$USAR_VENV" = true ]; then
    # shellcheck disable=SC1091
    source "$VENV_PATH/bin/activate"
fi

exec python3 controller.py
