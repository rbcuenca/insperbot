#!/bin/bash

GLOBAL_ENV_FILE="/tmp/insperbot_processes.env"

if [ ! -f "$GLOBAL_ENV_FILE" ]; then
    echo "Nenhum processo registrado ainda."
    exit 1
fi

source "$GLOBAL_ENV_FILE"

if [ -z "$PROCESSES_UP" ]; then
    echo "Nenhum processo registrado ainda."
    exit 1
fi

echo "Processos registrados como iniciados:"
for proc in $PROCESSES_UP; do
    echo "- $proc"
done
