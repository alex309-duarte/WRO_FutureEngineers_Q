#!/bin/bash

# Detener cualquier sesión de screen existente
echo "Deteniendo sesiones de screen existentes..."
pkill -f "screen /dev/ttyACM0"

# Esperar un momento
echo "Esperando 2 segundos..."
sleep 2

# Iniciar una nueva sesión de screen en segundo plano
echo "Iniciando nueva sesión de screen..."
screen -dmS spike_control /dev/ttyACM0 115200

# Esperar a que la sesión se establezca
sleep 1

# Enviar Ctrl+C para salir del intérprete si está activo
echo "Enviando comando para salir del intérprete..."
screen -S spike_control -X stuff $'\x03'
sleep 0.5

# Enviar Ctrl+D dos veces para salir del REPL si es necesario
echo "Enviando comandos para limpiar REPL..."
screen -S spike_control -X stuff $'\x04\x04'
sleep 1

# Mostrar instrucciones
echo ""
echo "=== Instrucciones ==="
echo "1. Copia el contenido de motor_control.py"
echo "2. Pega el código en la terminal"
echo "3. Presiona Enter dos veces"
echo "4. Presiona Ctrl+D dos veces para ejecutar"
echo ""

# Conectar a la sesión de screen
echo "Conectando a la sesión de screen..."
screen -r spike_control
