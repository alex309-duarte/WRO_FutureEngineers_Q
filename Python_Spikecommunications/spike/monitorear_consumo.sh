#!/bin/bash

# Archivo para guardar los resultados
LOG_FILE="consumo_energia_$(date +%Y%m%d_%H%M%S).log"

echo "Iniciando monitoreo de consumo de energía..."
echo "Los resultados se guardarán en: $LOG_FILE"
echo "Presiona Ctrl+C para detener el monitoreo"

# Función para limpiar al salir
cleanup() {
    echo "\nDeteniendo monitoreo..."
    pkill -P $$
    exit 0
}

# Capturar Ctrl+C
trap cleanup SIGINT

# Iniciar el monitoreo de energía en segundo plano
sudo powertop --time=1 --csv=/tmp/powertop.csv >/dev/null 2>&1 &
POWERTOP_PID=$!

# Iniciar el programa de la cámara en segundo plano
echo "Iniciando run_oak_model.py..."
python3 /home/maker/spike/run_oak_model.py &
CAMERA_PID=$!

echo "Monitoreando consumo de energía (actualizando cada 2 segundos)..."
echo "Hora,Consumo (W),Temperatura (C),Uso CPU (%)" > "$LOG_FILE"

# Bucle de monitoreo
while true; do
    # Obtener consumo de energía (usando vcgencmd)
    POWER_WATTS=$(vcgencmd measure_volts | cut -d= -f2 | tr -d 'V')
    CURRENT_MA=$(vcgencmd measure_current | cut -d= -f2 | tr -d 'mA')
    POWER_MW=$(echo "$POWER_WATTS * $CURRENT_MA" | bc -l)
    POWER_W=$(echo "scale=2; $POWER_MW / 1000" | bc -l)
    
    # Obtener temperatura
    TEMP=$(vcgencmd measure_temp | cut -d= -f2 | tr -d "'C")
    
    # Obtener uso de CPU
    CPU_USAGE=$(top -bn1 | grep "Cpu(s)" | awk '{print $2 + $4}')
    
    # Obtener hora actual
    TIMESTAMP=$(date +"%Y-%m-%d %H:%M:%S")
    
    # Mostrar y guardar información
    echo "[$TIMESTAMP] Consumo: ${POWER_W}W | Temp: ${TEMP}°C | CPU: ${CPU_USAGE}%"
    echo "$TIMESTAMP,$POWER_W,$TEMP,$CPU_USAGE" >> "$LOG_FILE"
    
    # Esperar 2 segundos
    sleep 2
done
