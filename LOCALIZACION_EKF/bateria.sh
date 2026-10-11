#!/usr/bin/env bash
# Bateria completa: 6 condiciones x REPETICIONES (por defecto 2), 3 vueltas
# cada una. ~5,5 min por ejecucion. Necesita el simulador con Nav2 + AMCL:
#   bash ../NAV2/lanzar.sh navegar
# Uso: bash bateria.sh            (o REPETICIONES=3 bash bateria.sh)
set -uo pipefail
cd "$(dirname "$0")"
REPETICIONES="${REPETICIONES:-2}"
AMBOS=(--escala-v 1.02 --escala-w 1.05 --sesgo-w 0.01)

for r in $(seq 1 "${REPETICIONES}"); do
  echo "===== repeticion ${r}/${REPETICIONES} ====="
  bash experimento.sh nada   --escala-v 1.0 --escala-w 1.0 --sesgo-w 0
  bash experimento.sh giro   --escala-v 1.0 --escala-w 1.05 --sesgo-w 0.01
  bash experimento.sh lineal --escala-v 1.05 --escala-w 1.0 --sesgo-w 0
  bash experimento.sh ambos  "${AMBOS[@]}"
  bash experimento.sh ruido  "${AMBOS[@]}" --ruido-v 0.02 --ruido-w 0.05 --semilla "${r}"
  IMU_SESGO_W=0.002 bash experimento.sh imu_sesgo "${AMBOS[@]}"
done
python3 evaluar.py resultados/*_{nada,giro,lineal,ambos,ruido,imu_sesgo}/
