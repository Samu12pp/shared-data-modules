#!/bin/sh
# Ejecuta todas las pruebas de vuelo sobre un aircraft.json (por defecto el del 767-300ER).
# Uso: sh run_all.sh [ruta/aircraft.json]
DIR=$(cd "$(dirname "$0")" && pwd)
F=${1:-$DIR/../../767-300ER/aircraft.json}
for t in flight_tests stall_trim perf_test ground_tests rotate_test dyn_tests spiral_test ap_test; do
  echo "=================== $t"
  node "$DIR/$t.js" "$F" || exit 1
done
