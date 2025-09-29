# unitree_brick_construction

## Uso de `aura_test.py`

Este script demuestra la evaluacion cinematica del brazo Aura empleando distintos metodos de IK.

### Requisitos previos
- Activar el entorno Conda configurado para el proyecto (`conda activate dktutorial`).
- Verificar que `src/aura_g1/kinematics/ik/IK.py` este en la misma carpeta que `aura_test.py`.
- Contar con las dependencias instaladas (roboticstoolbox, spatialmath, numpy, pandas y matplotlib).

### Ejecucion rapida
1. Abrir una terminal en la raiz del repositorio.
2. Ejecutar `python src/aura_g1/kinematics/ik/aura_test.py`.
3. Revisar la salida de consola para conocer el estado de carga del URDF, los resultados de cada solver y la tabla comparativa.

### Resultados generados
- Archivo `q_LM_result.npy` con la solucion del metodo LM (Chan).
- Graficas `ik_methods_comparison.png` e `ik_multi_scenario_summary.png` (si matplotlib esta disponible).
- Archivo `ik_multi_scenario_results.csv` con los escenarios evaluados (si pandas esta disponible).

### Notas utiles
- Los valores semilla se fijan para que las comparaciones sean reproducibles.
- Ajusta las metas en `targets` o las tolerancias de cada solver para explorar otros escenarios.
