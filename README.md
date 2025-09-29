# unitree_brick_construction

Proyecto para experimentación con el brazo Aura (Unitree) aplicado a manipulación de ladrillos. Este documento describe cómo preparar el entorno, qué contiene cada carpeta y cómo ejecutar las rutinas disponibles.

## 1. Requisitos de entorno

El código se ha probado en Windows 11 y Ubuntu 22.04 con Python 3.10. Recomendamos aislar el entorno con Conda (o cualquier gestor equivalente como venv).

### 1.1 Instalar Conda (opcional pero recomendado)
1. Descarga Miniconda para tu sistema operativo desde la página oficial: https://docs.conda.io/en/latest/miniconda.html#latest-miniconda-installer-links
2. Sigue las instrucciones de instalación correspondientes a tu plataforma.

### 1.2 Crear y activar el entorno dktutorial
El proyecto asume un entorno llamado dktutorial con Python 3.10. Si usas otro gestor, adapta los pasos según tu herramienta.

    conda create --name dktutorial python=3.10
    conda activate dktutorial

### 1.3 Paquetes Python necesarios
Con el entorno activo, instala las dependencias base del proyecto:

    pip install "roboticstoolbox-python>=1.1.0" spatialmath numpy pandas matplotlib rich
    pip install unitree-sdk2py

Nota: unitree-sdk2py expone las clases ChannelPublisher, LowCmd_, etc., usadas para comunicarse con hardware de Unitree. Si cuentas con un SDK oficial diferente o un fork, asegúrate de que el paquete proporcione las mismas interfaces.

Para trabajar con los URDF y algunos scripts, también se requiere instalar MuJoCo o bibliotecas gráficas auxiliares si planeas simulación. Estas no se distribuyen en el repositorio y dependen de la plataforma.

## 2. Estructura del repositorio

unitree_brick_construction/
├─ configs/                 # Archivos de configuración opcionales para pruebas específicas.
├─ data/
│  ├─ raw/                  # Datos de entrada sin procesar (se ignoran en git).
│  ├─ processed/            # Datos intermedios derivados (se ignoran en git).
│  ├─ results/              # Salidas numéricas (npy/csv/json) generadas por los scripts.
│  └─ plots/                # Gráficas (png/pdf) producidas por los scripts.
├─ docs/                    # Documentación adicional o notas de proyecto.
├─ examples/                # Ejemplos de uso, celdas o pruebas de terceros.
├─ simulators/              # Recursos de simulación (MuJoCo, etc.).
├─ src/
│  └─ aura_g1/
│     ├─ controllers/       # Controladores de alto nivel (por ejemplo, rutina_ladrillos.py).
│     ├─ kinematics/
│     │  ├─ ik/            # Métodos de cinemática inversa, URDF y utilidades de pallets.
│     │  └─ ...
│     ├─ tasks/             # Módulos auxiliares reutilizables (no ejecutables directamente).
│     └─ utils/             # Helpers compartidos (paths.py, io_paths.py, etc.).
├─ tasks/                   # Scripts ejecutables listos para terminal (python tasks/...).
├─ tests/                   # Pruebas automáticas (cuando apliquen).
├─ README.md                # Este tutorial.
└─ LICENSE                  # Licencia del proyecto.

Puntos clave:
- src contiene el paquete instalable aura_g1. Todo el código reutilizable vive ahí.
- tasks agrupa los scripts listos para ejecución manual. Cada script añade src al sys.path automáticamente.
- data/results y data/plots siempre se crean al ejecutar las utilidades de IO, por eso el repositorio incluye archivos de marcador para mantener la carpeta en git.

## 3. Flujo típico de uso

1. Clonar el repositorio (si aún no lo hiciste):

       git clone https://github.com/<usuario>/unitree_brick_construction.git
       cd unitree_brick_construction

2. Activar el entorno donde instalaste las dependencias:

       conda activate dktutorial

3. Verificar que la carpeta data exista. Las utilidades la crean automáticamente, pero puedes generarla manualmente si lo deseas.

4. Ejecutar scripts de evaluación o generación de rutinas:
   - Cinemática inversa y reportes:

         python src/aura_g1/kinematics/ik/aura_test.py

     Produce q_LM_result.npy, ik_methods_comparison.png, ik_multi_scenario_summary.png e ik_multi_scenario_results.csv en data/results y data/plots.

   - Generador de rutinas de pallets:

         python tasks/poses_from_pallet.py

     Genera rutina_ladrillo_prueba.json, q_steps_pallet_LM_left.npy, q_steps_pallet_LM_left.csv, etc., bajo data/results.

   - Controlador de ejecución en hardware:

         python -m src.aura_g1.controllers.rutina_ladrillos

     Lee la rutina previa desde data/results/rutina_ladrillo_prueba.json y la reproduce utilizando unitree_sdk2py.

5. Revisar los resultados: todas las salidas quedan centralizadas en data/results y las figuras en data/plots.

## 4. Módulos clave

- src/aura_g1/kinematics/ik: resolutores de cinemática inversa (IK.py), scripts de prueba (aura_test.py), utilidades para pallets (pallet.py) y URDF específicos del robot Aura.
- src/aura_g1/tasks/mover_ladrillo.py: clases para generar secuencias de movimiento entre pallets.
- tasks/poses_from_pallet.py: ejemplo integral que arma pallets, aplica IK y guarda rutinas.
- src/aura_g1/controllers/rutina_ladrillos.py: controlador que consume la rutina generada y la ejecuta en hardware Unitree (requiere conexión con el robot o simulador compatible).
- src/aura_g1/utils/paths.py y io_paths.py: helpers para asegurar directorios, construir rutas y guardar archivos sin preocuparse por la ruta absoluta.

## 5. Buenas prácticas y notas adicionales

- Activa el entorno antes de ejecutar los scripts para evitar errores por dependencias faltantes.
- Las semillas (np.random.seed) se fijan para reproducibilidad. Si deseas explorar variaciones, cambia o elimina las semillas.
- Los scripts bajo tasks añaden automáticamente la carpeta src al sys.path; si creas nuevos scripts, sigue ese patrón para mantener la estructura limpia.
- Los archivos en data/raw y data/processed se ignoran en git. Úsalos para entradas externas o análisis intermedios.
- Para hardware real, verifica la conexión con el SDK de Unitree y ajusta los canales en rutina_ladrillos.py según tu configuración.

Con esto deberías tener todo lo necesario para navegar el proyecto, ajustar parámetros y ejecutar las rutinas de manipulación. ¡Éxitos trabajando con Aura!
