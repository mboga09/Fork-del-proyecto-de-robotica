# Proyecto Robótica — Brazo PRR para transferencia de muestras de laboratorio

Este repositorio contiene el desarrollo de un brazo robótico tipo **PRR** diseñado para la transferencia automatizada de pequeños volúmenes de líquido en un entorno de laboratorio. El proyecto forma parte del curso **MT8005 - Robótica** y busca integrar diseño mecánico, electrónica, control, cinemática, generación de trayectorias, firmware y una interfaz de usuario para operar el sistema.

## Idea general del proyecto

El objetivo principal es diseñar e implementar un manipulador robótico capaz de recoger líquido desde un tubo de muestra y depositarlo en diferentes posiciones de una placa de laboratorio. Esta tarea es repetitiva, delicada y requiere precisión, por lo que representa una aplicación adecuada para automatización robótica.

El robot propuesto utiliza una configuración de tres grados de libertad tipo **PRR**:

- Un eje **prismático** para el movimiento vertical de la herramienta.
- Dos articulaciones **rotacionales** para posicionar la herramienta dentro del plano de trabajo.
- Un actuador adicional para accionar una jeringa o mecanismo de dispensación.

La herramienta final está pensada para aspirar y dispensar volúmenes controlados de líquido, con el propósito de realizar transferencias hacia uno, varios o todos los pozos definidos en la zona de trabajo.

## Arquitectura general

El proyecto se divide en tres áreas principales:

```text
Proyecto_Robotica/
│
├── docs/
├── firmware/
└── python/
```

### `docs/`

Contiene la documentación técnica del proyecto, incluyendo el enunciado, el diseño mecánico, la cinemática del robot, diagramas y referencias necesarias para el desarrollo.

### `firmware/`

Contiene el código que será ejecutado en el microcontrolador, ya sea Arduino o ESP32. Esta sección se encarga del control directo de los actuadores:

- Motores paso a paso NEMA 17.
- Drivers para los motores.
- Servomotor del mecanismo de jeringa.
- Final de carrera para homing.
- Recepción de comandos desde Python.

El firmware no calcula la cinemática inversa. Su función principal es recibir posiciones articulares objetivo y ejecutar el movimiento de forma segura.

### `python/`

Contiene la lógica de alto nivel del robot. Desde Python se realizará:

- Cálculo de cinemática directa e inversa.
- Generación de trayectorias.
- Validación de puntos dentro del espacio de trabajo.
- Comunicación serial con el microcontrolador.
- Coordinación de tareas de transferencia.
- Interfaz de usuario para seleccionar modos de operación.

La idea principal es que Python funcione como el “cerebro” del sistema, mientras que el firmware se encargue de ejecutar los movimientos físicos.

## Estrategia de control

La arquitectura de control elegida separa el cálculo robótico de la ejecución física:

```text
Usuario
  ↓
Interfaz en Python
  ↓
Cálculo de trayectoria e IK
  ↓
Envío de objetivos articulares
  ↓
Firmware
  ↓
Motores y actuador
```

Python calcula la cinemática inversa y convierte una posición cartesiana deseada en coordenadas articulares:

```text
q = [d1, theta2, theta3]
```

Donde:

- `d1` corresponde al desplazamiento prismático vertical.
- `theta2` corresponde a la primera articulación rotacional.
- `theta3` corresponde a la segunda articulación rotacional.

Después, el firmware recibe esos valores y los convierte en pasos de motor o posiciones de actuador.

## Homing

Debido a que el robot utiliza motores paso a paso, el sistema necesita una referencia física inicial para conocer su posición real. Para esto se contempla un único final de carrera ubicado en la posición de origen del robot:

```text
(0, 0, 0)
```

Al encender el sistema, el robot debe considerarse en una posición desconocida. Antes de ejecutar movimientos o tareas automáticas, deberá realizarse una rutina de homing. Una vez alcanzado el final de carrera, el firmware establece la posición actual como el origen del sistema.

## Interfaz de usuario

El proyecto contempla una interfaz humano-máquina que permitirá seleccionar y ejecutar modos de operación. Entre los modos previstos están:

- Transferir líquido a todos los pozos.
- Transferir líquido a un único pozo seleccionado.
- Transferir líquido a un subconjunto de pozos.
- Ejecutar pruebas manuales de movimiento.
- Realizar homing.
- Controlar el actuador de la jeringa.

La interfaz será desarrollada en Python y se comunicará con el controlador principal del robot.

## Componentes principales esperados

El diseño contempla el uso de:

- 3 motores paso a paso NEMA 17.
- 3 drivers para motores paso a paso.
- 1 servomotor para el mecanismo de jeringa.
- 1 microcontrolador Arduino o ESP32.
- 1 final de carrera para la posición de home.
- Fuente de alimentación para motores y electrónica de control.
- Estructura mecánica basada en una configuración PRR.

## Estado del proyecto

Este repositorio se encuentra en etapa inicial de organización y desarrollo. La estructura está pensada para permitir que distintas partes del sistema puedan ser trabajadas de forma independiente:

- Documentación y diseño.
- Firmware de bajo nivel.
- Código Python de alto nivel.
- Interfaz de usuario.
- Pruebas de hardware.
- Validación experimental.

## Objetivo final

El objetivo final es demostrar un brazo robótico funcional capaz de realizar transferencia de volúmenes de líquido dentro de una zona de trabajo definida, integrando el modelo cinemático, el control de actuadores, la generación de trayectorias, el homing y una interfaz de usuario operativa.
