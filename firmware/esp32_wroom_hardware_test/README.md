# ESP32 WROOM hardware test

Firmware independiente para validar cableado, direccionalidad y offsets antes de volver a conectar la HMI.

## Pines

| Función | Pin Arduino | GPIO |
|---|---:|---:|
| Tornillo q1 | D18 | GPIO18 |
| Brazo 1 q2 | D19 | GPIO19 |
| Brazo 2 q3 | D21 | GPIO21 |
| Servo de herramienta | D22 | GPIO22 |
| Final de carrera emergency q1 | D34 | GPIO34 |

GPIO34 es solo entrada y no tiene pull-up/pull-down interno. El switch de emergencia debe llevar resistencia externa.

## Seguridad del test

- El comando `ARM_TEST` arma el firmware sin mover el eje Z.
- La secuencia Python usa `z_dir = 0` y `z_time_s = 0` por defecto.
- Los jogs de q1 solo se ejecutan si se pasa `--jog-z`.
- El firmware rechaza cualquier movimiento de q1 mayor a `2.0 s`.
- q2 y q3 se mueven lento con pasos de `0.5 deg` cada `25 ms`.
- La herramienta es un servo 180 deg en D22.

## Convención de herramienta

Como el servo solo acepta comandos absolutos de `0 deg` a `180 deg`, usamos esta convención para representar los sentidos mecanicos:

| Acción | Servo absoluto | Sentido mecánico |
|---|---:|---:|
| `TOOL_HOME` | 90 deg | centro |
| `TOOL_ASPIRATE` | 180 deg | +180, succionar |
| `TOOL_DISPENSE` | 0 deg | -180, vaciar |

Tambien existe `TOOL_MOVE` para probar un angulo arbitrario:

```json
{"cmd":"TOOL_MOVE","name":"TOOL_120","tool_deg":120}
```

## Cores

- Core 0: `SerialTask`, lectura serial, parser JSON y logging.
- Core 1: `MotionTask`, PWM, estado del controlador y emergency stop.

## Comandos principales

```json
{"cmd":"PING"}
{"cmd":"ARM_TEST"}
{"cmd":"MOVE_ACT","name":"SAFE","z_dir":0,"z_time_s":0,"s2_deg":45,"s3_deg":90}
{"cmd":"CONFIG","q2_trim_deg":0,"q3_trim_deg":0,"tool_trim_deg":0,"q1_stop_us":1500,"q1_forward_us":1700,"q1_reverse_us":1300}
{"cmd":"TOOL_HOME"}
{"cmd":"TOOL_ASPIRATE"}
{"cmd":"TOOL_DISPENSE"}
{"cmd":"STOP"}
{"cmd":"ESTOP"}
```

## Uso desde Python

Desde la carpeta `python` del proyecto:

```powershell
python -m tools.firmware_hardware_test --port COM8
```

Para imprimir los JSON sin enviar nada:

```powershell
python -m tools.firmware_hardware_test --port COM8 --dry-run
```

Para probar q1 con dos jogs cortos de 0.25 s:

```powershell
python -m tools.firmware_hardware_test --port COM8 --jog-z
```

Para incluir prueba de herramienta:

```powershell
python -m tools.firmware_hardware_test --port COM8 --include-tool
```
