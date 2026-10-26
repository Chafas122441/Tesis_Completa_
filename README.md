# Robot DELTA — archivos de anexos

Este paquete contiene los diseños mecánicos seleccionados y el código fuente de los tres microcontroladores. Conserva las consignas, calibraciones, límites y comportamiento de la versión de trabajo. Los comentarios describen la función del código. No incluye historiales de desarrollo, registros de pruebas ni productos de compilación.

## Contenido

| Carpeta | Contenido |
| --- | --- |
| `INVENTOR_TESIS` | Copia completa del proyecto mecánico seleccionado, incluidos sus archivos auxiliares y versiones CAD anteriores. |
| `Codigo/10A_Gamepad_Camara_WiFi_ESP` | ESP32 10A, firmware 2.15. Mando Bluetooth y cámara HUSKYLENS 2. |
| `Codigo/10B_Control_Manual_Auto_Portenta` | Portenta H7 M7 10B, firmware 3.73. Coordinación, visión, cinemática y secuencia automática. |
| `Codigo/10C_Actuador_Seguro_Nucleo` | Nucleo F446RE 10C, firmware 2.51. Actuación, encoder y pala. |
| `Codigo/libraries/DFRobot_HuskylensV2` | Dependencia de cámara versión 1.0.9, con su licencia. |

## Diseños mecánicos

Abrir `INVENTOR_TESIS/Tesis_2025.ipj` como proyecto de Inventor. Se conservan las rutas relativas y la carpeta `Centro_de_contenido`. Este paquete incluye únicamente `INVENTOR_TESIS`, no el conjunto independiente del robot base guardado en `2025/INVENTOR/DELTA`.

La integridad de la copia se comprueba archivo por archivo. Esto no equivale a haber abierto y resuelto todas las referencias de los ensambles en Inventor. Los archivos de fabricación deben revisarse para la máquina y el material correspondientes antes de utilizarlos.

## Compilar los firmwares

### ESP32 — 10A

Abrir el `.ino` de la carpeta 10A. Utilizar el paquete **ESP32 + Bluepad32 4.1.0**, placa `esp32-bluepad32:esp32:esp32`. El paquete ESP32 común no sustituye la integración de Bluepad32 utilizada aquí. Instalar la biblioteca incluida `DFRobot_HuskylensV2` en la carpeta de bibliotecas de Arduino, o indicar `Codigo/libraries` mediante la opción `--libraries` de Arduino CLI.

### Portenta H7 — 10B

Abrir el `.ino` de la carpeta 10B. Utilizar **Arduino Mbed OS Portenta Boards 4.6.0**, destino Portenta H7 M7, identificador `arduino:mbed_portenta:envie_m7`. Mantener todos los encabezados de la carpeta junto al sketch.

### Nucleo F446RE — 10C

En STM32CubeIDE 1.18, importar la carpeta 10C como proyecto existente. Se incluyen `.project`, `.cproject`, `.ioc`, archivos de enlace, `Core` y `Drivers`. Compilar la configuración Debug con GNU Tools for STM32 13.3.rel1. No basta con importar únicamente `main.c`. Los controladores ST y ARM mantienen sus licencias y comentarios originales.

## Conexión y operación

Las copias de los tres firmwares se compilaron correctamente con las versiones de herramientas indicadas. ESP32 y Portenta se verificaron con Arduino CLI. Nucleo se compiló y enlazó con el compilador GNU 13.3.rel1 instalado con CubeIDE, sin abrir el IDE. La Nucleo conserva avisos por funciones de inicialización no utilizadas. Los binarios de esta comprobación no se incluyen en el paquete.

ESP32 y Portenta deben compartir la configuración del punto de acceso y los formatos de paquetes. La clave Wi-Fi incluida corresponde al montaje de laboratorio y debe cambiarse en ambos firmwares antes de utilizar una red compartida o pública. Los encabezados del protocolo y de calibración duplicados entre placas deben mantenerse sincronizados.

Se conservan los mensajes de diagnóstico por puerto serie porque forman parte del funcionamiento del firmware. No son anotaciones de desarrollo. Una confirmación de recepción no demuestra por sí sola que un actuador haya alcanzado su posición.

Compilar y cargar con la potencia de los motores desconectada. Mantener disponible el corte físico de alimentación y comprobar conexiones, calibraciones y espacio de movimiento antes de energizar. Esta preparación de archivos no carga placas ni valida mecánicamente el robot.
