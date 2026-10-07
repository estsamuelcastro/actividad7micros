# Proyecto ACO Híbrido - 1 ESP32 físico + 2 ESP32 virtuales + PyBullet

## Resultado final

El proyecto implementa una colonia distribuida de tres nodos ACO:

- **NODE 1:** ESP32 DevKit V1 físico. Crea el Access Point Wi-Fi `ACO_ESP32`, ejecuta ACO y retransmite mensajes UDP entre los otros nodos.
- **NODE 2:** ESP32 virtual en Python.
- **NODE 3:** ESP32 virtual en Python.
- **PyBullet:** representa los tres nodos como robots dentro de un laberinto y mueve cada robot según la mejor ruta ACO recibida.

Los dos ESP32 virtuales son una emulación de comportamiento del nodo, no una emulación electrónica del microcontrolador completo. Esta adaptación permite cumplir la arquitectura de tres nodos usando el hardware disponible.

## Estructura

```text
ACO_ESP32_FINAL/
├── esp32_fisico/
│   └── nodo1_aco.ino
├── virtuales/
│   └── virtual_node.py
├── simulacion/
│   └── pybullet_sim.py
├── requirements.txt
├── Dockerfile
├── docker-compose.yml
├── run_final.bat
├── run_docker.bat
└── README.md
```

## Arranque final recomendado

### 1. ESP32 físico

Abra `esp32_fisico/nodo1_aco.ino` en Arduino IDE, seleccione la placa **ESP32 Dev Module** (o la configuración equivalente a su DevKit V1), seleccione el puerto COM del ESP32 y cargue el sketch.

Abra el Monitor Serie a 115200.

El ESP32 crea:

```text
SSID: ACO_ESP32
PASSWORD: aco12345
IP: 192.168.4.1
UDP: 5000
```

### 2. Windows

Conecte Windows a la red Wi-Fi:

```text
ACO_ESP32
```

Contraseña:

```text
aco12345
```

### 3. Ejecución final directa en Windows

Desde esta carpeta ejecute:

```cmd
run_final.bat
```

Esto abre NODE 2 y NODE 3 y luego inicia PyBullet.

### 4. Ejecución con Docker

Dockeriza los dos nodos virtuales:

```cmd
docker compose up --build
```

Después ejecute PyBullet desde Windows:

```cmd
python simulacion\pybullet_sim.py
```

En Docker Desktop, el modo de red de los contenedores depende de la configuración de networking del equipo. El código permite usar la IP `192.168.4.1` del SoftAP del ESP32 y mandar la visualización al host con `host.docker.internal`.

## Protocolo UDP

```text
H|2
H|3
```
Registro de nodos virtuales.

```text
S|2|READY
S|3|READY
```
Confirmación del ESP32 físico.

```text
P|2|0|1|1.8234
```
Feromona de una arista.

```text
R|2|4.000|0,1,4,5,8
```
Mejor ruta de un nodo.

El NODE 1 físico recibe `P` y `R`, actualiza su colonia y retransmite el mensaje al otro nodo virtual. Los nodos virtuales también envían sus mejores rutas al puerto local 6000 para que PyBullet pueda visualizar el comportamiento.

## ACO

Parámetros:

```text
alpha = 1.0
beta  = 2.0
rho   = 0.15
Q     = 1.0
hormigas por iteración = 18
```

Grafo:

```text
0-1
0-3
1-4
2-5
3-6
4-5
4-7
5-8
6-7
7-8
```

Inicio: `0`

Meta: `8`

Dos rutas óptimas de costo 4 aparecen de forma natural en el grafo, por ejemplo:

```text
0 -> 1 -> 4 -> 5 -> 8
```

```text
0 -> 3 -> 6 -> 7 -> 8
```

La visualización no calcula las rutas. PyBullet únicamente representa las decisiones que los nodos ACO están produciendo.

## Qué mostrar en la sustentación

1. El Monitor Serie del ESP32 mostrando el Access Point y las iteraciones ACO.
2. Las dos terminales de los nodos virtuales mostrando rutas y feromonas recibidas.
3. La ventana de PyBullet mostrando tres robots etiquetados como NODE 1, NODE 2 y NODE 3.
4. El flujo: ACO local -> intercambio UDP -> actualización de feromonas -> mejor ruta -> movimiento en PyBullet.

## Texto breve para explicar el proyecto

“Implementamos una colonia distribuida de hormigas compuesta por un ESP32 físico y dos nodos virtuales. Cada nodo ejecuta ACO de manera independiente, calcula rutas hacia una meta y actualiza su matriz de feromonas. La información de feromonas y las mejores rutas se intercambia mediante UDP sobre la red Wi-Fi creada por el ESP32 físico. Finalmente, PyBullet funciona como gemelo digital de la colonia y representa los tres nodos como robots que recorren el laberinto siguiendo las rutas obtenidas. Debido a la disponibilidad de hardware, se utilizó un ESP32 físico y dos nodos virtuales, manteniendo la lógica distribuida exigida por el proyecto.”
