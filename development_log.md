# Development Log: Speed Monitor Node

## Overview
Este nodo forma parte de un sistema de detección de peatones y control de velocidad para vehículos autónomos. El sistema completo consta de tres nodos:
1. Nodo de cámara (captura de video)
2. Nodo de reconocimiento de personas (YOLO)
3. Nodo de monitor de velocidad (este nodo)

## Funcionalidad del Nodo
El nodo `speed_monitor` simula el control de velocidad de un vehículo basado en la detección de personas:
- Velocidad inicial: 100 km/h
- Desaceleración rápida cuando se detecta una persona (-20 km/h por segundo)
- Aceleración gradual cuando no hay personas (5 km/h por segundo)
- Visualización en tiempo real de la velocidad

## Implementación

### Suscripción a Mensajes
```python
self.subscription = self.create_subscription(
    PersonInfo,
    'person_detection',
    self.person_callback,
    10
)
```
El nodo se suscribe al topic 'person_detection' para recibir mensajes de tipo `PersonInfo`, que contienen:
- `person_exists`: booleano que indica si se detectó una persona
- `coordinates`: posición de la persona (no utilizado en este nodo)

### Control de Velocidad
```python
def person_callback(self, msg):
    if msg.person_exists:
        self.target_speed = 0.0  # Parada de emergencia
    else:
        self.target_speed = 100.0  # Velocidad normal
```
- Cuando se detecta una persona, se inicia una parada de emergencia
- Cuando no hay personas, se establece la velocidad objetivo a 100 km/h

### Visualización
Se utiliza matplotlib para crear una gráfica en tiempo real:
- Eje X: tiempo (ventana móvil de 10 segundos)
- Eje Y: velocidad (0-120 km/h)
- Actualización cada 50ms
- La línea azul muestra la velocidad actual

## Integración en el Proyecto
1. Archivo `speed_monitor.py` creado en el directorio `practica_B/practica_B/`
2. Actualizado `setup.py` para incluir:
   - Nuevas dependencias (matplotlib, numpy)
   - Entry point para el nodo

## Uso del Nodo
Para ejecutar el nodo (requiere ROS 2):
```bash
# Construir el paquete
colcon build
source install/setup.bash

# Ejecutar el nodo
ros2 run practica_B speed_monitor
```

## Parámetros Configurables
- `self.current_speed`: Velocidad inicial (100 km/h)
- `self.acceleration`: Tasa de aceleración (5 km/h/s)
- `self.deceleration`: Tasa de frenado (-20 km/h/s)
- `self.time_window`: Ventana de tiempo en la gráfica (10s)

## Mejoras Potenciales
1. Parametrización de velocidades y tasas de aceleración
2. Añadir diferentes modos de conducción
3. Implementar un sistema de logging más detallado
4. Añadir tests unitarios
5. Mejorar la visualización con más información (distancia al peatón, etc.) 