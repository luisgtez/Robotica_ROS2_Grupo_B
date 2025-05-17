* `person_recon.py`

Como todos los nodos, crea un logger con su parametro para poder cambiar el nivel de logging en tiempo de ejecución. El logger se inicializa con un nivel por defecto de DEBUG (20) y puede ser modificado a través del parámetro "global_log_level".

## Funcionamiento del Nodo

### Entradas
- El nodo se suscribe al topic "camera/image_raw" que recibe mensajes de tipo `sensor_msgs/Image`
- Los mensajes contienen frames de video/cámara que son procesados para detectar personas

### Procesamiento
1. Utiliza el modelo YOLOv8n para la detección de personas
2. Procesa cada frame recibido buscando personas (clase 0 en YOLO)
3. Dibuja bounding boxes verdes alrededor de las personas detectadas
4. Invierte horizontalmente el frame para visualización

### Conversión de Imágenes (CV Bridge)
El nodo utiliza CV Bridge para convertir entre formatos de imagen de ROS2 y OpenCV:
- `sensor_msgs/Image` (formato ROS2) → `numpy.ndarray` (formato OpenCV)
- La conversión se realiza con `imgmsg_to_cv2()` que:
  - Deserializa el mensaje ROS2
  - Convierte los datos a un formato que OpenCV puede procesar
  - Mantiene la codificación de color (BGR8 en este caso)
- Es necesario porque:
  - ROS2 usa su propio formato de mensaje para imágenes
  - YOLO de ultralytics necesita arrays de numpy para procesamiento (o la ruta de la imagen pero en este caso existen solo en memoria)
  - Permite la interoperabilidad entre ROS2 y bibliotecas de visión por computador

### Salidas
- Publica en el topic "person_info" mensajes de tipo `person_msg/PersonInfo` que contienen:
  - `person_exists`: booleano que indica si se detectó una persona
  - `coordinates`: punto (x,y,z) con las coordenadas de la persona detectada
    - x: posición horizontal
    - y: posición vertical
    - z: siempre 0.0 (no se usa profundidad)

### Logger
El sistema de logging permite:
- Cambiar el nivel de logging en tiempo de ejecución
- Niveles disponibles:
  - DEBUG: información detallada para debugging
  - INFO: información general del funcionamiento
  - WARN: advertencias
  - ERROR: errores críticos
- Se registran eventos importantes como:
  - Inicialización del nodo
  - Carga del modelo YOLO
  - Detección de personas
  - Errores en el procesamiento
  - Cambios en el nivel de logging 

* Tipo de dato `PersonInfo` definido en `person_msg/PersonInfo.msg`

El paquete `person_msg` es un paquete de mensajes personalizado que define el tipo de dato `PersonInfo.msg`. Este mensaje está compuesto por dos campos:
- `person_exists`: un booleano que indica si se ha detectado una persona
- `coordinates`: un punto de tipo `geometry_msgs/Point` que contiene las coordenadas (x,y,z) de la persona detectada

Este mensaje personalizado permite una comunicación estructurada entre nodos, facilitando el intercambio de información sobre la detección de personas y sus posiciones en el espacio.

