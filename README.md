This Repository contains all the history to build an autonomous vehicle for the WRO Future Engineers category

list of material to build your robot

Spike Prime hub
Raspberry Pi5 
Raspberry Pi AI HAT+
Cámara Módulo 3 Raspberry Pi - 12 Mp Gran Angular
RPLidar A1
3 TOF sensor model VL53L1X

To see the Robot 3D model, please install Studio from the following link https://www.bricklink.com/v3/studio/download.page
The model is on the path /Robot 3D model

The Lidar is used to detect boundaries on the game field.

A camera is used to recognize the traffic lights on the field. The camera and the AI hat from raspberry Pi are combined to process the images and detect the traffic lights. The object detection model used is yolov8n

To train the Yolov8 model, a dataset of images is needed. For that, we take several images of the game field, and later we label the images with the objects to detect. The labeling was done using Label-Studio, which allows exporting the images in different formats for image processing, in our case Yolo8 format. Once generated, the model can be trained using the following steps from Google Colab https://colab.research.google.com/github/luxonis/depthai-ml-training/blob/master/colab-notebooks/YoloV8_training.ipynb#scrollTo=91Z_AtMyvrId
Colab offers GPUs to speed up the training model; the training model could take some hours, depending on the number of images. In our case, we used 1000 images.

Once we have the training model done, we need to run the model on the raspberry Pi 5 operating system. The IA hat from raspberry Pi uses the HAILO accelerator to speed up the image detection processing using different object detection models like YOLO. To run yolo custom model on the HAILO accelerator, we need to convert our ONNX model to HEF format. The steps to do that can be found in the following link: https://www.cytron.io/tutorial/raspberry-pi-ai-kit-onnx-to-hef-conversion?srsltid=AfmBOoo8RtAYAU1g-JS7Basru8lXYtfCQZbNn-Md9rvm77fqVix72JHy

To control the robot's movements, a Spike Prime hub is used. Spike Prime is a robotics kit that offers a variety of motors and sensors that can be controlled by a main HUB. We decided to control the robot’s movements using this robotics kit. To connect the raspberry PI to the SPIKE HUB, a serial communication is done. First, the SPIKE prime hub is configured in interpreter mode. Later, the raspberry Pi can send the command over the USB connection to control the motor and perform the navigation on the game field.

The central computer that controls the robot's logic is the Raspberry Pi 5, which gathers the data from the camera, Lidar points, and SPIKE Prime hub.

--------------------------------------

Este repositorio contiene toda la documentación histórica para construir un vehículo autónomo para la categoría Future Engineers de la WRO.

Lista de materiales para construir tu robot
Spike Prime

Raspberry Pi 5

Visualización del modelo 3D
Para ver el modelo 3D del robot:

Instala Studio desde:
https://www.bricklink.com/v3/studio/download.page

El modelo está en la ruta:
/Robot 3D model

Sistema de detección de límites
Se utilizan 3 sensores ultrasónicos para detectar paredes y obstáculos en el campo de juego.

Reconocimiento de semáforos
Cámara: Luxonis OAK-D-Lite

Modelo de IA: YOLOV8

La cámara carga modelos de entrenamiento para detección de objetos

Devuelve imágenes con las detecciones identificadas

Entrenamiento del modelo YOLOV8
Dataset:

Captura de imágenes del campo de juego

Etiquetado:

Herramienta: Label-Studio

Formato de exportación: Yolo

Entrenamiento:

Plataforma: Google Colab

GPU acelerada

Enlace:
https://colab.research.google.com/github/luxonis/depthai-ml-training/blob/master/colab-notebooks/YoloV8_training.ipynb

Tiempo estimado: Horas (para 1,000 imágenes)

Conversión del modelo
Utilizar la herramienta de Luxonis para convertir modelos YOLO a formato compatible con la cámara.

Ejemplos disponibles en la documentación de Luxonis.

Control de movimiento
Controlador principal: Spike Prime Hub

Kit robótico con motores y sensores

Funciones clave:

Control de motores

Giroscopio interno para navegación

Integración con sensores ultrasónicos

Unidad central de procesamiento
Raspberry Pi 5

Procesa datos de la cámara

Ejecuta la lógica principal del robot

Comunica instrucciones al Spike Prime via:

USB

Protocolo serie personalizado


