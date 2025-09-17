# Introduction
This Repository contains all the history to build an autonomous vehicle for the WRO Future Engineers category

# list of material to build your robot

  - Spike Prime hub

  - Raspberry Pi5

  - Raspberry Pi AI HAT+

  - Cámara Módulo 3 Raspberry Pi - 12 Mp Gran Angular

  - RPLidar A1

  - 3 TOF sensor model VL53L1X


> **Note:** To see the Robot 3D model, please install Studio from the following link https://www.bricklink.com/v3/studio/download.page
The model is on the path /Robot 3D model

# Summary

The Lidar is used to detect boundaries on the game field.

A camera is used to recognize the traffic lights on the field. The camera and the AI hat from raspberry Pi are combined to process the images and detect the traffic lights. The object detection model used is yolov8n

To train the Yolov8 model, a dataset of images is needed. For that, we take several images of the game field, and later we label the images with the objects to detect. The labeling was done using Label-Studio, which allows exporting the images in different formats for image processing, in our case Yolo8 format. Once generated, the model can be trained using the following steps from Google Colab https://colab.research.google.com/github/luxonis/depthai-ml-training/blob/master/colab-notebooks/YoloV8_training.ipynb#scrollTo=91Z_AtMyvrId
Colab offers GPUs to speed up the training model; the training model could take some hours, depending on the number of images. In our case, we used 1000 images.

Once we have the training model done, we need to run the model on the raspberry Pi 5 operating system. The IA hat from raspberry Pi uses the HAILO accelerator to speed up the image detection processing using different object detection models like YOLO. To run yolo custom model on the HAILO accelerator, we need to convert our ONNX model to HEF format. The steps to do that can be found in the following link: https://www.cytron.io/tutorial/raspberry-pi-ai-kit-onnx-to-hef-conversion?srsltid=AfmBOoo8RtAYAU1g-JS7Basru8lXYtfCQZbNn-Md9rvm77fqVix72JHy

To control the robot's movements, a Spike Prime hub is used. Spike Prime is a robotics kit that offers a variety of motors and sensors that can be controlled by a main HUB. We decided to control the robot’s movements using this robotics kit. To connect the raspberry PI to the SPIKE HUB, a serial communication is done. First, the SPIKE prime hub is configured in interpreter mode. Later, the raspberry Pi can send the command over the USB connection to control the motor and perform the navigation on the game field.

The central computer that controls the robot's logic is the Raspberry Pi 5, which gathers the data from the camera, Lidar points, and SPIKE Prime hub.

--------------------------------------

# Introducción
Este repositorio contiene todo el historial para construir un vehículo autónomo para la categoría **WRO Future Engineers**.

# Lista de materiales para construir tu robot

  - Spike Prime hub

  - Raspberry Pi 5

  - Raspberry Pi AI HAT+

  - Cámara Módulo 3 Raspberry Pi - 12 Mp Gran Angular

  - RPLidar A1

  - 3 sensores TOF modelo VL53L1X


> **Nota:** Para ver el modelo 3D del robot, instala Studio desde el siguiente enlace https://www.bricklink.com/v3/studio/download.page  
El modelo se encuentra en la ruta `/Robot 3D model`

# Resumen

El Lidar se utiliza para detectar los límites en el campo de juego.  

Una cámara se utiliza para reconocer los semáforos en el campo. La cámara y el AI HAT de Raspberry Pi se combinan para procesar las imágenes y detectar los semáforos. El modelo de detección de objetos usado es **yolov8n**.  

Para entrenar el modelo Yolov8, se necesita un conjunto de datos de imágenes. Para ello, tomamos varias imágenes del campo de juego y, posteriormente, etiquetamos las imágenes con los objetos a detectar. El etiquetado se realizó usando **Label-Studio**, que permite exportar las imágenes en diferentes formatos para el procesamiento de imágenes; en nuestro caso, en formato **Yolo8**. Una vez generado, el modelo puede entrenarse usando los siguientes pasos en Google Colab:  
https://colab.research.google.com/github/luxonis/depthai-ml-training/blob/master/colab-notebooks/YoloV8_training.ipynb#scrollTo=91Z_AtMyvrId  

Colab ofrece GPUs para acelerar el entrenamiento del modelo; este proceso puede tardar varias horas dependiendo de la cantidad de imágenes. En nuestro caso, usamos **1000 imágenes**.  

Una vez finalizado el entrenamiento, necesitamos ejecutar el modelo en el sistema operativo de la Raspberry Pi 5. El AI HAT de Raspberry Pi utiliza el **acelerador HAILO** para optimizar el procesamiento de detección de imágenes con diferentes modelos de detección de objetos como YOLO. Para ejecutar un modelo YOLO personalizado en el acelerador HAILO, es necesario convertir nuestro modelo **ONNX** al formato **HEF**. Los pasos para hacerlo se encuentran en el siguiente enlace:  
https://www.cytron.io/tutorial/raspberry-pi-ai-kit-onnx-to-hef-conversion?srsltid=AfmBOoo8RtAYAU1g-JS7Basru8lXYtfCQZbNn-Md9rvm77fqVix72JHy  

Para controlar los movimientos del robot, se utiliza un **Spike Prime hub**. Spike Prime es un kit de robótica que ofrece una variedad de motores y sensores que pueden ser controlados por un HUB principal. Decidimos controlar los movimientos del robot usando este kit de robótica. Para conectar la Raspberry Pi al SPIKE HUB, se realiza una **comunicación serial**. Primero, el Spike Prime hub se configura en **modo intérprete**. Posteriormente, la Raspberry Pi puede enviar comandos a través de la conexión USB para controlar los motores y realizar la navegación en el campo de juego.  

La computadora central que controla la lógica del robot es la **Raspberry Pi 5**, la cual recopila los datos de la cámara, los puntos del Lidar y el Spike Prime hub.
