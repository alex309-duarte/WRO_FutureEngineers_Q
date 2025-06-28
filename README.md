This Repository contains all the history to build an autonomous vehicle for the WRO Future Engineers category

list of material to build your robot

Spike Prime
Raspberry Pi5 

To see the Robot 3D model, please install Studio from the following link https://www.bricklink.com/v3/studio/download.page
The model is on the path /Robot 3D model

To detect boundaries on the game field, 3 ultrasonic sensors are used to detect the walls and obstacles.

A camera is used to recognize the traffic lights on the field; the camera model is Luxonis Oak-D-Lite. This camera can load different image training models to detect several objects. The one used for this category is YOLOV8, which can be loaded into the camera, and the camera will return the images alongside the object detections.

To train the Yolov8 model, a dataset of images is needed. For that, we take several images of the game field, and later these images we need to label the objects to detect. The labeling was done using Label-Studio, which allows exporting the images in different formats for image processing, in our case Yolo. once generated the model can be trained using the following steps from Google Colab https://colab.research.google.com/github/luxonis/depthai-ml-training/blob/master/colab-notebooks/YoloV8_training.ipynb#scrollTo=91Z_AtMyvrId
Colab offers GPUs to speed up the training model; the training model could take some hours, depending on the number of images. In our case, we used 1000 images.

Once we have the training model done, from Luxonix web page, they offer an application to convert our YOLO model to a detection model recognized by the camera. Also, Luxonix offers different examples to use your model with the camera.

To control the movements of the robot, a Spike Prime hub is used. Spike Prime is a robotics kit that offers different motors and sensors. we decided to control the movements of the robot using this robotics kit. From the Spike prime controller, we also used its internal Gyroscope, which allows us to navigate in the game field alongside ultrasonic sensors.

The central computer that controls the robots logic is the Raspberry Pi 5 which, it process the data from the camera and send the data over the spike prime controller to move the robot.
