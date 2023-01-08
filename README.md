# Smart-Vision-Glasses-for-blind (Made using Raspberry Pi and Python)
The glasses will consist of object detection, face detection/recognition and text reading features that will help the blind in carrying out the daily activities.

#Description

According to a study in 2017, out of 207.7 million people in Pakistan, an estimated 1.12 million (95% Uncertainty Interval [UI] 1.07–1.19) were blind (Visual Acuity [VA] <3/60). People who are visually impaired have difficulty in carrying out the basic everyday tasks. Because of lack of vision, they sometimes get themselves bumped into hurdles and objects. Hence, the device we propose is intended to help visually impaired people. In this project, we are developing a device that detects objects, recognizes faces and identifies image’s text and converts it to speech. It is implemented using a Raspberry Pi and a Raspberry Pi camera. The captured image goes through a series of image preprocessing steps to identify only the parts of the image that contain text, object or face and remove the background and converts a new image to speech using these tools. OCR (optical character recognition) software, Yolo algorithm and TTS (text-to-speech) engine. Audio output is through the Raspberry Pi's audio jack with speakers or headphones. 

#Manual

1. Before running the code it is essential to setup the hardware that includes raspberry pi, ultransonic sensor, camera, mouse, keyboard, and pc. 
2. Once the hardware is all set download the repository on raspberry pi software. Also download the opencv, python, and thonny dependencies. 
3. Then click on the "code" folder. It will take you to a section that includes files with different extensions. "fac_rec.py" is the file that includes all the implementation regarding raspberry pi software and camera initialization, object detection, and face recognition. 
4. Open it using a "thonny" IDE. It will build an environment where you can easily run the code. 
5. The opened file will show a "run" button at the top. Pressing it will run the code. 
6. Before the camera, the ultrasonic sensor is initialized to detect an obstacle. The camera then asks for input. 
7. Take a picture of a person or an object by pressing the button attached to raspberry. 
8.The image you take will be sent to the object and face detection algorithms for processing. 
9. First you will receive the output for object detection after 15s of taking the image. A file will be created and it will be shown at the top of the screen in the form of a tab. By clicking on this tab you can check the output for object detection. 
10. As soon as you close this tab the next algorithm for face recognition is run and you can check the output in a similar way.
