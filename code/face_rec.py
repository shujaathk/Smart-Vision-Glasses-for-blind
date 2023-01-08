from imutils.video import VideoStream
from imutils.video import FPS
import matplotlib.pyplot as plt
from picamera import PiCamera
import RPi.GPIO as GPIO
from time import sleep
import face_recognition
import numpy as np
import imutils
import pickle
import time
import cv2
import os

#Set warnings off (optional)
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

#Set Button and LED pins
Button = 16
buzzer = 23
GPIO_TRIGGER = 18
GPIO_ECHO = 24

#Setup Button and LED
GPIO.setup(Button,GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(buzzer, GPIO.OUT)

#%matplotlib inline

def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    StartTime = time.time()
    StopTime = time.time()
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
    return distance

# get outout layers for object detection
def get_output_layers():
    
    layer_names = DONet.getLayerNames()
    try:
        output_layers = [layer_names[i - 1] for i in DONet.getUnconnectedOutLayers()]
    except:
        output_layers = [layer_names[i[0] - 1] for i in DONet.getUnconnectedOutLayers()]

    return output_layers

# draw prediction on detected objects
def draw_prediction(image, label, color, confidence, x, y, x_plus_w, y_plus_h):
    # draw rectangle with label
    cv2.rectangle(image, (x,y), (x_plus_w,y_plus_h), color, 2)
    cv2.putText(image, label, (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
def init_detect(fileClasses, fileWeights, fileConfig):
    global CLASSES
    global COLORS
    global DONet

    CLASSES = None
    
    with open(fileClasses, 'r') as f:
        CLASSES = [line.strip() for line in f.readlines()]

    COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

    DONet = cv2.dnn.readNet(fileWeights, fileConfig)
    
# detect objects in specified image
def detect_objects(image, fileOutput):
    assert not isinstance(image, type(None)), 'image not found'

    # clone of image
    image = image.copy()
    
    Width = image.shape[1]
    Height = image.shape[0]
    scale = 0.00392

    blob = cv2.dnn.blobFromImage(image, scale, (416,416), (0,0,0), True, crop=False)

    DONet.setInput(blob) 

    outs = DONet.forward(get_output_layers())

    class_ids = []
    confidences = []
    boxes = []
    conf_threshold = 0.5
    nms_threshold = 0.4

    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                center_x = int(detection[0] * Width)
                center_y = int(detection[1] * Height)
                w = int(detection[2] * Width)
                h = int(detection[3] * Height)
                x = center_x - w / 2
                y = center_y - h / 2
                class_ids.append(class_id)
                confidences.append(float(confidence))
                boxes.append([x, y, w, h])

    indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)

    # write label of detected objects in outout file
    with open(fileOutput, 'w') as outputFile:
        # for each indices draw reactangle
        for i in indices:
            try:
                box = boxes[i]
            except:
                i = i[0]
                box = boxes[i]
            
            x = box[0]
            y = box[1]
            w = box[2]
            h = box[3]
            
            # class/object id
            class_id = class_ids[i]
            # label of detected object
            label = str(CLASSES[class_id])
            # color of detected object
            color = COLORS[class_id]
            
            # write label to file
            outputFile.write(label)
            outputFile.write('\n')
            
            # draw prediction on detected object to test
            draw_prediction(image, label, color, confidences[i], round(x), round(y), round(x+w), round(y+h))

    cv2.imshow("object detection", image)
    cv2.waitKey()
        
    cv2.imwrite("dummy.jpg", image)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        init_detect('yolov3.txt', 'yolov3.weights', 'yolov3.cfg')
        camera = PiCamera()
        while True:
            button_state = GPIO.input(Button)
            dist = distance()
            if dist < 20:
                GPIO.output(buzzer, GPIO.HIGH)
                sleep(0.5)
                print ("Measured Distance = %.1f cm" % dist)
                time.sleep(1)
            else:
                print ("Measured Distance = %.1f cm" % dist)
                GPIO.output(buzzer, GPIO.LOW)
                sleep(0.5)
           
            if button_state == GPIO.LOW:
                print("BUTTON IS PRESSED")
              #  camera = PiCamera()
                camera.start_preview()
                sleep(5)
                camera.capture('/home/aswah/Desktop/FYP/he3.jpg')
                camera.stop_preview()
                sleep(1)
                currentname = "unknown"

                # grab the frame from the threaded video stream and resize it
                # to 500px (to speedup processing) << multiple of 32 like 480 is batter
                frame = cv2.imread("he3.jpg")                
                frame = imutils.resize(frame, width=500)
                
                # detect objects in captured frame
                print("[INFO] object detection...")
                detect_objects(frame, 'object_detection.txt')
                
                # determine faces from encodings.pickle file model created from train_model.py
                encodingsP = "encodings.pickle"
                # load the known faces and embeddings along with OpenCV's Haar
                # cascade for face detection
                print("[INFO] loading encodings + face detector...")
                data = pickle.loads(open(encodingsP, "rb").read())                
                
                # detect the fce boxes
                boxes = face_recognition.face_locations(frame)
                print(type(frame))
                
                # compute the facial embeddings for each face bounding box
                encodings = face_recognition.face_encodings(frame, boxes)
                names = []

                # loop over the facial embeddings
                for encoding in encodings:
                    # attempt to match each face in the input image to our known
                    # encodings
                    matches = face_recognition.compare_faces(data["encodings"],
                        encoding)
                    name = "Unknown" #if face is not recognized, then print Unknown

                    # check to see if we have found a match
                    if True in matches:
                        # find the indexes of all matched faces then initialize a
                        # dictionary to count the total number of times each face
                        # was matched
                        matchedIdxs = [i for (i, b) in enumerate(matches) if b]
                        counts = {}

                        # loop over the matched indexes and maintain a count for
                        # each recognized face face
                        for i in matchedIdxs:
                            name = data["names"][i]
                            counts[name] = counts.get(name, 0) + 1

                        # determine the recognized face with the largest number
                        # of votes (note: in the event of an unlikely tie Python
                        # will select first entry in the dictionary)
                        name = max(counts, key=counts.get)

                        #If someone in your dataset is identified, print their name on the screen
                        if currentname != name:
                            currentname = name
                            print(currentname)

                    # update the list of names
                    names.append(name)
                    print(names)
                # loop over the recognized faces
                for ((top, right, bottom, left), name) in zip(boxes, names):
                    # draw the predicted face name on the image - color is in BGR
                    cv2.rectangle(frame, (left, top), (right, bottom),
                        (0, 255, 225), 2)
                    y = top - 15 if top - 15 > 15 else top + 15
                    cv2.putText(frame, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX,
                        .8, (0, 255, 255), 2)

                # display the image to our screen
                # cv2_imshow("Facial Recognition is Running", frame)
                plt.imshow(frame)
                plt.show()
                 
            else:
                print("BUTTON IS NOT PRESSED so no image")
                sleep(1)
           
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
        
        
