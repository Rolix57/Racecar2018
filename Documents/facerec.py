# -*- coding: utf-8 -*-
"""
Created on Fri Jul 27 12:01:36 2018

@author: arath
"""

import cv2

import face_recognition

face_csc = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
cam = cv2.VideoCapture(1)

while(cam):
    
    img = cam.read()       
       
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    faces = face_csc.detectMultiScale(gray, 1.1, 4)
    
    for (x, y, w, h) in faces:
        cv2.rectangle(gray, (x,y), (x+w, y+h), (0,0,0), 5)
        
    cv2.imshow('img', gray)
    key = cv2.waitKey(1)
    if key == 27:
        break
    
        # Load the jpg files into numpy arrays
    ahmad = face_recognition.load_image_file("ahmad.png")
    belyi = face_recognition.load_image_file("belyi.png")
    fishberg = face_recognition.load_image_file("fishberg.png")
    unknown_image = face_recognition.load_image_file("unknown.jpg")
    current_image = face_recognition.load_image_file("img")
    
    # Get the face encodings for each face in each image file
    # Since there could be more than one face in each image, it returns a list of encodings.
    # But since I know each image only has one face, I only care about the first encoding in each image, so I grab index 0.
    
    ahmad_encoding = face_recognition.face_encodings(ahmad)[0]
    belyi_encoding = face_recognition.face_encodings(belyi)[0]
    fishberg_encoding = face_recognition.face_encodings(fishberg)[0]
    unknown_encoding = face_recognition.face_encodings(unknown_image)[0]
    current_encoding = face_recognition.face_encodings(current_image)[0]
    
    known_faces = [
        ahmad_encoding,
        belyi_encoding,
        fishberg_encoding
    ]
    
    # results is an array of True/False telling if the unknown face matched anyone in the known_faces array
    results = face_recognition.compare_faces(known_faces, current_encoding)
    
    print("Is the unknown face a picture of ahmad? {}".format(results[0]))
    print("Is the unknown face a picture of belyi? {}".format(results[1]))
    print("Is the unknown face a new person that we've never seen before? {}".format(not True in results))

cam.release()
cv2.destroyAllWindows()
