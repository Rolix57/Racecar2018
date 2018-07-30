
import face_recognition
import cv2

video_capture = cv2.VideoCapture(0)

ahmad_image = face_recognition.load_image_file("ahmad.png")
ahmad_face_encoding = face_recognition.face_encodings(ahmad_image)[0]

fishberg_image = face_recognition.load_image_file("fishberg.png")
fishberg_face_encoding = face_recognition.face_encodings(fishberg_image)[0]

mali_image = face_recognition.load_image_file("mali.png")
mali_face_encoding = face_recognition.face_encodings(mali_image)[0]

cruz_image = face_recognition.load_image_file("cruz.png")
cruz_face_encoding = face_recognition.face_encodings(cruz_image)[0]

hassan_image = face_recognition.load_image_file("hassan.png")
hassan_face_encoding = face_recognition.face_encodings(hassan_image)[0]

nguyen_image = face_recognition.load_image_file("nguyen.png")
nguyen_face_encoding = face_recognition.face_encodings(nguyen_image)[0]

belyi_image = face_recognition.load_image_file("belyi.png")
belyi_face_encoding = face_recognition.face_encodings(belyi_image)[0]

plancher_image = face_recognition.load_image_file("plancher.png")
plancher_face_encoding = face_recognition.face_encodings(plancher_image)[0]

known_face_encodings = [
    ahmad_face_encoding,
    fishberg_face_encoding,
    mali_face_encoding,
    cruz_face_encoding,
    hassan_face_encoding,
    nguyen_face_encoding
]
known_face_names = [
    "Ahmad",
    "Fishberg",
    "Mali",
    "Cruz",
    "Hassan",
    "Nguyen",
    "Belyi",
    "Plancher"
]


face_locations = []
face_encodings = []
face_names = []
process_this_frame = True

while True:
    
    ret, frame = video_capture.read()

    
    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)


    rgb_small_frame = small_frame[:, :, ::-1]#

    
    if process_this_frame:
        
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

        face_names = []
        for face_encoding in face_encodings:
        
            matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
            name = "Unknown"


            if True in matches:
                first_match_index = matches.index(True)
                name = known_face_names[first_match_index]

            face_names.append(name)

    process_this_frame = not process_this_frame

    for (top, right, bottom, left), name in zip(face_locations, face_names):
        
        top *= 4
        right *= 4
        bottom *= 4
        left *= 4

        
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

        
        cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

    
    cv2.imshow('Video', frame)

    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


video_capture.release()
cv2.destroyAllWindows()