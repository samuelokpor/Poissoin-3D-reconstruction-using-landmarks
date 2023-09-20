# import mediapipe as mp
# import cv2
# import csv
# import os
# import numpy as np



# mp_drawing = mp.solutions.drawing_utils
# mp_holistic = mp.solutions.holistic

# cap = cv2.VideoCapture(0)
# #intiate holistic model
# with mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
#     while cap.isOpened():
#         ret, frame = cap.read()

#         #recolor Feed
#         image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#         image.flags.writeable = False
#         #make detections
#         results = holistic.process(image)
#         #primt(results.face_landmarks)

#         #face_landmarks,pose_landmarks, left_hand_landmarks, right_hand_landmarks

#         #recolor image back to BGR for rendering
#         image.flags.writeable = True
#         image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

#         #1. draw face landmarks
#         mp_drawing.draw_landmarks(image, results.face_landmarks, mp_holistic.FACEMESH_CONTOURS,
#                                     mp_drawing.DrawingSpec(color=(80,110,10), thickness=1, circle_radius=1),
#                                     mp_drawing.DrawingSpec(color=(80,256,121), thickness=1, circle_radius=1)
#                                     )

#         cv2.imshow('Raw Webcam Feed', image)

#         if cv2.waitKey(10)& 0xFF == ord('q'):
#             break
# cap.release()
# cv2.destroyAllWindows()    

# num_coords = len(results.face_landmarks.landmark)

# landmarks = ['class']
# for val in range(1, num_coords+1):
#     landmarks += ['x{}'.format(val), 'y{}'.format(val), 'z{}'.format(val), 'v{}'.format(val)]        

# print(landmarks)                            


import mediapipe as mp
import cv2
import json
import numpy as np

mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic

cap = cv2.VideoCapture(0)

view = "front"  # Initialize with the front view

# Initialize holistic model
with mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
    while cap.isOpened():
        ret, frame = cap.read()

        # Recolor Feed
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        # Make detections
        results = holistic.process(image)

        # Recolor image back to BGR for rendering
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # Draw face landmarks
        mp_drawing.draw_landmarks(image, results.face_landmarks, mp_holistic.FACEMESH_CONTOURS,
                                  mp_drawing.DrawingSpec(color=(80, 110, 10), thickness=1, circle_radius=1),
                                  mp_drawing.DrawingSpec(color=(80, 256, 121), thickness=1, circle_radius=1))

        # Show current view
        cv2.putText(image, f'Current View: {view.capitalize()}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        
        cv2.imshow('Raw Webcam Feed', image)

        key = cv2.waitKey(10) & 0xFF

        if key == ord('f'):
            view = "front"
        elif key == ord('l'):
            view = "left"
        elif key == ord('r'):
            view = "right"
        elif key == ord('s'):
            landmarks_data = []
            for landmark in results.face_landmarks.landmark:
                landmark_dict = {
                    'x': landmark.x,
                    'y': landmark.y,
                    'z': landmark.z,
                    'v': landmark.visibility
                }
                landmarks_data.append(landmark_dict)
            
            with open(f'{view}_landmarks.json', 'w') as f:
                json.dump(landmarks_data, f)
        
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

