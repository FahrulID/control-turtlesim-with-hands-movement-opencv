#!/usr/bin/env python3
import cv2
import time
import numpy as np
import mediapipe as mp
import matplotlib.pyplot as plt
from publisher import publisher

######################################## CORE FUNCTIONS

# Initialize the mediapipe hands class.
mp_hands = mp.solutions.hands

# Set up the Hands functions for images and videos.
hands = mp_hands.Hands(static_image_mode=True, max_num_hands=2, min_detection_confidence=0.5)
hands_videos = mp_hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.5)

# Initialize the mediapipe drawing class.
mp_drawing = mp.solutions.drawing_utils
class core():
    def __init__(self, width, height):
        print("Initiated Core functions...")
        self.width = width
        self.height = height

    def detectHandsLandmarks(self, image, hands, draw=True, display = True):
        '''
        This function performs hands landmarks detection on an image.
        Args:
            image:   The input image with prominent hand(s) whose landmarks needs to be detected.
            hands:   The Hands function required to perform the hands landmarks detection.
            draw:    A boolean value that is if set to true the function draws hands landmarks on the output image. 
            display: A boolean value that is if set to true the function displays the original input image, and the output 
                    image with hands landmarks drawn if it was specified and returns nothing.
        Returns:
            output_image: A copy of input image with the detected hands landmarks drawn if it was specified.
            results:      The output of the hands landmarks detection on the input image.
        '''
        
        # Create a copy of the input image to draw landmarks on.
        output_image = image.copy()
        
        # Convert the image from BGR into RGB format.
        imgRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Perform the Hands Landmarks Detection.
        results = hands.process(imgRGB)
        
        # Check if landmarks are found and are specified to be drawn.
        if results.multi_hand_landmarks and draw:
            
            # Iterate over the found hands.
            for hand_landmarks in results.multi_hand_landmarks:
                
                # Draw the hand landmarks on the copy of the input image.
                mp_drawing.draw_landmarks(image = output_image, landmark_list = hand_landmarks,
                                        connections = mp_hands.HAND_CONNECTIONS,
                                        landmark_drawing_spec=mp_drawing.DrawingSpec(color=(255,255,255),
                                                                                    thickness=2, circle_radius=2),
                                        connection_drawing_spec=mp_drawing.DrawingSpec(color=(0,255,0),
                                                                                        thickness=2, circle_radius=2))
        
        # Check if the original input image and the output image are specified to be displayed.
        if display:
            
            # Display the original input image and the output image.
            plt.figure(figsize=[15,15])
            plt.subplot(121);plt.imshow(image[:,:,::-1]);plt.title("Original Image");plt.axis('off');
            plt.subplot(122);plt.imshow(output_image[:,:,::-1]);plt.title("Output");plt.axis('off');
            
        # Otherwise
        else:
            
            # Return the output image and results of hands landmarks detection.
            return output_image, results              

    def countFingers(self, image, results, draw=True, display=True):
        '''
        This function will count the number of fingers up for each hand in the image.
        Args:
            image:   The image of the hands on which the fingers counting is required to be performed.
            results: The output of the hands landmarks detection performed on the image of the hands.
            draw:    A boolean value that is if set to true the function writes the total count of fingers of the hands on the
                    output image.
            display: A boolean value that is if set to true the function displays the resultant image and returns nothing.
        Returns:
            output_image:     A copy of the input image with the fingers count written, if it was specified.
            fingers_statuses: A dictionary containing the status (i.e., open or close) of each finger of both hands.
            count:            A dictionary containing the count of the fingers that are up, of both hands.
        '''
        
        # Get the height and width of the input image.
        height, width, _ = image.shape
        
        # Create a copy of the input image to write the count of fingers on.
        output_image = image.copy()
        
        # Initialize a dictionary to store the count of fingers of both hands.
        count = {'RIGHT': 0, 'LEFT': 0}
        
        # Store the indexes of the tips landmarks of each finger of a hand in a list.
        fingers_tips_ids = [mp_hands.HandLandmark.INDEX_FINGER_TIP, mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
                            mp_hands.HandLandmark.RING_FINGER_TIP, mp_hands.HandLandmark.PINKY_TIP]
        
        # Initialize a dictionary to store the status (i.e., True for open and False for close) of each finger of both hands.
        fingers_statuses = {'RIGHT_THUMB': False, 'RIGHT_INDEX': False, 'RIGHT_MIDDLE': False, 'RIGHT_RING': False,
                            'RIGHT_PINKY': False, 'LEFT_THUMB': False, 'LEFT_INDEX': False, 'LEFT_MIDDLE': False,
                            'LEFT_RING': False, 'LEFT_PINKY': False}
        
        
        # Iterate over the found hands in the image.
        for hand_index, hand_info in enumerate(results.multi_handedness):
            
            # Retrieve the label of the found hand.
            hand_label = hand_info.classification[0].label
            
            # Retrieve the landmarks of the found hand.
            hand_landmarks =  results.multi_hand_landmarks[hand_index]
            
            # Iterate over the indexes of the tips landmarks of each finger of the hand.
            for tip_index in fingers_tips_ids:
                
                # Retrieve the label (i.e., index, middle, etc.) of the finger on which we are iterating upon.
                finger_name = tip_index.name.split("_")[0]
                
                # Check if the finger is up by comparing the y-coordinates of the tip and pip landmarks.
                if (hand_landmarks.landmark[tip_index].y < hand_landmarks.landmark[tip_index - 2].y):
                    
                    # Update the status of the finger in the dictionary to true.
                    fingers_statuses[hand_label.upper()+"_"+finger_name] = True
                    
                    # Increment the count of the fingers up of the hand by 1.
                    count[hand_label.upper()] += 1
            
            # Retrieve the y-coordinates of the tip and mcp landmarks of the thumb of the hand.
            thumb_tip_x = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x
            thumb_mcp_x = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP - 2].x
            
            # Check if the thumb is up by comparing the hand label and the x-coordinates of the retrieved landmarks.
            if (hand_label=='Right' and (thumb_tip_x < thumb_mcp_x)) or (hand_label=='Left' and (thumb_tip_x > thumb_mcp_x)):
                
                # Update the status of the thumb in the dictionary to true.
                fingers_statuses[hand_label.upper()+"_THUMB"] = True
                
                # Increment the count of the fingers up of the hand by 1.
                count[hand_label.upper()] += 1
        
        # Check if the total count of the fingers of both hands are specified to be written on the output image.
        if draw:

            # Write the total count of the fingers of both hands on the output image.
            cv2.putText(output_image, " Total Fingers: " + str(sum(count.values())), (int((self.width/2) - 150), 25),cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)

        # Check if the output image is specified to be displayed.
        if display:
            
            # Display the output image.
            plt.figure(figsize=[10,10])
            plt.imshow(output_image[:,:,::-1]);plt.title("Output Image");plt.axis('off');
        
        # Otherwise
        else:

            # Return the output image, the status of each finger and the count of the fingers up of both hands.
            return output_image, fingers_statuses, count

    def recognizeGestures(self, image, fingers_statuses, count, draw=True, display=True):
        '''
        This function will determine the gesture of the left and right hand in the image.
        Args:
            image:            The image of the hands on which the hand gesture recognition is required to be performed.
            fingers_statuses: A dictionary containing the status (i.e., open or close) of each finger of both hands. 
            count:            A dictionary containing the count of the fingers that are up, of both hands.
            draw:             A boolean value that is if set to true the function writes the gestures of the hands on the
                            output image, after recognition.
            display:          A boolean value that is if set to true the function displays the resultant image and 
                            returns nothing.
        Returns:
            output_image:   A copy of the input image with the left and right hand recognized gestures written if it was 
                            specified.
            hands_gestures: A dictionary containing the recognized gestures of the right and left hand.
        '''
        
        # Create a copy of the input image.
        output_image = image.copy()
        
        # Store the labels of both hands in a list.
        hands_labels = ['RIGHT', 'LEFT']
        
        # Initialize a dictionary to store the gestures of both hands in the image.
        hands_gestures = {'RIGHT': "UNKNOWN", 'LEFT': "UNKNOWN"}
        
        # Iterate over the left and right hand.
        for hand_index, hand_label in enumerate(hands_labels):
            
            # Initialize a variable to store the color we will use to write the hands gestures on the image.
            # Initially it is red which represents that the gesture is not recognized.
            color = (0, 0, 255)
            
            if count[hand_label] == 2 and fingers_statuses[hand_label+'_INDEX'] and fingers_statuses[hand_label+'_THUMB']:
                    
                # Update the gesture value of the hand that we are iterating upon to SPIDERMAN SIGN.
                hands_gestures[hand_label] = "FORWARD"

                # Update the color value to green.
                color=(0,255,0)

            elif count[hand_label] == 3 and fingers_statuses[hand_label+'_INDEX'] and fingers_statuses[hand_label+'_THUMB'] and fingers_statuses[hand_label+'_MIDDLE']:
                    
                # Update the gesture value of the hand that we are iterating upon to SPIDERMAN SIGN.
                hands_gestures[hand_label] = "FORWARD_SPEED"

                # Update the color value to green.
                color=(0,255,0)
                
            # Check if the number of fingers up is 3 and the fingers that are up, are the thumb, index and the pinky finger.
            elif count[hand_label] == 1 and fingers_statuses[hand_label+'_INDEX']:
                    
                # Update the gesture value of the hand that we are iterating upon to SPIDERMAN SIGN.
                hands_gestures[hand_label] = "INDEX"

                # Update the color value to green.
                color=(0,255,0)
                
            # Check if the number of fingers up is 3 and the fingers that are up, are the thumb, index and the pinky finger.
            elif count[hand_label] == 5:
                    
                # Update the gesture value of the hand that we are iterating upon to SPIDERMAN SIGN.
                hands_gestures[hand_label] = "HIGH_FIVE"

                # Update the color value to green.
                color=(0,255,0)

            # Check if the hands gestures are specified to be written.
            if draw:
            
                # Write the hand gesture on the output image. 
                cv2.putText(output_image, hand_label +': '+ hands_gestures[hand_label] , (10, (hand_index+1) * 60),
                            cv2.FONT_HERSHEY_PLAIN, 4, color, 5)
                
        
        # Check if the output image is specified to be displayed.
        if display:

            # Display the output image.
            plt.figure(figsize=[10,10])
            plt.imshow(output_image[:,:,::-1]);plt.title("Output Image");plt.axis('off');
        
        # Otherwise
        else:

            # Return the output image and the gestures of the both hands.
            return output_image, hands_gestures

class app():
    def __init__(self):
        self.wCam = 1280
        self.hCam = 960

        self.camera_video = cv2.VideoCapture(0)
        self.camera_video.set(3,1280)
        self.camera_video.set(4,960)

        self.core = core(self.wCam, self.hCam)
        self.publisher = publisher()

        # Create named window for resizing purposes.
        cv2.namedWindow('Fahrul R Putra', cv2.WINDOW_NORMAL)

        ##########[________Values________]##########
        self.speed = 0
        self.minSpeed, self.maxSpeed = 0, 1000
        self.speedBar = 400 # Speed bar
        self.speedPer = 0 # Speed Percentage

        self.moveForward = 0
        self.moveBelok = 0

        ##########[______Lasst Frame Value______]#########
        self.last_speed = 0
        self.last_moveForward = 0
        self.last_moveBelok = 0

        self.detectHandsLandmarks = True
        self.fingerCounts = True

        self.start()
    
    def start(self):
        # Iterate until the webcam is accessed successfully.
        while self.camera_video.isOpened():
            
            # Read a self.frame.
            ok, self.frame = self.camera_video.read()
            
            # Check if self.frame is not read properly then continue to the next iteration to read the next self.frame.
            if not ok:
                continue
            
            # Flip the self.frame horizontally for natural (selfie-view) visualization.
            self.frame = cv2.flip(self.frame, 1)
            
            # Get the height and width of the self.frame of the webcam video.
            frame_height, frame_width, _ = self.frame.shape
            
            # Perform Hands landmarks detection on the self.frame.
            self.frame, results = self.core.detectHandsLandmarks(self.frame, hands_videos, self.detectHandsLandmarks, False)
            
            # Check if the hands landmarks in the self.frame are detected.
            if results.multi_hand_landmarks:
                    
                # Count the number of fingers up of each hand in the self.frame.
                self.frame, fingers_statuses, count = self.core.countFingers(self.frame, results, self.fingerCounts, False)
                
                # Perform the hand gesture recognition on the hands in the self.frame.
                _, hands_gestures = self.core.recognizeGestures(self.frame, fingers_statuses, count, True, False)

                ### Check for gesutres
                self.Move(hands_gestures)

            
            self.speedBarUI()
            
            # Display the self.frame.
            cv2.imshow('Fahrul R Putra', self.frame)
            
            # Check if 'ESC' is pressed and break the loop.
            self.handleStop()

    def handleStop(self):
        # Wait for 1ms. If a key is pressed, retreive the ASCII code of the key.
        k = cv2.waitKey(1) & 0xFF
        if(k == 27):
            # Release the VideoCapture Object and close the windows.
            self.camera_video.release()
            cv2.destroyAllWindows()


    ############################################## Core Movement Functions
    def Move(self, gesture):
        # Save last frame value
        self.last_speed = self.speed
        self.last_moveForward = self.moveForward
        self.last_moveBelok = self.moveBelok

        # Check if the gesture of any hand in the self.frame is SPIDERMAN SIGN.
        left = gesture["LEFT"]
        right = gesture["RIGHT"]

        if ((right == "FORWARD" or right == "FORWARD_SPEED") and (left == "FORWARD" or left == "FORWARD_SPEED") and (right != "FORWARD_SPEED" and left != "FORWARD_SPEED")):
            self.moveForward = 1
            self.moveBelok = 0
            
            if(right == "FORWARD_SPEED"):
                self.changeSpeed(1)
            elif(left == "FORWARD_SPEED"):
                self.changeSpeed(-1)
        elif right == "FORWARD_SPEED" and left == "FORWARD_SPEED":
            self.speed = 1000
            self.changeSpeed(0)
        elif right == "FORWARD_SPEED" or left == "FORWARD_SPEED":
            if(right == "FORWARD_SPEED"):
                self.changeSpeed(1)
            elif(left == "FORWARD_SPEED"):
                self.changeSpeed(-1)
            self.moveForward = 0
            self.moveBelok = 0
        elif (right == "FORWARD" and left == "INDEX"):
            self.moveForward = 1
            self.moveBelok = 1
        elif (right == "INDEX" and left == "FORWARD"):
            self.moveForward = 1
            self.moveBelok = -1
        elif (right == "FORWARD" and left != "FORWARD"):
            self.moveForward = 0
            self.moveBelok = 1
        elif (right != "FORWARD" and left == "FORWARD"):
            self.moveForward = 0
            self.moveBelok = -1
        elif (right == "INDEX" and left == "INDEX"):
            self.moveForward = -1
            self.moveBelok = 0
        elif (right == "INDEX" and left == "HIGH_FIVE"):
            self.moveForward = -1
            self.moveBelok = 1
        elif (right == "HIGH_FIVE" and left == "INDEX"):
            self.moveForward = -1
            self.moveBelok = -1
        elif right == "HIGH_FIVE" and left == "HIGH_FIVE":
            self.speed = 0
            self.moveForward = 0
            self.moveBelok = 0
            self.changeSpeed(0)
        else:
            self.moveForward = 0
            self.moveBelok = 0
        
        self.moveUI()
        self.checkForChange()

    def changeSpeed(self, direction):
        if direction > 0 and self.speed + 10 <= self.maxSpeed:
            self.speed += 50
        elif direction < 0 and self.speed - 10 >= self.minSpeed:
            self.speed -= 50

        self.speedBar = int(400 - (250 * self.speed / self.maxSpeed))
        self.speedPer = int(self.speed / self.maxSpeed * 100)

    def speedBarUI(self):
        cv2.rectangle(self.frame, (50, 150), (85, 400), (255, 0, 0), 3)
        cv2.rectangle(self.frame, (50, self.speedBar), (85, 400), (255, 0, 0), cv2.FILLED)
        cv2.putText(self.frame, f'{int(self.speedPer)} %', (40, 450), cv2.FONT_HERSHEY_COMPLEX,
                    1, (255, 0, 0), 3)

    def moveUI(self):
        if self.moveForward > 0:
            cv2.putText(self.frame, f'MAJU', (40, 40), cv2.FONT_HERSHEY_COMPLEX,
                    1, (255, 0, 0), 3)
        elif self.moveForward < 0:
            cv2.putText(self.frame, f'MUNDUR', (40, 40), cv2.FONT_HERSHEY_COMPLEX,
                    1, (255, 0, 0), 3)
        else:
            cv2.putText(self.frame, f'DIAM', (40, 40), cv2.FONT_HERSHEY_COMPLEX,
                    1, (255, 0, 0), 3)
        
        if self.moveBelok < 0:
            cv2.putText(self.frame, f'KIRI', (40, 100), cv2.FONT_HERSHEY_COMPLEX,
                    1, (255, 0, 0), 3)
        elif self.moveBelok > 0:
            cv2.putText(self.frame, f'KANAN', (40, 100), cv2.FONT_HERSHEY_COMPLEX,
                    1, (255, 0, 0), 3)
        else:
            cv2.putText(self.frame, f'', (40, 100), cv2.FONT_HERSHEY_COMPLEX,
                    1, (255, 0, 0), 3)

    def sendData(self, speed, forward, belok):
        # Serialize data to speed;forward;belok
        msg = str(speed/1000) + ";" + str(forward) + ";" + str(belok)
        self.publisher.talk(msg)

    def checkForChange(self):
        if(self.speed != self.last_speed or self.moveForward != self.last_moveForward or self.moveBelok != self.last_moveBelok):
            self.sendData(self.speed, self.moveForward, self.moveBelok)

###############################
def main():
    start = app()

        
if __name__ == '__main__':
    main()