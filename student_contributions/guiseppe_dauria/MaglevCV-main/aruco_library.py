# In this library, we define functions that will be used to identify and track the rotation angle of ArUco markers.

import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import math
import time

def detect_ArUco(img):  # This method identifies each ArUco in an image and returns a dictionary of ArUcos,
                        # where the pairs consist of (key: ArUco's id, value: corners)

    Detected_ArUco_markers = {}  
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50) 
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
    parameters = cv2.aruco.DetectorParameters() 
    
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    for i in range(len(ids)):
        Detected_ArUco_markers.update({ids[i][0]: corners[i]})
    
    return Detected_ArUco_markers 

def Calculate_orientation(Detected_ArUco_markers):  # Function that calculates the angle of an ArUco relative to the z-axis and 
                                                    # returns a dictionary where the pairs consist of (key: ArUco's id, value: angle)

    ArUco_marker_angles = {}  

    for key in Detected_ArUco_markers:
        corners = Detected_ArUco_markers[key] 
        tl = corners[0][0]    
        tr = corners[0][1]    
        br = corners[0][2]    
        bl = corners[0][3]    
        top = (tl[0] + tr[0]) / 2, -((tl[1] + tr[1]) / 2)  # coordinates of the midpoint of the top edge
        centre = (tl[0] + tr[0] + bl[0] + br[0]) / 4, -((tl[1] + tr[1] + bl[1] + br[1]) / 4)  # coordinates of the center of the ArUco

        try:
            angle = round(math.degrees(np.arctan((top[1] - centre[1]) / (top[0] - centre[0]))))
        except:
            if (top[1] > centre[1]):
                angle = 90
            elif (top[1] < centre[1]):
                angle = 270

        if (top[0] >= centre[0] and top[1] < centre[1]):
            angle = 360 + angle 
        elif (top[0] < centre[0]):
            angle = 180 + angle 

        ArUco_marker_angles.update({key: angle}) 
    
    return ArUco_marker_angles  # Returns the dictionary with the angles of the ArUco markers.

def mark_ArUco(img, Detected_ArUco_markers, ArUco_marker_angles):  # Function that modifies an image, marking for each ArUco: id, vertices,
                                                                    #and angle relative to the z-axis
    
    for key in Detected_ArUco_markers:
        corners = Detected_ArUco_markers[key]  
        tl = corners[0][0]    
        tr = corners[0][1]    
        br = corners[0][2]    
        bl = corners[0][3]    

        top = int((tl[0] + tr[0]) / 2), int((tl[1] + tr[1]) / 2)
        centre = int((tl[0] + tr[0] + bl[0] + br[0]) / 4), int((tl[1] + tr[1] + bl[1] + br[1]) / 4)

        img = cv2.line(img, top, centre, (105, 24, 0), 3)

        img = cv2.circle(img, (int(tl[0]), int(tl[1])), 6, (100, 100, 100), -1)  
        img = cv2.circle(img, (int(tr[0]), int(tr[1])), 6, (0, 255, 0), -1)  
        img = cv2.circle(img, (int(br[0]), int(br[1])), 6, (100, 100, 255), -1)  
        img = cv2.circle(img, (int(bl[0]), int(bl[1])), 6, (255, 255, 255), -1)  

        img = cv2.circle(img, centre, 5, (0, 0, 255), -1)

        img = cv2.putText(img, str(key), centre, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 200), 3, cv2.LINE_AA)
        
        img = cv2.putText(img, str(ArUco_marker_angles[key]), top, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 80), 3, cv2.LINE_AA)
    
    return img


