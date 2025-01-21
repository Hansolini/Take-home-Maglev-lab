#This script uses the functions from the aruco_library.py module to process a video, identifying and tracking the rotation angle of the levitating magnet over time.

import cv2
import aruco_library
import matplotlib.pyplot as plt
import csv  # Import the csv module

# Path to the input video
cap = cv2.VideoCapture(r'C:\Users\giuse\OneDrive\Desktop\personale\Triennale\Terzo anno\Secondo semestre\Tesi\myThesis\video\maglev1.mp4')

# Define the frame rate
frame_rate = cap.get(cv2.CAP_PROP_FPS)  # Frames per second
angles = []  # List to store the angles of each frame
times = []  # List to store the corresponding times

# Define the codec and create a VideoWriter object to save the output video in AVI format
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec for AVI
out = cv2.VideoWriter(
    r'C:\Users\giuse\OneDrive\Desktop\personale\Triennale\Terzo anno\Secondo semestre\Tesi\myThesis\video\maggy1.avi',
    fourcc, 20.0, (int(cap.get(3)), int(cap.get(4)))
)

# Create and open the CSV file for writing data
with open(r'C:\Users\giuse\OneDrive\Desktop\personale\Triennale\Terzo anno\Secondo semestre\Tesi\myThesis\csv\measured_rotation.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    while True:
        ret, frame = cap.read()
        frame_no = cap.get(cv2.CAP_PROP_POS_FRAMES)  # Get the current frame number

        if not ret:
            break

        # Detect markers and calculate orientation
        detected_aruco_markers = aruco_library.detect_ArUco(frame)
        angle = aruco_library.Calculate_orientation(detected_aruco_markers)  # Calculate orientation
        img = aruco_library.mark_ArUco(frame, detected_aruco_markers, angle)
        cv2.imshow("Image", frame)

        # Write the frame to the output video
        out.write(frame)

        # If markers are detected, add data to the CSV
        if detected_aruco_markers:
            marker_id = list(detected_aruco_markers.keys())[0]  # Get the ID of the first detected marker
            marker_angle = angle.get(marker_id)
            if marker_angle is not None:
                angles.append(marker_angle)  # Add the angle to the list
                time_in_sec = frame_no / frame_rate  # Calculate the corresponding time
                times.append(time_in_sec)

                # Write the data to the CSV file
                writer.writerow([time_in_sec, marker_angle])

        # Exit by pressing the 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release video resources
cap.release()
out.release()
cv2.destroyAllWindows()

# Plot the graph of angles over time
plt.plot(times, angles, label='Angle (degrees)')
plt.xlabel('Time (seconds)')
plt.ylabel('Angle (degrees)')
plt.title('Rotation over Time')
plt.legend()
plt.grid(True)
plt.show()

