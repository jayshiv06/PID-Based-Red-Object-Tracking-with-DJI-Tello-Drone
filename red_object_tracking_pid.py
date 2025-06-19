"""
Author: Jayharish Shivakumar
Project: PID-Based Red Object Tracking with DJI Tello Drone
Platform: Python, OpenCV, djitellopy, matplotlib
Date: 18th June 2025

Description:
This project implements real-time red object detection and tracking using a DJI Tello drone.
It uses PID control for adjusting drone movements (yaw, altitude, and forward/backward) based on object position and area.
Live error plots are generated using matplotlib for easier PID tuning and visualization.

© 2025 Jayharish Shivakumar. All rights reserved.
"""

from djitellopy import tello
import cv2
import numpy as np
import time
import matplotlib.pyplot as plt

# Initialize matplotlib
plt.ion()
fig, ax = plt.subplots()
ax.set_title("PID Errors")
ax.set_xlabel("Time (frames)")
ax.set_ylabel("Error")

x_errors, y_errors, area_errors = [], [], []
frame_count = 0
plot_interval = 10  # Update plot every 10 frames

drone = tello.Tello()
drone.connect()
print("Battery: ", drone.get_battery())
drone.streamon()
time.sleep(3)
drone.takeoff()

FRAME_WIDTH = 960
FRAME_HEIGHT = 720
CENTER_X = FRAME_WIDTH // 2
CENTER_Y = FRAME_HEIGHT // 2

KP_x, KI_x, KD_x = 0.25, 0.0002, 0.03
KP_y, KI_y, KD_y = 0.25, 0.0002, 0.03
KP_area, KI_area, KD_area = 0.006, 0.000005, 0.03
x_integral = y_integral = area_integral = 0
area_setpoint = 10000
prev_error_x = prev_error_y = prev_error_area = 0
prev_time = time.time()

while True:
    frame = drone.get_frame_read().frame
    frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        if area > 800:
            x, y, w, h = cv2.boundingRect(largest_contour)
            cx = x + w // 2
            cy = y + h // 2

            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (155, 0, 0), -1)

            current_time = time.time()
            dt = current_time - prev_time

            # Horizontal (yaw)
            error_x = CENTER_X - cx
            x_integral += error_x
            x_derivative = (error_x - prev_error_x) / dt
            prev_error_x = error_x
            output_x = output_yaw = int(np.clip(KP_x * error_x + KI_x * x_integral + KD_x * x_derivative, -100, 100))

            # Vertical (altitude)
            error_y = CENTER_Y - cy
            y_integral += error_y
            y_derivative = (error_y - prev_error_y) / dt
            prev_error_y = error_y
            output_y = int(np.clip(KP_y * error_y + KI_y * y_integral + KD_y * y_derivative, -100, 100))

            # Forward/backward (distance via area)
            error_area = area_setpoint - area
            area_integral += error_area
            area_derivative = (error_area - prev_error_area) / dt
            prev_error_area = error_area
            output_area = int(np.clip(KP_area * error_area + KI_area * area_integral + KD_area * area_derivative, -100, 100))

            prev_time = current_time

            drone.send_rc_control(0, output_area, output_y, -output_yaw)

            # Add errors to lists
            x_errors.append(error_x)
            y_errors.append(error_y)
            area_errors.append(error_area)

            frame_count += 1
            if frame_count % plot_interval == 0:
                ax.clear()
                ax.plot(x_errors, label="Error X", color='r')
                ax.plot(y_errors, label="Error Y", color='g')
                ax.plot(area_errors, label="Error Area", color='b')
                ax.set_title("PID Errors Over Time")
                ax.set_xlabel("Frame")
                ax.set_ylabel("Error Value")
                ax.legend()
                plt.pause(0.001)
        else:
            drone.send_rc_control(0, 0, 0, 0)

    cv2.imshow("Red object tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        drone.send_rc_control(0, 0, 0, 0)
        break

drone.streamoff()
drone.reboot()
drone.end()
cv2.destroyAllWindows()
