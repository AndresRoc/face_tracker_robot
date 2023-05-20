import cv2
import numpy as np
import serial
import time
from serial.tools import list_ports


def find_arduino_port():
    ports = list_ports.comports()
    for port in ports:
        if 'Arduino' in port.description or 'USB Serial Device' in port.description:
            return port.device
    return None

def calculate_velocity(x, y, z, distance):
    target_distance = 100  # 100 centimeters = 1 meter
    distance_error = target_distance - distance

    # Calculate linear velocity in the X and Y direction (in centimeters)
    linear_velocity_x = x / 100
    linear_velocity_y = y / 100

    # Calculate angular velocity in the Z direction
    angular_velocity_z = distance_error / 100

    return linear_velocity_x, linear_velocity_y, angular_velocity_z

def send_data_to_arduino(linear_velocity_x, linear_velocity_y, angular_velocity_z):
    arduino.write(f'{linear_velocity_x},{linear_velocity_y},{angular_velocity_z}\n'.encode())
    print(f'{linear_velocity_x},{linear_velocity_y},{angular_velocity_z}')
    read_arduino_response()

def read_arduino_response():
    while arduino.in_waiting > 0:
        response = arduino.readline().decode().strip()
        print(f"Arduino response: {response}")

arduino_serial_port = find_arduino_port()
if arduino_serial_port is None:
    print("Arduino not found. Please check the connection and try again.")
    exit(1)
else:
    print("Arduino found at: " + str(arduino_serial_port))

serial_baud_rate = 115200
arduino = serial.Serial(arduino_serial_port, serial_baud_rate, timeout=10)
time.sleep(2)
# # Flush input and output buffers
arduino.flushInput()
arduino.flushOutput()

cap = cv2.VideoCapture(0)
_, frame = cap.read()

# Display instructions on the frame
instruction_text = "Select a ROI and then press SPACE or ENTER button!"
cv2.putText(frame, instruction_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
cv2.imshow("Tracking", frame)
cv2.waitKey(1000)  # Wait for 1 second to give the user time to read the instructions

bbox = cv2.selectROI("Tracking", frame, False)
tracker = cv2.TrackerCSRT_create()
tracker.init(frame, bbox)

# Actual height of the object (in meters)
actual_height = 1.6 # 0.3

# Focal length of the camera (in pixels)
focal_length = 500

while True:
    _, frame = cap.read()

    success, bbox = tracker.update(frame)
    if success:
        x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # Calculate coordinates and distance
        x_coord = x + w // 2
        y_coord = y + h // 2
        
        # Estimate distance
        distance = (actual_height * focal_length) / h
        distance = round(distance, 2)  # Round to 2 decimal places

        # Set Z coordinate to the estimated distance
        z_coord = distance

        # Calculate velocities
        linear_velocity_x, linear_velocity_y, angular_velocity_z = calculate_velocity(x_coord, y_coord, z_coord, distance)

        # Prints results
        coord_text = f"X:{x_coord}, Y:{y_coord}, D:{distance}"
        velocity_text = f"Vx:{linear_velocity_x}, Vy:{linear_velocity_y}, Vz:{angular_velocity_z}"
        cv2.putText(frame, coord_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(frame, velocity_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        # Draw frame to the right
        cv2.line(frame,(420, 0),(420,600),(0,255,0), thickness=1)
        # Draw frame to the left
        cv2.line(frame,(220, 0),(220,600),(0,255,0), thickness=1)

        # Calculate the center of the face
        face_center = x + w / 2

        # Compute the linear and angular velocities
        if distance >= 100 and distance <= 200:
            linear_velocity_x = 0  # No need to move forward or backward
            linear_velocity_y = 0  # Face is in the center, so no need to move sideways
            angular_velocity_z = 0
        else:
            linear_velocity_x = 10 if distance < 100 else -10  # Move forward or backward as required

        if face_center >= 220 and face_center <= 420:
            linear_velocity_x = 0
            linear_velocity_y = 0  # Face is in the center, so no need to move sideways
            angular_velocity_z = 0
        elif face_center < 220:
            linear_velocity_y = -10  # Move left as the face is on the left
        else:
            linear_velocity_y = 10  # Move right as the face is on the right

        angular_velocity_z = 0  # Assuming no rotation needed in this case

        # Send data to Arduino
        send_data_to_arduino(linear_velocity_x, linear_velocity_y, angular_velocity_z)
        # read_arduino_response()

    cv2.imshow("Tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
