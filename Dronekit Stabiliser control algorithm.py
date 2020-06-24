from __future__ import print_function
from dronekit import connect, VehicleMode
import time
import cv2
import numpy as np


#Testing of OpenCV
def start():
    
    imgpath = "/home/pi/Desktop/dayum.jpeg"
    img = cv2.imread(imgpath)
    
    cv2.imshow('dayum', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
start()


# Set up option parsing to get connection string
# This set of code is NOT to be removed
# Don't try to understand the code below

import argparse  
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect', default = '/dev/ttyACM0',
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect



# Connect to the Vehicle. 
# Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.

print("\nConnecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True,baud = 921600)

# Vehicle status before launch
print(" Vehicle Status before launch : \n")
print(" Velocity: %s" % vehicle.velocity)
print(" GPS: %s" % vehicle.gps_0)
print(" Battery: %s" % vehicle.battery)
print(" EKF OK?: %s" % vehicle.ekf_ok)
print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
print(" Heading: %s" % vehicle.heading)
print(" Is Armable?: %s" % vehicle.is_armable)
print(" System status: %s" % vehicle.system_status.state)
print(" Groundspeed: %s" % vehicle.groundspeed)    # settable
print(" Mode: %s" % vehicle.mode.name)    # settable
print(" Armed: %s" % vehicle.armed)    # settable

def detect_marker():
    while vehicle.armed:
        
        global circles
        cap = cv2.VideoCapture(0)

        cap.set(3, 320)
        cap.set(4, 240)

        print(cap.get(3))
        print(cap.get(4))

        retval, frame = cap.read()

        if cap.isOpened():
            print(retval)
            print(frame)

        else:
            print(retval)

        while retval:
            ret, orig_frame = cap.read()

            # Gray Scale Video
            Gray_video = cv2.cvtColor(orig_frame, cv2.COLOR_BGR2GRAY)
            # cv2.imshow("Gray video", Gray_video)

            G_blur = cv2.GaussianBlur(Gray_video, (11, 11), cv2.BORDER_DEFAULT)
            
            # Bilaterally Filtered Video
            # Bil_Fil = cv2.bilateralFilter(Gray_video, 9, 350, 350)
            # cv2.imshow("Bilaterally Filtered Video", Bil_Fil)

            # Detecting Circles
            circles = cv2.HoughCircles(G_blur, cv2.HOUGH_GRADIENT, 0.9, 120, param1=100, param2=60, minRadius=3, maxRadius=200)

            # Central lines
            cv2.line(orig_frame, (0, 120), (320, 120), (0, 255, 0), 1)
            cv2.line(orig_frame, (160, 0), (160, 240), (0, 255, 0), 1)

            # Iron sight centre
            cv2.circle(orig_frame, (160, 120), 5, (0, 0, 255), 2)

            # Iron sight scope
            cv2.circle(orig_frame, (160, 120), 60, (0, 0, 255), 2)

            # Centre coordinates and quadrants
            cv2.putText(orig_frame, 'O(0, 0)', (170, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1, cv2.LINE_AA)
            cv2.putText(orig_frame, '1st Quadrant', (240, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.putText(orig_frame, '2nd Quadrant', (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.putText(orig_frame, '3rd Quadrant', (20, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.putText(orig_frame, '4th Quadrant', (240, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1, cv2.LINE_AA)

            if circles is not None:
                circles = np.uint16(np.around(circles))
                a = circles[0, 0, 0]
                b = circles[0, 0, 1]
                c = circles[0, 0, 2]
                a0 = int(a)
                b0 = int(b)
                c0 = int(c)

                # Vector Resolutions
                x_i = a0 - 160
                y_i = b0 - 120

                # Scaled Vector Resolutions
                x_i_scaled = x_i/5
                y_i_scaled = y_i/5

                # If UAV is inside Marker Iron sight
                if 100 < a0 < 220 and 60 < b0 < 180:
                    
                    # If UAV is in 1st Quadrant
                    if x_i < 0 and y_i > 0:
                        
                        # Roll and Pitch in Degrees
                        roll_d = (12 * -x_i_scaled)/(-x_i_scaled + 15)
                        pitch_d = (12 * y_i_scaled)/(y_i_scaled + 15)
                        
                        # Roll and Pitch in Radians
                        roll_r = roll_d/57.29
                        pitch_r = pitch_d/57.29
                        
                        print("Central Circle: (", a0, ",", b0, ")", ", radius: ", c0, "Scaled Vector: ", x_i_scaled,
                              "i", y_i_scaled, "j", "\nRoll_c: ", -roll_r, "Pitch_c: ", pitch_r, vehicle.attitude, vehicle.armed)
                        
                    # If UAV is in 2nd Quadrant
                    if x_i > 0 and y_i > 0:
                        
                        # Roll and Pitch in Degrees
                        roll_d = (12 * x_i_scaled)/(x_i_scaled + 15)
                        pitch_d = (12 * y_i_scaled)/(y_i_scaled + 15)
                        
                        # Roll and Pitch in Radians
                        roll_r = roll_d/57.29
                        pitch_r = pitch_d/57.29
                        
                        print("Central Circle: (", a0, ",", b0, ")", ", radius: ", c0, "Scaled Vector: ", x_i_scaled,
                              "i", y_i_scaled, "j", "\nRoll_c: ", roll_r, "Pitch_c: ", pitch_r, vehicle.attitude, vehicle.armed)
                        
                    # If UAV is in 3rd Quadrant
                    if x_i > 0 and y_i < 0:
                        
                        # Roll and Pitch in Degrees
                        roll_d = (12 * x_i_scaled)/(x_i_scaled + 15)
                        pitch_d = (12 * -y_i_scaled)/(-y_i_scaled + 15)
                        
                        # Roll and Pitch in Radians
                        roll_r = roll_d/57.29
                        pitch_r = pitch_d/57.29
                        
                        print("Central Circle: (", a0, ",", b0, ")", ", radius: ", c0, "Scaled Vector: ", x_i_scaled,
                              "i", y_i_scaled, "j", "\nRoll_c: ", roll_r, "Pitch_c: ", -pitch_r, vehicle.attitude, vehicle.armed)
                        
                    # If UAV is in 4th Quadrant
                    if x_i < 0 and y_i > 0:
                        
                        # Roll and Pitch in Degrees
                        roll_d = (12 * -x_i_scaled)/(-x_i_scaled + 15)
                        pitch_d = (12 * y_i_scaled)/(y_i_scaled + 15)
                        
                        # Roll and Pitch in Radians
                        roll_r = roll_d/57.29
                        pitch_r = pitch_d/57.29
                        
                        print("Central Circle: (", a0, ",", b0, ")", ", radius: ", c0, "Scaled Vector: ", x_i_scaled,
                              "i", y_i_scaled, "j", "\nRoll_c: ", -roll_r, "Pitch_c: ", pitch_r, vehicle.attitude, vehicle.armed)
                        
                else:
                    print("Central Marker Not Found")

                for i in circles[0, :]:
                    # Draw outer circle
                    cv2.circle(orig_frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
                    # Draw inner circle
                    cv2.circle(orig_frame, (i[0], i[1]), i[2], (0, 0, 255), 3)
                    # Draw vector9
                    cv2.line(orig_frame, (a0, b0), (160, 120), (0, 0, 255), 1)

            cv2.imshow("Circles Observed", orig_frame)

            
            if cv2.waitKey(1) == 27:
                break

        # Data of Circles Array
        #print("Data type of Array Circles is : ", type(circles))
        #print("Number of Dimensions of Circles : ", circles.ndim)
        #print("Shape of Array is : ", circles.shape)
        #print("Size of Array is : ", circles.size)
        #print("Array stores elements of type : ", circles.dtype)

        cv2.destroyAllWindows()
        cap.release()
        
def arm():

    print("Arming motors in 10 seconds")

    time.sleep(10)

    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True
    print(" Mode: %s" % vehicle.mode.name) 
    time.sleep(2)
    

    
def takeoff(aTargetAltitude):
    print("Taking off!")
    
    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.7
    SMOOTH_TAKEOFF_THRUST = 0.6
    
    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        detect_marker()
        current_altitude = vehicle.location.global_relative_frame.alt
        print(" Altitude: %f  Desired: %f" %
              (current_altitude, aTargetAltitude))
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            print("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust = thrust)
        time.sleep(0.2)
    

    
def main():
 
    arm()
    takeoff(2.5)
    
    

print(vehicle.armed)
   

# Get all vehicle attributes (state)
# print("\nGet all vehicle attribute values:")
# print(" Autopilot Firmware version: %s" % vehicle.version)
# print(" Major version number: %s" % vehicle.version.major)
# print(" Minor version number: %s" % vehicle.version.minor)
# print(" Patch version number: %s" % vehicle.version.patch)
# print(" Release type: %s" % vehicle.version.release_type())
# print(" Release version: %s" % vehicle.version.release_version())
# print(" Stable release?: %s" % vehicle.version.is_stable())
# print(" Autopilot capabilities")
#print(" Gimbal status: %s" % vehicle.gimbal)
#print(" Rangefinder: %s" % vehicle.rangefinder)
#print(" Rangefinder distance: %s" % vehicle.rangefinder.distance)
#print(" Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
#print(" Airspeed: %s" % vehicle.airspeed)   # settable
#print("   Supports PARAM_UNION message type: %s" % vehicle.capabilities.param_union)
#print("   Supports ftp for file transfers: %s" % vehicle.capabilities.ftp)
#print("   Supports commanding attitude offboard: %s" % vehicle.capabilities.set_attitude_target)
#print("   Supports commanding position and velocity targets in local NED frame: %s" % vehicle.capabilities.set_attitude_target_local_ned)
#print("   Supports set position + velocity targets in global scaled integers: %s" % vehicle.capabilities.set_altitude_target_global_int)
#print("   Supports terrain protocol / data handling: %s" % vehicle.capabilities.terrain)
#print("   Supports direct actuator control: %s" % vehicle.capabilities.set_actuator_target)
#print("   Supports the flight termination command: %s" % vehicle.capabilities.flight_termination)
#print("   Supports mission_float message type: %s" % vehicle.capabilities.mission_float)
#print("   Supports onboard compass calibration: %s" % vehicle.capabilities.compass_calibration)
#print(" Global Location: %s" % vehicle.location.global_frame)
#print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
#print(" Local Location: %s" % vehicle.location.local_frame)
