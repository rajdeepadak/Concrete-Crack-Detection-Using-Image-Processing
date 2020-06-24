# Running the same code on Rpi may cause latency issues. In such case either decrease the picture resolution or remove
# Gaussian Blur function. For proper functioning of Hough Circles function a primary blurred image is required.

import cv2
import numpy as np


def main():
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
                
                # If marker is in 1st Quadrant (UAV itself is in 3rd Quadrant)
                    if x_i > 0 and y_i < 0:
                        
                        # Roll and Pitch in Degrees
                        roll_d = -(12 * x_i_scaled)/(x_i_scaled + 15)
                        pitch_d = (12 * -y_i_scaled)/(-y_i_scaled + 15)
                        
                        # Roll and Pitch in Radians
                        roll_r = roll_d/57.29
                        pitch_r = pitch_d/57.29
                        
                        # Print Scaled Vector to marker and required roll & pitch to be given to UAV
                        print("Scaled Vector : ", x_i_scaled, "i", y_i_scaled, "j", "Roll: ", roll_r, "Pitch: ", pitch_r)
                
                # If marker is in 2nd Quadrant (UAV itself is in 4th Quadrant)
                    if x_i < 0 and y_i < 0:
                        
                        # Roll and Pitch in Degrees
                        roll_d = (12 * -x_i_scaled)/(-x_i_scaled + 15)
                        pitch_d = (12 * -y_i_scaled)/(-y_i_scaled + 15)
                        
                        # Roll and Pitch in Radians
                        roll_r = roll_d/57.29
                        pitch_r = pitch_d/57.29
                        
                        # Print Scaled Vector to marker and required roll & pitch to be given to UAV
                        print("Scaled Vector : ", x_i_scaled, "i", y_i_scaled, "j", "Roll: ", roll_r, "Pitch: ", pitch_r)

                # If marker is in 3rd Quadrant (UAV itself is in 1st Quadrant)        
                    if x_i < 0 and y_i > 0:
                        
                        # Roll and Pitch in Degrees
                        roll_d = (12 * -x_i_scaled)/(-x_i_scaled + 15)
                        pitch_d = -(12 * y_i_scaled)/(y_i_scaled + 15)
                        
                        # Roll and Pitch in Radians
                        roll_r = roll_d/57.29
                        pitch_r = pitch_d/57.29
                        
                        # Print Scaled Vector to marker and required roll & pitch to be given to UAV
                        print("Scaled Vector : ", x_i_scaled, "i", y_i_scaled, "j", "Roll: ", roll_r, "Pitch: ", pitch_r)

                # If marker is in 4th Quadrant (UAV itself is in 2nd Quadrant)        
                    if x_i > 0 and y_i > 0:
                        
                        # Roll and Pitch in Degrees
                        roll_d = -(12 * x_i_scaled)/(x_i_scaled + 15)
                        pitch_d = -(12 * y_i_scaled)/(y_i_scaled + 15)
                        
                        # Roll and Pitch in Radians
                        roll_r = roll_d/57.29
                        pitch_r = pitch_d/57.29
                        
                        # Print Scaled Vector to marker and required roll & pitch to be given to UAV
                        print("Scaled Vector : ", x_i_scaled, "i", y_i_scaled, "j", "Roll: ", roll_r, "Pitch: ", pitch_r)
                        
            else:
                print("Central Marker not found")
                
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


main()    