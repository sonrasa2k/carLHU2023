import cv2
import numpy as np
cap = cv2.VideoCapture("output.mp4")
# Load the image
# image = cv2.imread("image.jpg")

while True:
    ret,frame = cap.read()
    frame = cv2.resize(frame,(640,480))
    # Pre-process the image
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)

    # Detect lanes
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=20, maxLineGap=4)

    # Draw the lanes on the image
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # Calculate the steering angle
    left_lane = [line for line in lines if line[0][1] > frame.shape[0]/2]
    right_lane = [line for line in lines if line[0][1] <= frame.shape[0]/2]

    if left_lane and right_lane:
        left_lane = left_lane[0]
        right_lane = right_lane[0]
        steering_angle = (left_lane[0][0] - right_lane[0][0]) / frame.shape[1]
    else:
        steering_angle = 0
    cv2.imshow("frame", frame)
    if cv2.waitKey(24) & 0xFF == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()
# Display the image

