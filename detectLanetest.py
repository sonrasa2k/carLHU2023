import math
import time

import cv2
import numpy as np
cap = cv2.VideoCapture("output.mp4")

def grayscale(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
def canny(img, low_threshold, high_threshold):
    return cv2.Canny(img, low_threshold, high_threshold)
def gaussian_blur(img, kernel_size):
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    if len(img.shape) > 2:
        channel_count = img.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image
def process_image(image):
    image = cv2.resize(image,(640,480))
    blurred = gaussian_blur(canny(image, 100, 150), 7)
    rows, cols = image.shape[:2]
    bottom_left = [int(cols * 0.2), int(rows * 0.95)]
    top_left = [int(cols * 0.25), int(rows * 0.45)]
    bottom_right = [int(cols * 0.95), int(rows * 0.95)]
    top_right = [int(cols * 0.75), int(rows * 0.45)]
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    copied = np.copy(blurred)
    # cv2.line(copied, tuple(bottom_left), tuple(bottom_right), (255, 0, 0), 5)
    # cv2.line(copied, tuple(bottom_right), tuple(top_right), (255, 0, 0), 5)
    # cv2.line(copied, tuple(top_left), tuple(bottom_left), (255, 0, 0), 5)
    # cv2.line(copied, tuple(top_left), tuple(top_right), (255, 0, 0), 5)
    interested = region_of_interest(copied,vertices)
    where_left = np.where(interested[:, :interested.shape[1] // 2] > 80)
    where_right = np.where(interested[:, interested.shape[1] // 2:] > 80)
    try:
        dot_top_left = [np.max(where_left[1]), np.min(where_left[0])]
        pseudo_bottom_left = [np.min(where_left[1]), np.max(where_left[0])]
        m_left, c_left = np.polyfit([pseudo_bottom_left[1], dot_top_left[1]],
                                    [pseudo_bottom_left[0], dot_top_left[0]], 1)
        dot_bottom_left = [int(image.shape[0] * m_left + c_left), image.shape[0]]
        dot_top_right = [image.shape[1] // 2 + np.min(where_right[1]), np.min(where_right[0])]
        pseudo_bottom_right = [image.shape[1] // 2 + np.max(where_right[1]), np.max(where_right[0])]
        m_right, c_right = np.polyfit([pseudo_bottom_right[1], dot_top_right[1]],
                                      [pseudo_bottom_right[0], dot_top_right[0]], 1)
        dot_bottom_right = [int(image.shape[0] * m_right + c_right), image.shape[0]]
        middle_top = [int(dot_top_left[0] + ((dot_top_right[0] - dot_top_left[0]) / 2)), dot_top_left[1]]
        middle_bottom = [int(dot_bottom_left[0] + ((dot_bottom_right[0] - dot_bottom_left[0]) / 2)), image.shape[0]]
        steering_val = middle_bottom[0] - image.shape[1] // 2
        CONST_ANGLE= 1/21
        if steering_val > 0:
            steering_val = np.abs(steering_val)/(image.shape[1]*0.95)
            steering_val = steering_val/CONST_ANGLE
        else:
            steering_val = np.abs(steering_val)/(image.shape[1]*0.95)
            steering_val = -steering_val/CONST_ANGLE
    except:
        steering_val = 0
    return image,blurred,interested,steering_val

start = time.time()
while True:
    ret, frame = cap.read()
    frame,blu,imr,steering_val = process_image(image=frame)
    print(steering_val)
    print(time.time()-start)
    cv2.imshow("frame",frame)
    cv2.imshow("blue",blu)
    cv2.imshow("process",imr)
    if cv2.waitKey(24) & 0xFF == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()
