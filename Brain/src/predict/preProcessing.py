import cv2
import numpy as np
def make_coordinate(img,line_parameters):
    slope,intercept = line_parameters
    y1=img.shape[0]
    y2=int(y1*(3/5))
    x1=int((y1-intercept)/slope)
    x2=int((y2-intercept)/slope)
    return np.array([x1,y1,x2,y2])
def average_slope_intercept(img,lines):
    left_fit=[]
    right_fit=[]
    for line in lines:
        x1,y1,x2,y2 = line.reshape(4)
        # returns slope 1st and y-intercept 2nd
        parameters = np.polyfit((x1,x2),(y1,y2),1)#degree 1
        slope=parameters[0]
        intercept=parameters[1]
        if slope < 0:
            left_fit.append((slope,intercept))
        else:
            right_fit.append((slope,intercept))
    left_fit_average=np.average(left_fit,axis=0)
    right_fit_average=np.average(right_fit,axis=0)
    left_line= make_coordinate(img, left_fit_average)
    right_line= make_coordinate(img, right_fit_average)
    return np.array([left_line, right_line])
def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    match_mask_color = 255 #only one color because it is a gray image
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def draw_the_lines(img,lines):
    img = np.copy(img)
    blank_img = np.zeros((img.shape[0],img.shape[1],3), dtype=np.uint8)
    for line in lines:
        for i in range(len(line)):
          if line[i] < 0:
            line[i] = 0
          if line[i] > 680:
              line[i] = 680
        x1 = line[0]
        y1 = line[1]
        x2 = line[2]
        y2 = line[3]
        cv2.line(blank_img,(x1,y1),(x2,y2),(0,255,0),thickness=4)
    img = cv2.addWeighted(img,0.8,blank_img,1,0.0)
    return img