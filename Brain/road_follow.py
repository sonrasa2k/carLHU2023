import cv2 
import numpy as np

def middle_lane_point(lines):
    y_const = 350
    x_const = 360
    x_right_list = []
    x_left_list = []

    x_right = 720
    x_left = 0

    for line in lines:
        x1, y1, x2, y2 = line[0]
        x_check = (x1+x2)/2
        y_check = (y1+y2)/2
        check_x = x_check - x_const
        if max([y1,y2])>300 :
            if check_x>0 and x_check<x_right :
                _x = int((x_check + x_right)/2)
                x_right_list.append(_x)
                x_right = np.average(x_right_list)

            elif check_x<0 and x_check>x_left:
                _x = int((x_check + x_left)/2)
                print(_x)
                x_left_list.append(_x)
                x_left = np.average(x_left_list)
            # error_y = abs(y1-y_check)
    if len(x_left_list) == 0:
        x_left = -200
    if len(x_right_list) == 0:
        x_right = 920
    x= int((x_right+x_left)/2)
    print(x_right_list)
    print(x_left_list)
    print("///")
    return (x, y_const)

def lane_tracking(edges):
    lines_list =[]
    lines = cv2.HoughLinesP(
                edges, # Input edge image
                1, # Distance resolution in pixels
                np.pi/180, # Angle resolution in radians
                threshold=30, # Min number of votes for valid line
                minLineLength=10, # Min allowed length of line
                maxLineGap=4# Max allowed gap between line for joining them
                )
    # print(lines)
    for points in lines:
        # Extracted points nested in the list
        x1,y1,x2,y2=points[0]
        lines_list.append([x1,y1,x2,y2])
    (x,y) = middle_lane_point(lines)
    return lines_list, (x,y)
def process_image(image):
    src_img = cv2.resize(image,(720,480))
    gray_img = cv2.cvtColor(src_img, cv2.COLOR_BGR2GRAY)
    ksize = (5, 5)
    blur_img = cv2.blur(gray_img, ksize, cv2.BORDER_DEFAULT) 
    edges = cv2.Canny(blur_img,190,230,None, 3)
    return edges
if __name__ == "__main__":
    cap = cv2.VideoCapture("./test_video.mp4")
    pre_x = 0
    while True:
        _,src_img = cap.read() 
        src_img = cv2.resize(src_img,(720,480))
        edges = process_image(src_img)
        image = src_img
        # try:
        lines ,(x,y) = lane_tracking(edges)
        if abs(x-pre_x) < 20:
            x = pre_x
        pre_x = x
        for line in lines:
            x1,y1,x2,y2=line
            # Draw the lines joing the points
            # On the original image
            cv2.line(src_img,(x1,y1),(x2,y2),(0,255,0),2)
        image = cv2.circle(src_img, (x,y), radius=1, color=(0, 0, 255), thickness=4)
        image = cv2.circle(src_img, (360,350), radius=1, color=(0, 255, 0), thickness=4)
        cv2.imshow("black white image",edges)
        # except:
        #     print("errror")
        cv2.imshow("Image with lines",image)
        if cv2.waitKey(10) == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()