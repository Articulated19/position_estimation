import cv2
import numpy as np

cap = cv2.VideoCapture(0)

# range of yellow color in HSV
lower_yellow = np.array([20,120,120])
upper_yellow = np.array([40,255,255])

font = cv2.FONT_HERSHEY_SIMPLEX
pixel_thresh = 9000

while (cap.isOpened()):
    ret, frame = cap.read()
    if (ret):
        # Convert from rgb to hsv for easy filtering
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # inRange to only capture values between lower/upper yellow
        mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
        # Bitwise AND of mask and webcam feed
        res = cv2.bitwise_and(frame, frame, mask=mask)
        # Count yellow pixels found
        nzCount = cv2.countNonZero(mask)
        
        cv2.putText(frame,'Yellow pixel count: ',(20,30), font, 1,(255,0,0),1,cv2.LINE_AA)
        cv2.putText(frame, str(nzCount),(20,80), font, 2,(255,0,0),1,cv2.LINE_AA)

        if nzCount > pixel_thresh:
            cv2.putText(frame,'Truck at turn! ',(100,200), font, 2,(0,0,255),8,cv2.LINE_AA)
        cv2.imshow('hsv_img', hsv_img)
        cv2.imshow('mask', mask)
        cv2.imshow('Filtered color only', res)
        cv2.imshow('Webcam',frame)
    
    if cv2.waitKey(1) == 13: #Enter
        break
        
cap.release()
cv2.destroyAllWindows()