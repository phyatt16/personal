import numpy as np
import cv2

cap = cv2.VideoCapture(1)

while True:
    # Capture frame by frame
    ret, frame = cap.read()

    if ret==True:
    
        # Our operations on the frame come here
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        height = frame.shape[0]
        width = frame.shape[1]
        frame = cv2.resize(frame,(width*2,height*2))

        # Display the resulting frame
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
            break

cap.release()
cv2.destroyAllWindows()

