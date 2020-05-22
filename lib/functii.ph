import cv2

width, height = 600, 600
cap = cv2.VideoCapture(0)

#cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)


def verificcamera():
    if cap.isOpened():
        #camera ok
	#meniu_lbl.configure(text="Camera OK")
        return True
    else:
        #meniu_lbl.configure(text="Lipsa camera")
        return False