import cv2
import img_processing as ip

CAM_INDEX = 1
capture = cv2.VideoCapture(CAM_INDEX)

if capture.isOpened():
    capture_val, frame = capture.read()
else:
    capture_val = False
    print("Cannot open the camera of index " + str(CAM_INDEX) + ".")

previous_cnrs, previous_intxns, history_bk, temp_bk = ([],)*4
fr_cnt = 0

while capture_val:
    cropped, previous_cnrs, previous_intxns, new_bks = ip.imgProcessing(
        frame, previous_cnrs, previous_intxns, history_bk)

    history_bk, temp_bk, fr_cnt = ip.noiseFiltering(
        history_bk, new_bks, temp_bk, fr_cnt)

    # Calibrate constraints.
    key = cv2.waitKey(3)
    if (key == 'a'):
        ip.areaConstraintsCalibration(frame)
    elif (key == 'c'):
        ip.colorConstraintsCalibration(cropped, previous_intxns)

cv2.destroyAllWindows()
