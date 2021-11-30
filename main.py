import HandTrackingModule as htm
import cv2
import numpy as np
import mediapipe as mp
import time
import math
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume


######################
wcam, hcam = 640, 480
######################

cap = cv2.VideoCapture(0)
cap.set(3, wcam)
cap.set(4, hcam)

devices = AudioUtilities.GetSpeakers()
interface = devices.Activate(
    IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
volume = cast(interface, POINTER(IAudioEndpointVolume))

volrange = volume.GetVolumeRange()
minvol = volrange[0]
maxvol = volrange[1]


detector = htm.handDetector()

p_time = 0
vol = 0
volBar = 400
volPer = 0
while True:
    success, img = cap.read()
    # img = cv2.resize(img, (0, 0), fx=0.6, fy=0.6)
    img = detector.findHands(img)
    lmList = detector.findPosition(img, draw=False)
    if len(lmList) != 0:
        # print(lmList)
        thumb = lmList[4]
        index = lmList[8]
        # print(thumb, index)
        tx, ty = thumb[1], thumb[2]
        ix, iy = index[1], index[2]
        cx, cy = (tx + ix) // 2, (ty + iy) // 2
        cv2.circle(img, (tx, ty), 10, (255, 0, 255), cv2.FILLED)
        cv2.circle(img, (ix, iy), 10, (255, 0, 255), cv2.FILLED)
        cv2.line(img, (tx, ty), (ix, iy), (255, 0, 255), thickness=3)
        cv2.circle(img, (cx, cy), 10, (255, 0, 255), cv2.FILLED)

        length = math.hypot(ix - tx, iy - ty)
        # print(length)
        vol = np.interp(length, [50, 200], [minvol, maxvol])
        volBar = np.interp(length, [50, 200], [400, 150])
        volPer = np.interp(length, [50, 200], [0, 100])

        # print(vol, length)
        volume.SetMasterVolumeLevel(vol, None)

        cv2.rectangle(img, (50, 150), (85, 400), (255, 0, 0), 3)
        cv2.rectangle(img, (50, int(volBar)), (85, 400), (255, 0, 0), cv2.FILLED)
        cv2.putText(img, f'{int(volPer)} %', (40, 450), cv2.FONT_HERSHEY_COMPLEX,
                    1, (255, 0, 0), 3)

        if length < 50:
            cv2.circle(img, (cx, cy), 10, (0, 255, 0), cv2.FILLED)

    c_time = time.time()
    fps = 1 / (c_time - p_time)
    p_time = c_time
    
    cv2.putText(img, str(int(fps)), (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 3)
    cv2.imshow('Frame', img)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

volume.SetMasterVolumeLevel(-10, None)
