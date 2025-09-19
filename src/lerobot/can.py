import cv2

# Camera
capture = cv2.VideoCapture(2)
capture.set(cv2.CAP_PROP_FRAME_WIDTH,640)
capture.set(cv2.CAP_PROP_FRAME_WIDTH,480)

while cv2.waitKey(33) < 0 :
    # 33ms마다 반복문을 실행
    ret, frame = capture.read() # ret:camera 이상 여부, frame : 현재 시점의 frame
    cv2.imshow("VideoFrame", frame)
    print(cv2.waitKey(33))

capture.release()
cv2.destroyAllWindows()
