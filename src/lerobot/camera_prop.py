import cv2

cap = cv2.VideoCapture(6)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)	# 원래 3이였음.
cap.set(cv2.CAP_PROP_AUTO_WB, 0)
cap.set(cv2.CAP_PROP_EXPOSURE, 30.0)
# cap.set(cv2.CAP_PROP_GAIN, 255.0)
cap.set(cv2.CAP_PROP_TEMPERATURE, 5600)

exposure = cap.get(cv2.CAP_PROP_EXPOSURE)
gain = cap.get(cv2.CAP_PROP_GAIN)

while True:
	print(
		f"{cap.get(cv2.CAP_PROP_EXPOSURE)} | \
		{cap.get(cv2.CAP_PROP_GAIN)} |\
		{cap.get(cv2.CAP_PROP_TEMPERATURE)} |\
		{cap.get(cv2.CAP_PROP_FPS)} \
		{cap.get(cv2.CAP_PROP_MODE)} \
		{cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)} \
		"
	)
	ret, frame = cap.read()
	cv2.imshow('test', frame)
	key = cv2.waitKey(1)
	match key:
		case 119: # w
			gain += 1
			cap.set(cv2.CAP_PROP_GAIN, gain)
			gain = cap.get(cv2.CAP_PROP_GAIN)
		case 115: # s
			gain -= 1
			cap.set(cv2.CAP_PROP_GAIN, gain)
			gain = cap.get(cv2.CAP_PROP_GAIN)
		case 113: # q
			exposure += 1
			cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
			exposure = cap.get(cv2.CAP_PROP_EXPOSURE)
		case 97: # a
			exposure -= 1
			cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
			exposure = cap.get(cv2.CAP_PROP_EXPOSURE)