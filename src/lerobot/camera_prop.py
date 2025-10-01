import cv2, os, time
from rich import print
from rich.live import Live
from rich.console import Console
from rich.layout import Layout
from rich.table import Table
import getchlib

waitkey = getchlib.HotKeyListener(catch=True)

console = Console()
cap = cv2.VideoCapture(0)


cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)	# 원래 3이였음.
cap.set(cv2.CAP_PROP_AUTO_WB, 0)
cap.set(cv2.CAP_PROP_EXPOSURE, 30.0)
# cap.set(cv2.CAP_PROP_GAIN, 255.0)
cap.set(cv2.CAP_PROP_TEMPERATURE, 5600)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)


layout = Layout()
fixed_table = Table(title="FIXED PROPs")
config_table = Table(title="User PROPs")

layout.split_column(
    Layout(name="upper"),
    Layout(name="lower")
)

layout["upper"].update(fixed_table)
layout["lower"].update(config_table)

fixed_table.add_column("Name")
fixed_table.add_column("Value")
fixed_table.add_row("CAP_PROP_GUID", str(cap.get(cv2.CAP_PROP_GUID)))
fixed_table.add_row("CAP_PROP_AUTO_EXPOSURE", str(cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)))
fixed_table.add_row("CAP_PROP_AUTO_WB", str(cap.get(cv2.CAP_PROP_AUTO_WB)))
fixed_table.add_row("CAP_PROP_AUTOFOCUS", str(cap.get(cv2.CAP_PROP_AUTOFOCUS)))
fixed_table.add_row("CAP_PROP_MODE", str(cap.get(cv2.CAP_PROP_MODE)))
fixed_table.add_row("CAP_PROP_FPS", str(cap.get(cv2.CAP_PROP_FPS)))

config_table.add_column("Name")
config_table.add_column("Value")
config_table.add_row("Time", time.asctime())
config_table.add_row("CAP_PROP_EXPOSURE", str(cap.get(cv2.CAP_PROP_EXPOSURE)))
config_table.add_row("CAP_PROP_TEMPERATURE", str(cap.get(cv2.CAP_PROP_TEMPERATURE)))
config_table.add_row("CAP_PROP_GAIN", str(cap.get(cv2.CAP_PROP_GAIN)))
config_table.add_row("CAP_PROP_FOCUS", str(cap.get(cv2.CAP_PROP_FOCUS)))
config_table.add_row("CAP_PROP_ZOOM", str(cap.get(cv2.CAP_PROP_ZOOM)))


exposure = cap.get(cv2.CAP_PROP_EXPOSURE)
gain = cap.get(cv2.CAP_PROP_GAIN)
focus = cap.get(cv2.CAP_PROP_FOCUS)
zoom = cap.get(cv2.CAP_PROP_ZOOM)


console.clear()
print(layout)


while True:
	config_table = Table(title="User PROPs")
	config_table.add_column("Name")
	config_table.add_column("Value")
	config_table.add_row("Time", time.asctime())
	config_table.add_row("CAP_PROP_EXPOSURE", str(cap.get(cv2.CAP_PROP_EXPOSURE)))
	config_table.add_row("CAP_PROP_GAIN", str(cap.get(cv2.CAP_PROP_GAIN)))
	config_table.add_row("CAP_PROP_TEMPERATURE", str(cap.get(cv2.CAP_PROP_TEMPERATURE)))
	config_table.add_row("CAP_PROP_FOCUS", str(cap.get(cv2.CAP_PROP_FOCUS)))
	config_table.add_row("CAP_PROP_ZOOM", str(cap.get(cv2.CAP_PROP_ZOOM)))
	layout["lower"].update(config_table)
	print(layout)
	
	ret, frame = cap.read()
	cv2.imshow('test', frame)
	key = cv2.waitKey(1)
	key = getchlib.getkey(False)
	match key:
		case 'q':
			exposure += 1
			cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
			exposure = cap.get(cv2.CAP_PROP_EXPOSURE)
		case 'a':
			exposure -= 1
			cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
			exposure = cap.get(cv2.CAP_PROP_EXPOSURE)
		case 'w':
			gain += 1
			cap.set(cv2.CAP_PROP_GAIN, gain)
			gain = cap.get(cv2.CAP_PROP_GAIN)
		case 's':
			gain -= 1
			cap.set(cv2.CAP_PROP_GAIN, gain)
			gain = cap.get(cv2.CAP_PROP_GAIN)
		case 'e':
			focus += 0.01
			cap.set(cv2.CAP_PROP_FOCUS, focus)
			focus = cap.get(cv2.CAP_PROP_FOCUS)
		case 'd':
			focus -= 0.01
			cap.set(cv2.CAP_PROP_FOCUS, focus)
			focus = cap.get(cv2.CAP_PROP_FOCUS)
		case 'r':
			zoom += 1
			cap.set(cv2.CAP_PROP_ZOOM, zoom)
			zoom = cap.get(cv2.CAP_PROP_ZOOM)
		case 'f':
			zoom -= 1
			cap.set(cv2.CAP_PROP_ZOOM, zoom)
			zoom = cap.get(cv2.CAP_PROP_ZOOM)
