TARGET ?= esp32s3
PORT   ?= /dev/tty.usbmodem*  # macOS; Linux: /dev/ttyUSB0 or /dev/ttyACM0

.PHONY: set-target build flash monitor full clean

set-target:
	idf.py set-target $(TARGET)

build:
	idf.py build

flash:
	idf.py -p $(PORT) flash

monitor:
	idf.py -p $(PORT) monitor

full: set-target build flash monitor
clean:
	idf.py clean
