#BOARD := esp32:esp32:esp32s3:CDCOnBoot=cdc
BOARD := esp32:esp32:esp32
PORT ?= $(shell ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -1)
BAUD := 115200
UPLOAD_SPEED := 115200
SKETCH := .

.PHONY: all compile upload flash monitor clean ports boards

all: compile

compile:

	arduino-cli compile --fqbn $(BOARD) $(SKETCH)

upload:

	arduino-cli upload -p $(PORT) --fqbn $(BOARD):UploadSpeed=$(UPLOAD_SPEED) $(SKETCH)

flash: compile upload

monitor:

	arduino-cli monitor -p $(PORT) -c baudrate=$(BAUD)

clean:

	rm -rf build

ports:

	arduino-cli board list

boards:

	arduino-cli board listall | grep -i esp32
