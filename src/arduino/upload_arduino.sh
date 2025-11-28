#!/bin/bash
# filepath: /home/pi/ros2_ws/src/arduino/upload_arduino.sh

SKETCH_DIR="$HOME/ros2_ws/src/arduino/motorControllerClient"
BOARD_FQBN="arduino:avr:mega"
PORT="/dev/ttyACM0"

echo "=== Arduino Upload Script ==="
echo "Sketch: $SKETCH_DIR"
echo "Board: $BOARD_FQBN"
echo "Port: $PORT"
echo ""

# Check if port exists
if [ ! -e "$PORT" ]; then
    echo "ERROR: Arduino not found on $PORT"
    echo "Available ports:"
    ls -la /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "No serial devices found"
    exit 1
fi

echo "Compiling Arduino sketch..."
arduino-cli compile --fqbn $BOARD_FQBN $SKETCH_DIR

if [ $? -eq 0 ]; then
    echo ""
    echo "Uploading sketch to Arduino Mega on $PORT..."
    arduino-cli upload -p $PORT --fqbn $BOARD_FQBN $SKETCH_DIR
    
    if [ $? -eq 0 ]; then
        echo ""
        echo "Upload successful!"
        echo ""
        echo "To monitor serial output:"
        echo "  arduino-cli monitor -p $PORT -c baudrate=115200"
        exit 0
    else
        echo ""
        echo "Upload failed!"
        exit 1
    fi
else
    echo ""
    echo "Compilation failed!"
    exit 1
fi