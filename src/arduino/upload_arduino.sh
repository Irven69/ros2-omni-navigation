#!/bin/bash
# filepath: /home/pi/ros2_ws/src/arduino/upload_arduino.sh

ARDUINO_DIR="$HOME/ros2_ws/src/arduino"
SRC_DIR="$ARDUINO_DIR/src"
BOARD_FQBN="arduino:avr:mega"
PORT="/dev/ttyACM0"

# Function to show usage
usage() {
    echo "=== Arduino Upload Script ==="
    echo "Usage: $0 <sketch_name>"
    echo ""
    echo "Available sketches:"
    ls -1 "$SRC_DIR"/*.ino 2>/dev/null | xargs -n1 basename | sed 's/\.ino$//' || echo "  No .ino files found in $SRC_DIR"
    echo ""
    echo "Example:"
    echo "  $0 motorControllerClient"
    echo "  $0 ArduinoPIDTuning"
    exit 1
}

# Check if argument provided
if [ $# -eq 0 ]; then
    echo "ERROR: No sketch name provided"
    echo ""
    usage
fi

SKETCH_NAME="$1"
SOURCE_FILE="$SRC_DIR/${SKETCH_NAME}.ino"

# Check if source file exists
if [ ! -f "$SOURCE_FILE" ]; then
    echo "ERROR: Sketch not found: $SOURCE_FILE"
    echo ""
    usage
fi

# Create temporary folder with sketch name
TEMP_DIR="$ARDUINO_DIR/$SKETCH_NAME"

echo "=== Arduino Upload Script ==="
echo "Sketch: $SKETCH_NAME"
echo "Source: $SOURCE_FILE"
echo "Temp folder: $TEMP_DIR"
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

# Create temp directory
echo "Creating temporary folder..."
mkdir -p "$TEMP_DIR"

# Copy sketch to temp folder
cp "$SOURCE_FILE" "$TEMP_DIR/${SKETCH_NAME}.ino"

if [ $? -ne 0 ]; then
    echo "ERROR: Failed to copy sketch to temp folder"
    rm -rf "$TEMP_DIR"
    exit 1
fi

# Compile
echo ""
echo "Compiling Arduino sketch..."
arduino-cli compile --fqbn $BOARD_FQBN "$TEMP_DIR"

if [ $? -eq 0 ]; then
    # Upload
    echo ""
    echo "Uploading sketch to Arduino Mega on $PORT..."
    arduino-cli upload -p $PORT --fqbn $BOARD_FQBN "$TEMP_DIR"
    
    if [ $? -eq 0 ]; then
        echo ""
        echo "✓ Upload successful!"
        echo ""
        echo "Cleaning up temporary folder..."
        rm -rf "$TEMP_DIR"
        echo "✓ Cleanup complete"
        echo ""
        echo "To monitor serial output:"
        echo "  arduino-cli monitor -p $PORT -c baudrate=115200"
        exit 0
    else
        echo ""
        echo "✗ Upload failed!"
        echo "Cleaning up temporary folder..."
        rm -rf "$TEMP_DIR"
        exit 1
    fi
else
    echo ""
    echo "✗ Compilation failed!"
    echo "Cleaning up temporary folder..."
    rm -rf "$TEMP_DIR"
    exit 1
fi