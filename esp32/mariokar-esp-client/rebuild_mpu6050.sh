#!/bin/bash
# Script to force rebuild with MPU6050 sensor

set -e

echo "======================================"
echo "Rebuilding with MPU6050 sensor"
echo "======================================"

# Verify sensor selection
echo ""
echo "Checking sensor selection in sensor.h..."
grep "IMU_SENSOR_TYPE" main/sensor.h
echo ""

# Clean everything
echo "Cleaning all build artifacts..."
rm -rf build/
rm -rf managed_components/
rm -rf .cache/
rm -rf sdkconfig.old
echo "✓ Clean complete"

# Show what will be compiled
echo ""
echo "Conditional compilation will use:"
echo "  - sensor_mpu6050.c (MPU6050 I2C driver)"
echo "  - sensor.c will be EXCLUDED (ICM20948 code won't compile)"
echo ""

# Source ESP-IDF if not already sourced
if ! command -v idf.py &> /dev/null; then
    echo "Sourcing ESP-IDF environment..."
    source /opt/esp/idf/export.sh || source $HOME/esp/esp-idf/export.sh
fi

# Reconfigure to download dependencies
echo "Reconfiguring project (downloading MPU6050 component)..."
idf.py reconfigure

# Build
echo ""
echo "Building project..."
idf.py build

echo ""
echo "======================================"
echo "✓ Build complete!"
echo "======================================"
echo ""
echo "To flash and monitor:"
echo "  idf.py flash monitor"
echo ""
echo "Expected output:"
echo "  I (xxx) MPU6050: Initializing MPU6050 IMU"
echo "  I (xxx) MPU6050: MPU6050 initialized successfully (I2C mode)"
echo ""

