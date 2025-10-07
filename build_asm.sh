#!/bin/bash
# build_asm.sh

echo "Building assembly optimizations..."

# Check for NASM
if ! command -v nasm &> /dev/null; then
    echo "NASM not found. Installing..."
    # Ubuntu/Debian
    sudo apt-get install nasm
    # macOS
    # brew install nasm
fi

# Assemble the ASM files
nasm -f elf64 -o flight_control_asm.o flight_control_asm.asm

# Build the Python extension
python setup.py build_ext --inplace

echo "Build complete!"
