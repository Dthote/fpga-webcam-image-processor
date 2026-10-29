# FPGA-Based Webcam Image Processor

An FPGA-based digital image processing system that combines Python, OpenCV, UART communication, and Verilog HDL to process webcam images using the Digilent Arty A7-35T FPGA board.

## Overview

This project captures images from a webcam, preprocesses them on a computer, and transfers pixel data to an FPGA for hardware-based image processing. The processed image is sent back to the computer and displayed using Python and OpenCV.

## Features

- Webcam image capture using Python and OpenCV
- Image resizing to 160 × 120 pixels
- RGB565 pixel-format conversion
- UART communication between the computer and FPGA
- Colour-negative image processing
- Automatic white-balance processing
- Automatic-exposure processing
- Switch-based selection of image-processing functions

## Technologies Used

- **Hardware:** Digilent Arty A7-35T FPGA
- **Hardware Description Language:** Verilog HDL
- **FPGA Development:** AMD Xilinx Vivado
- **Programming Language:** Python
- **Image Processing:** OpenCV and NumPy
- **Serial Communication:** PySerial and UART

## Architecture and Workflow

1. The webcam captures an image.
2. Python and OpenCV resize the image to 160 × 120 pixels.
3. The image is converted to RGB565 format.
4. UART transfers the pixel data to the FPGA.
5. The FPGA performs the selected image-processing operation.
6. The processed data is returned to the computer.
7. Python reconstructs and displays the output image.

## Hardware Requirements

- Digilent Arty A7-35T FPGA board
- Laptop or desktop computer
- Webcam
- USB cable for FPGA programming and communication

## Software Requirements

- Python 3
- AMD Xilinx Vivado
- OpenCV
- NumPy
- PySerial

## Setup and Usage

1. Install Python and AMD Xilinx Vivado.
2. Install the Python dependencies:

   ```bash
   pip install opencv-python numpy pyserial
   ```

3. Clone or download this repository.
4. Open the Verilog project in Vivado.
5. Configure the FPGA constraints file for the Arty A7-35T board.
6. Synthesize and implement the design, then generate and program the FPGA bitstream.
7. Connect the board to the computer using USB.
8. Update the Python script with the correct serial port and baud rate.
9. Run the Python application and test the supported image-processing functions.

## Visual Demonstration

Add genuine screenshots showing:
- Original webcam image
- Colour-negative output
- Automatic white-balance output
- Automatic-exposure output

You can also include a short demonstration video showing the complete image-transfer and processing workflow.

## Known Limitations

- Image processing is limited to the implemented functions.
- The image resolution is 160 × 120 pixels.
- UART transfer speed and reliability affect overall processing performance.
- Further work could explore higher resolutions, faster interfaces, and additional image-processing algorithms.

## Future Improvements

- Improve image-transfer throughput.
- Support higher image resolutions.
- Add more image-enhancement algorithms.
- Measure processing latency and resource utilization.

## Author

Add your name, GitHub profile, and project details here.
