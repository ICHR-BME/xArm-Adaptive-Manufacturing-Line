# xArm Adaptive Manufacturing Line

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![xArm](https://img.shields.io/badge/Hardware-uFactory_xArm_Lite_6-orange.svg)](https://www.ufactory.cc/)
[![YOLOv8](https://img.shields.io/badge/Vision-YOLOv8-yellow.svg)](https://ultralytics.com/)

## Overview
This repository contains the core software for an **automatic manufacturing line with high adaptability**. Moving away from traditional, rigid hardcoded positions, this system leverages advanced computer vision and machine learning to dynamically adjust to changing environments and part locations in real-time. 

Developed at **Tecnológico de Monterrey** as part of a Mechatronics Engineering initiative, this project serves as a practical implementation of flexible manufacturing concepts, aiming for presentation at the **Conference on Learning Factories (CLF) 2026**.

## Key Features
* **High Adaptability & Dynamic Calibration:** Uses scikit-learn's Linear Regression to autonomously map pixel coordinates to real-world robot coordinates (mm) with sub-millimeter precision.
* **Multi-View Vision System:**
    * **Top Camera (XY Plane):** Contour detection and object localization.
    * **Side Camera (Z Plane):** Height calculation using a calibrated depth formula to safely approach parts of varying sizes.
* **Deep Learning Integration:** * Object detection powered by a custom-trained **YOLOv8** model (`best.pt`).
    * Monocular depth estimation testing using the **MiDaS / Depth-Anything** Transformer model.
* **Safe Trajectory Planning:** Built-in "Retraction Mode" and safe-Z travel algorithms to avoid collisions with surrounding equipment (e.g., Bambu Lab A1 3D printers).

## Hardware Requirements
* **Robot Arm:** uFactory xArm Lite 6
* **Vision:** 2x Standard USB Webcams (Top & Side views)
* **Environment:** Conveyor belt or automated manufacturing cell (e.g., 3D printer bed unloading)

## Repository Structure

```text
xArm-Adaptive-Manufacturing/
├── calibration/                # Auto-calibration scripts for XY and Z axes
├── vision/                     # Isolated vision tools (MiDaS, OpenCV contours, ROI setup)
├── data/                       # Stored CSV coordinate mapping data
├── models/                     # Custom trained AI models (.pt)
├── main_adaptive_yolo.py       # Core executable: YOLO-driven adaptive pick & place
├── main_adaptive_midas.py      # Core executable: Transformer-driven depth estimation
├── requirements.txt            # Python dependencies
└── README.md
