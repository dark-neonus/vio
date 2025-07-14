# DL-VIO on RPi4

## Real-time 6-DoF Visual-Inertial Odometry using Deep Learning on Raspberry Pi 4

This project aims to develop and deploy a real-time 6-Degrees-of-Freedom (6-DoF) Visual-Inertial Odometry (VIO) system leveraging deep learning techniques (PyTorch) on a Raspberry Pi 4 (4GB RAM). The system will integrate an autofocus camera and an MPU-6050 IMU to estimate the device's pose in real-time.

**Inspired by the DeepVO paper**, this project seeks to provide a robust and efficient solution for various applications where GPS or radio signals may be unavailable or unreliable.

## Possible Usages

* **FPV Drone Navigation:** Navigate FPV drones autonomously without reliance on GPS or external radio signals.
* **General Purpose VIO for Robotics:** Serve as a fundamental building block for more advanced VIO navigation systems for both flying and ground-based drones.

## Hardware

* **Raspberry Pi 4 (4GB RAM):** The target deployment platform.
* **Camera Module / USB Camera:** For visual input.
* **MPU-6050 IMU:** For inertial sensor data (accelerometer and gyroscope).

## Software & Frameworks

Development is conducted within a Docker container to ensure a consistent and reproducible environment.

* **Host OS:** Manages Docker and the overall workflow.
* **Docker Container (based on Ubuntu 24.04 LTS):**
    * **ROS 2 Jazzy:** (Targeting Ubuntu Noble 24.04) Used for data management, visualization, and integration of various components.
    * **Gazebo Harmonic:** A powerful robotics simulator used for generating synthetic data and testing the VIO system in a virtual environment.
* **Python:** The primary programming language.
* **PyTorch:** The deep learning framework.
    * **GPU:** Utilized on the desktop for efficient model training.
    * **ARM CPU:** The target for model deployment on the Raspberry Pi.
* **ROS2:** Facilitates synchronized data collection, visualization with RViz, and seamless integration of different modules.
* **Gazebo:** Provides a realistic simulation environment for robot and sensor data generation.

## Approach: End-to-End Deep Learning

The core of the VIO system is an end-to-end deep learning architecture, specifically a combination of Convolutional Neural Networks (CNNs) and Recurrent Neural Networks (RNNs).

* **CNN (Convolutional Neural Network):** Used for extracting robust visual features from input images.
* **RNN / LSTM / GRU:** Employed for sequence modeling and effectively fusing visual features with IMU data over time.

**Inputs:**
* **Images:** Consecutive frames (e.g., 256x256, 256x192, or 128x128 pixels), potentially stacked to capture temporal information.
* **IMU Data:** Accelerometer and gyroscope readings.

**Output:**
* **6-DoF Pose:** Consisting of 3D translation (X, Y, Z) and 3D orientation (Euler angles or Quaternions).

## Data

* **Simulated Data:** Primarily generated from Gazebo using ROS2 for synchronized collection. Domain randomization techniques will be applied to reduce the sim2real gap.
* **Real-world Datasets (Potential):** Fine-tuning on established datasets like EuRoC MAV or TUM VI.
* **Custom Real-world Data:** Collection of project-specific real-world data as needed.

## Training

Model training is performed on a powerful desktop machine, leveraging GPU acceleration within the Docker container. For computationally intensive tasks, Google Colab cloud resources may be utilized.

* **Transfer Learning:** Employing pre-trained CNN backbones to accelerate training and improve performance.
* **Loss Function:** Mean Squared Error (MSE) loss, with potential weighting for translation and orientation components to balance their contributions.
* **Optimizer:** Adam or AdamW.
* **Regularization:** Dropout layers to prevent overfitting.
* **Early Stopping:** To halt training when validation performance saturates.
* **Data Augmentation:** To increase the diversity of the training data and improve generalization.
* **Batch Normalization:** To stabilize and accelerate training.

## Optimization (for Raspberry Pi Deployment)

After training the model on the desktop, several optimization techniques will be applied to ensure efficient execution on the Raspberry Pi's limited resources.

* `torch.compile`: PyTorch's native compilation for performance improvement.
* TorchScript: Converting PyTorch models into a serializable and optimizable format.
* Quantization (Static / Quantization-Aware Training - QAT): Reducing model precision to decrease memory footprint and accelerate inference.
* ONNX Export with ONNX Runtime: Exporting the model to the Open Neural Network Exchange (ONNX) format for cross-platform deployment and efficient inference with ONNX Runtime.

## Deployment

* The optimized PyTorch model will run directly on the Raspberry Pi's CPU.

## Testing & Evaluation

The performance of the VIO system will be rigorously evaluated using standard metrics.

* **Absolute Trajectory Error (ATE) RMSE:** Measures the overall difference between the estimated and ground truth trajectories.
* **Relative Pose Error (RPE) RMSE:** Quantifies the local accuracy of the pose estimates over a specific time interval.
* **Visualization:** Using RViz (via ROS2 topics) to visually inspect the estimated trajectories and compare them with ground truth.

## Short project context
```
"**Project: DL-VIO on RPi4.** Goal: Real-time 6-DoF VIO using deep learning (PyTorch) on Raspberry Pi 4 (4GB RAM) with autofocus camera & MPU-6050 IMU. Possible uages: navigate FPV drone without GPS or radiosignal, base for more advanced general purpose VIO navigation for flying and ground drones **Inspired by DeepVO paper.** **Hardware:** RPi4, Camera Module/USB Cam, MPU-6050. **Software/Frameworks:** Development occurs inside a Docker container based on Ubuntu 24.04 LTS with ROS 2 Jazzy (targeting Ubuntu Noble 24.04) and Gazebo Harmonic for simulation and integration. Host OS runs Docker and manages workflow. Python, PyTorch (GPU on desktop; ARM CPU on Pi), **ROS2** (for data mgmt/viz/integration; Gazebo simulation), **Gazebo** (robotics simulator). **Approach:** End-to-end DL using RCNN (CNN for visual feature ext., RNN/LSTM/GRU for seq. modeling & visual-IMU fusion). Input: Images (e.g., 256x256, 256x192, or 128x128; stacked consecutive frames) + IMU data (accel/gyro). Output: 6-DoF Pose (translation & Euler/Quaternion orientation). **Data:** Primarily simulated from Gazebo (using ROS2 for synchronized collection, with domain randomization for sim2real gap reduction), potentially fine-tune on real-world datasets (EuRoC MAV/TUM VI), or collect custom real-world data. **Training:** On desktop GPU (inside Docker container); if computations are heavy, will use Google Colab cloud. Employs transfer learning (pre-trained CNN backbone), MSE loss (weighted for translation/orientation), Adam/AdamW optimizer, dropout, early stopping, data augmentation, batch normalization. **Optimization (for Pi deployment):** Post-training (on desktop) model optimizations like `torch.compile`, TorchScript, Quantization (static/QAT), and ONNX export with ONNX Runtime are planned. **Deployment:** Optimized PyTorch model runs on Raspberry Pi's CPU. **Testing/Evaluation:** Metrics include ATE RMSE & RPE RMSE; visualization using RViz (via ROS2 topics)."
```
