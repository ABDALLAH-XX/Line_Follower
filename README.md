# E-Puck Line Follower (Webots & OpenCV)

A high-speed line follower simulation using the e-puck robot in Webots. This project implements a **PID controller** and leverages **OpenCV** for advanced image processing to detect the track under various conditions.

## 🚀 Features
- **OpenCV Integration:** Real-time camera processing for line detection.
- **PID Control:** Smooth steering and speed management (tuned for 5.8 rad/s).
- **Custom Worlds:** Includes specific maps (`race1`, `map1`, etc.) for testing performance.

## 📁 Project Structure
- `controllers/EPuckLineFollower/`: Core C++ logic and Makefile.
- `worlds/`: Simulation environments and track textures.
- `protos/` & `plugins/`: Custom robot definitions and physics.

## 🛠️ Setup
1. **Webots:** Ensure you have Webots R2023b or later installed.
2. **OpenCV:** You must have OpenCV installed on your system.
   - *Linux:* `sudo apt-get install libopencv-dev`
3. **Compilation:**
   - Open the controller folder in a terminal.
   - Run `make` to compile the C++ code.

## 🎮 How to Run
1. Open Webots.
2. Load the world file: `worlds/e-puck_line_follower.wbt`.
3. Press the **Play** button to start the simulation.