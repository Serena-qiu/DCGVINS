# Tightly-Coupled Double-Difference GNSS Constrained Collaborative Visual-Inertial Navigation for Multi-Agent Systems

<p align="center">
  <a href="https://youtu.be/2zWdf0J1EU0"><img src="https://img.shields.io/badge/Video-YouTube-red?logo=youtube" alt="Video"></a>
  <a href="#"><img src="https://img.shields.io/badge/Paper-IEEE-blue" alt="Paper"></a>
  <a href="#license"><img src="https://img.shields.io/badge/License-GPL--3.0-green" alt="License"></a>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/972d752d-63a7-4f92-9538-d73206b1beeb" width="90%" alt="DC-GVINS System Overview">
</p>

**DC-GVINS** is a fully distributed, tightly coupled framework that integrates **infrastructure-free double-difference (DD) GNSS constraints** into visual-inertial state estimation for multi-agent systems. By exploiting the geometric co-observation of satellites between agents, DC-GVINS eliminates common-mode atmospheric and clock errors without requiring static base stations, achieving simultaneous improvement in both absolute and relative positioning accuracy.

---

## Key Features

- **Infrastructure-Free DD-GNSS**: No base station required. Collocated agents tracking common satellites naturally provide the geometric configuration for double-difference processing.
- **Tightly Coupled Factor Graph**: DD pseudorange constraints are formulated as factors within a unified GVINS optimization framework alongside IMU, visual, and single-point GNSS factors.
- **Distributed Architecture**: Each agent maintains an independent factor graph. Only minimal data (~2.5 KB/s per agent pair) is exchanged via peer-to-peer communication.
- **Quality-Aware Optimization**: SNR and harmonic-mean elevation-based covariance scaling, dynamic weight decay for communication latency, and MAD-based outlier rejection.
- **Graceful Degradation**: Three-tier fallback strategy (DC-GVINS → GVINS → VINS) ensures continuous operation under communication loss or GNSS denial.

---

## 1. Prerequisites

### Dependencies

| Dependency | Version | Notes |
|------------|---------|-------|
| ROS | Kinetic | `ros-kinetic-perception` meta-package |
| Eigen | 3.3.3 | Built from source |
| Ceres Solver | 1.12.0 | Built from source |
| OpenCV | Included with ROS | Via `cv_bridge` |
| [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm) | latest | GNSS message definitions |

Additional ROS packages: `cv-bridge`, `image-transport`, `message-filters`, `tf`.

### Hardware (for real-world deployment)

- Monocular or stereo camera
- IMU (≥200 Hz)
- Multi-constellation GNSS receiver (e.g., u-blox F9P)
- WiFi link between agents

---

## 2. Build

### Option A: Docker (Recommended)

A Dockerfile is provided for reproducible builds with all dependencies pre-configured.

```bash
# Clone the repository
git clone https://github.com/Serena-qiu/DCGVINS.git
cd DCGVINS

# Build Docker image
docker build -t dc-gvins .

# Run container
docker run -it --rm \
    -v /path/to/your/data:/root/data \
    dc-gvins
```

The Docker image handles the full toolchain: Eigen 3.3.3, Ceres 1.12.0, gnss_comm, and all ROS dependencies are built automatically. The workspace is ready to use once the container starts.

### Option B: Native Build

If you prefer a native installation, follow these steps on **Ubuntu 16.04 with ROS Kinetic**.

**Install system dependencies:**

```bash
sudo apt-get update && sudo apt-get install -y \
    git cmake libatlas-base-dev libgoogle-glog-dev \
    libsuitesparse-dev python-catkin-tools \
    ros-kinetic-cv-bridge ros-kinetic-image-transport \
    ros-kinetic-message-filters ros-kinetic-tf
```

**Build Eigen 3.3.3:**

```bash
git clone https://gitlab.com/libeigen/eigen.git
cd eigen && git checkout tags/3.3.3
mkdir build && cd build
cmake .. && sudo make install
cd ../.. && rm -rf eigen
```

**Build Ceres Solver 1.12.0:**

```bash
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver && git checkout tags/1.12.0
mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
cd ../.. && rm -rf ceres-solver
```

**Build the workspace:**

```bash
# Create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone gnss_comm
git clone https://github.com/HKUST-Aerial-Robotics/gnss_comm.git

# Clone DC-GVINS
git clone https://github.com/Serena-qiu/DCGVINS.git

# Configure and build
cd ~/catkin_ws
catkin config \
    --extend /opt/ros/kinetic \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=14 \
    -DCMAKE_CXX_FLAGS="-std=c++14"

# Build gnss_comm first, then remaining packages
catkin build gnss_comm
catkin build

# Source workspace
source ~/catkin_ws/devel/setup.bash
```

> **Note**: If you encounter a `D2R` redefinition error during compilation, comment out the duplicate definition in `estimator/src/estimator_node.cpp`.

---

## 3. Usage

Detailed run instructions and example commands will be provided after the dataset release. Stay tuned.

---

## 4. Datasets

Datasets collected in Hong Kong urban environments will be released. Details coming soon.

| Dataset | Platform | Duration | Environment | Status |
|---------|----------|----------|-------------|--------|
| HK Harbourfront | Handheld | 530 s | Dense urban canyon | Coming soon |
| UAV Flight | Quadcopter | 190 s | Semi-open campus | Coming soon |

---

## Citation

If you find this work useful, please cite:

```bibtex
@article{qiu2025dcgvins,
  title     = {Tightly-Coupled Double-Difference GNSS Constrained Collaborative
               Visual-Inertial Navigation for Multi-Agent Systems},
  author    = {Qiu, Shaoting and Wen, Weisong and Hu, Jiahao and
               Zhao, Jiaqi and Wang, Yingying},
  journal   = {IEEE Transactions on Robotics},
  year      = {2025},
  note      = {Under review}
}
```

---

## License

This project is released under the [GPL-3.0 License](LICENSE).

---

## Acknowledgements

This work was supported by the Smart Traffic Fund under the project "Development of an Assisted Navigation and Collision Avoidance System using AI and Location-based Service" (project no. PSRI/73/2309/PR).
