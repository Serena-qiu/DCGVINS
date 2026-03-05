# Tightly-Coupled Double-Difference GNSS Constrained Collaborative Visual-Inertial Navigation for Multi-Agent Systems

<p align="center">
  <a href="https://youtu.be/2zWdf0J1EU0"><img src="https://img.shields.io/badge/Video-YouTube-red?logo=youtube" alt="Video"></a>
  <a href="#"><img src="https://img.shields.io/badge/Paper-IEEE-blue" alt="Paper"></a>
  <a href="#license"><img src="https://img.shields.io/badge/License-GPL--3.0-green" alt="License"></a>
  <a href="#"><img src="https://img.shields.io/badge/Platform-ROS%20Noetic-brightgreen?logo=ros" alt="ROS"></a>
</p>

<p align="center">
  <img src="docs/system_overview.png" width="90%" alt="DC-GVINS System Overview">
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

## Performance

### Absolute Positioning Accuracy (RMSE)

| Method | Exp 1: Urban Handheld | Exp 2: UAV Flight |
|--------|:-----:|:-----:|
| SPP | 50.96 m | — |
| VINS-Mono | 20.81 m | 6.61 m |
| GVINS | 9.02 m | 4.02 m |
| **DC-GVINS** | **3.25 m** | **1.50 m** |

### Relative Positioning Accuracy (RMSE)

| Method | Exp 1: Baseline Length | Exp 1: Vector Error | Exp 2: Baseline Length | Exp 2: Vector Error |
|--------|:-----:|:-----:|:-----:|:-----:|
| VINS-Mono | 23.73 m | 40.58 m | 1.55 m | 1.87 m |
| GVINS | 12.41 m | 20.18 m | 1.16 m | 1.41 m |
| **DC-GVINS** | **2.14 m** | **4.33 m** | **0.69 m** | **0.79 m** |

### Computational Efficiency

| Platform | GVINS | DC-GVINS (10 DD) | Overhead |
|----------|:-----:|:-----:|:-----:|
| Intel i7-1165G7 | 20.94 ms | 26.41 ms | +26% |
| Jetson AGX Xavier | 10.45 ms | 12.86 ms | +23% |

DD factor cost: ~0.017 ms per constraint (analytical Jacobians).

---

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│                    Agent M (Primary)                │
│                                                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │
│  │  Camera   │  │   IMU    │  │  GNSS Receiver   │  │
│  └────┬─────┘  └────┬─────┘  └────────┬─────────┘  │
│       │              │                 │             │
│       ▼              ▼                 ▼             │
│  ┌─────────────────────────────────────────────┐    │
│  │       Tightly Coupled Factor Graph          │    │
│  │                                             │    │
│  │  Visual ── IMU ── GNSS ── DD Factors        │    │
│  │  Factors   Factors Factors  ▲               │    │
│  └─────────────────────────────┼───────────────┘    │
│                                │                     │
└────────────────────────────────┼─────────────────────┘
                                 │  Peer-to-Peer
                                 │  (~2.5 KB/s)
┌────────────────────────────────┼─────────────────────┐
│                    Agent N (Secondary)               │
│                                │                     │
│  ┌──────────┐  ┌──────────┐  ┌┴─────────────────┐   │
│  │  Camera   │  │   IMU    │  │  GNSS Receiver   │   │
│  └────┬─────┘  └────┬─────┘  └────────┬─────────┘   │
│       ▼              ▼                 ▼              │
│  ┌──────────────────────────────────────────────┐    │
│  │       Independent Factor Graph               │    │
│  └──────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────┘
```

---

## Prerequisites

### Hardware Requirements

- Monocular or stereo camera
- IMU (≥200 Hz)
- Multi-constellation GNSS receiver (e.g., u-blox F9P) supporting GPS/GLONASS/Galileo/BeiDou
- WiFi or other wireless communication link between agents

### Software Dependencies

- **Ubuntu 20.04**
- **ROS Noetic**
- **OpenCV** ≥ 3.4
- **Ceres Solver** ≥ 2.0
- **Eigen3** ≥ 3.3

---

## Build

```bash
# Create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone repository
git clone https://github.com/Serena-qiu/DCGVINS.git

# Install dependencies
sudo apt-get install ros-noetic-cv-bridge ros-noetic-tf ros-noetic-message-filters

# Build
cd ~/catkin_ws
catkin_make -j$(nproc)
source devel/setup.bash
```

---

## Quick Start

### Single-Pair Collaborative Navigation

```bash
# Terminal 1: Launch primary agent
roslaunch dc_gvins primary_agent.launch

# Terminal 2: Launch secondary agent
roslaunch dc_gvins secondary_agent.launch

# Terminal 3: Play rosbag (example dataset)
rosbag play your_dataset.bag
```

### Configuration

Key parameters in `config/dc_gvins.yaml`:

```yaml
# GNSS Configuration
gnss:
  enable_dd: true                # Enable double-difference constraints
  dd_elevation_mask: 15.0        # Satellite elevation mask (degrees)
  dd_snr_threshold: 30.0         # Minimum SNR for DD (dB-Hz)

# Communication
communication:
  sync_threshold: 1.0            # Max time offset for DD (seconds)
  degraded_threshold: 0.5        # Communication degraded threshold (s)
  disconnected_threshold: 2.0    # Communication lost threshold (s)
  weight_decay: 0.9              # Weight decay factor for degraded state

# Outlier Rejection
outlier:
  mad_scale: 3.0                 # MAD-based outlier threshold
  enable_adaptive_weight: true   # Enable quality-aware weighting
```

---

## Datasets

We provide sample datasets collected in two scenarios:

| Dataset | Platform | Duration | Environment | Download |
|---------|----------|----------|-------------|----------|
| HK Harbourfront | Handheld | 530 s | Dense urban canyon | [Link](#) |
| UAV Flight | Quadcopter | 190 s | Semi-open campus | [Link](#) |

### Data Format

Each dataset contains:
```
dataset/
├── agent_M/
│   ├── camera/          # Image sequences
│   ├── imu.csv          # IMU measurements (200 Hz)
│   ├── gnss_raw.obs     # GNSS raw observations (10 Hz)
│   └── ground_truth.csv # RTK reference trajectory
├── agent_N/
│   ├── ...              # Same structure
└── calibration/
    ├── cam_imu_M.yaml   # Camera-IMU extrinsics (Agent M)
    └── cam_imu_N.yaml   # Camera-IMU extrinsics (Agent N)
```

---

## Evaluation

We provide evaluation scripts for reproducing the results reported in the paper.

```bash
# Run evaluation on HK Harbourfront dataset
python scripts/evaluate.py --dataset hk_harbour --method dc_gvins

# Compare with baselines
python scripts/evaluate.py --dataset hk_harbour --method all --plot

# Compute relative positioning metrics
python scripts/relative_eval.py --dataset hk_harbour
```

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

## Related Projects

- [GVINS](https://github.com/HKUST-Aerial-Robotics/GVINS) — Tightly Coupled GNSS-Visual-Inertial Fusion (single-agent baseline)
- [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) — Monocular Visual-Inertial State Estimator
- [RTKLIB](https://github.com/tomojitakasu/RTKLIB) — Open Source GNSS Positioning

---

## License

This project is released under the [GPL-3.0 License](LICENSE).

---

## Acknowledgements

This work was supported by the Smart Traffic Fund under the project "Development of an Assisted Navigation and Collision Avoidance System using AI and Location-based Service" (project no. PSRI/73/2309/PR).
