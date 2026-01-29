# VINS-Fusion ROS2 Utilities

Scripts for preparing test data for VINS-Fusion with barometer integration.

## Scripts

### `create_baro_bag.py`

Creates a ROS2 bag with spoofed barometer data for testing VINS with barometric altitude measurements.

**Features:**
- Copies an existing ROS2 bag
- Adds synthetic barometer data at configurable frequency
- Optional duration limiting (copy only first N seconds)
- Optional topic filtering (skip specific topics)

---

## Getting Started

### 1. Download Test Dataset

We use the EuRoC MAV datasets from the OpenVINS documentation.

#### Download V1_02_medium (Recommended for Testing)

```bash
# Create test_data directory if it doesn't exist
mkdir -p test_data
cd test_data

# Download the ROS1 bag (843 MB)
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_02_medium/V1_02_medium.bag

# Convert from ROS1 to ROS2 format
rosbags-convert V1_02_medium.bag --dst V1_02_medium

# Clean up the ROS1 bag to save space (optional)
rm V1_02_medium.bag

cd ..
```

#### Other Available Datasets

From [OpenVINS Dataset Page](https://docs.ros.org/en/noetic/api/ov_core/html/gs-datasets.html):

**EuRoC MAV Datasets:**
- V1_01_easy - Easy difficulty, well-lit
- V1_02_medium - Medium difficulty (recommended)
- V1_03_difficult - Fast motions, difficult
- V2_01_easy - Different room, easy
- V2_02_medium - Different room, medium
- MH_01_easy - Machine hall, easy
- MH_02_easy - Machine hall, easy
- MH_03_medium - Machine hall, medium
- MH_04_difficult - Machine hall, difficult
- MH_05_difficult - Machine hall, difficult

Download any of these:
```bash
# Example for MH_01_easy
cd test_data
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag
rosbags-convert MH_01_easy.bag --dst MH_01_easy
cd ..
```

### 2. Install Dependencies

```bash
# Make sure ROS2 Humble is sourced
source /opt/ros/humble/setup.bash

# Install required packages
sudo apt-get update
sudo apt-get install -y \
    ros-humble-rosbag2-py \
    ros-humble-rclpy \
    ros-humble-std-msgs

# Install rosbags-convert for bag format conversion
pip3 install rosbags
```

Or use the provided setup script:
```bash
bash setup_environment.sh
```

---

## Usage

### Basic Usage (with defaults)

```bash
# Edit the script to configure:
# - INPUT_BAG = "test_data/V1_02_medium"
# - OUTPUT_BAG = "test_data/New_Test"
# - BARO_FREQUENCY = 20.0  # Hz
# - DURATION_SECONDS = None  # None = entire bag

python3 create_baro_bag.py
```

### Command Line Arguments

```bash
# Syntax
python3 create_baro_bag.py [input_bag] [output_bag] [baro_freq_hz] [duration_sec]

# Examples:

# Copy entire bag with 20 Hz barometer
python3 create_baro_bag.py test_data/V1_02_medium test_data/V1_02_baro 20

# Copy first 30 seconds only
python3 create_baro_bag.py test_data/V1_02_medium test_data/V1_02_30s 20 30

# Copy first 60 seconds with 50 Hz barometer
python3 create_baro_bag.py test_data/V1_02_medium test_data/V1_02_60s 50 60

# Quick 10-second test
python3 create_baro_bag.py test_data/V1_02_medium test_data/Quick_Test 20 10
```

### Skipping Topics

To skip specific topics (e.g., topics that cause warnings), edit the script:

```python
# In create_baro_bag.py, change:
SKIP_TOPICS = ['/fcu/motor_speed']  # Skip motor speed topic
```

---

## Configuration

Edit these variables at the top of `create_baro_bag.py`:

```python
INPUT_BAG = "test_data/V1_02_medium"        # Input bag path
OUTPUT_BAG = "test_data/New_Test"           # Output bag path
BARO_FREQUENCY = 20.0                       # Barometer sampling rate (Hz)
DURATION_SECONDS = None                     # Duration limit (None = entire bag)
SKIP_TOPICS = []                            # Topics to skip (empty = copy all)
```

### Barometer Data

The script adds a `/baro` topic with:
- **Message type:** `std_msgs/Float32`
- **Data:** Constant height = 0.0 meters (simulates hovering at initialization height)
- **Frequency:** Configurable (default: 20 Hz)
- **Timestamps:** Synchronized with bag timestamps

---

## Testing with VINS-Fusion

After creating your bag:

```bash
# Terminal 1: Launch RViz
ros2 launch vins vins_rviz.launch

# Terminal 2: Run VINS with your config
ros2 run vins vins_node ~/path/to/your/config.yaml

# Terminal 3: Play the bag
ros2 bag play test_data/New_Test
```

### Common Warnings (Safe to Ignore)

```
Ignoring a topic '/fcu/motor_speed', reason: package 'asctec_hl_comm' not found
```
This is normal - the motor_speed topic uses a proprietary message type. It doesn't affect VINS.

---

## Output Structure

```
test_data/
├── V1_02_medium/              # Original ROS2 bag
│   ├── metadata.yaml
│   └── V1_02_medium_0.db3
├── New_Test/                  # Output bag with barometer data
│   ├── metadata.yaml
│   └── New_Test_0.db3
└── ...
```

---

## Troubleshooting

### "Unable to locate package python3-rosbag2"
The packages are named differently in ROS2 Humble. Use:
```bash
sudo apt-get install ros-humble-rosbag2-py ros-humble-rclpy
```

### "rosbags-convert: command not found"
Install the conversion tool:
```bash
pip3 install rosbags
```

### Missing topics in output bag
Make sure the topic isn't in `SKIP_TOPICS` list.

### Bag validation fails
Re-run the script. If it still fails, check disk space and permissions.

---

## Dataset Information

**EuRoC MAV Dataset:**
- Camera: Stereo global shutter, 20 fps, 752×480
- IMU: ADIS16448, 200 Hz
- Ground truth: Vicon motion capture system

**Citation:**
```
M. Burri, J. Nikolic, P. Gohl, T. Schneider, J. Rehder, S. Omari, M. Achtelik and R. Siegwart,
"The EuRoC micro aerial vehicle datasets",
International Journal of Robotics Research, 2016.
```

---

## License

These utilities are part of the VINS-Fusion-ROS2-humble-arm project and follow the same GPLv3 license.
