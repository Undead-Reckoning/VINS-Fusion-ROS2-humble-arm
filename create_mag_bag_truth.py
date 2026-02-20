#!/usr/bin/env python3
"""
Create a ROS2 bag with REAL magnetometer data extracted from ground truth Vicon measurements.
This creates realistic magnetometer data for testing the magnetometer factor integration.
"""

import sys
import os
from pathlib import Path
import csv
import numpy as np
import yaml
import re
import math

try:
    from rosbag2_py import SequentialReader, SequentialWriter
    from rosbag2_py import StorageOptions, ConverterOptions, TopicMetadata
    from rclpy.serialization import serialize_message
    from std_msgs.msg import Float32
    from sensor_msgs.msg import MagneticField
    from std_msgs.msg import Header
except ImportError as e:
    print(f"Error: Missing required Python packages.")
    print(f"Please install: pip install rosbag2-py rclpy")
    print(f"Error details: {e}")
    sys.exit(1)

def load_world_mag_field_from_config(config_path):
    """Load world magnetic field from YAML config file."""
    try:
        # Read file and strip YAML directive lines (e.g. "%YAML:1.0") which PyYAML may reject
        # Also remove OpenCV YAML tags like '!!opencv-matrix' which SafeLoader may not accept
        with open(config_path, 'r') as f:
            raw = f.read()
        filtered = "\n".join([ln for ln in raw.splitlines() if not ln.lstrip().startswith('%')])
        # remove any !!tag occurrences (e.g. !!opencv-matrix)
        filtered = re.sub(r"!!\S+", "", filtered)
        config = yaml.safe_load(filtered)
        if isinstance(config, dict) and 'mag_world_field' in config:
            mag_field = config['mag_world_field']
            print(f"Loaded world magnetic field from config: {mag_field}")
            return np.array(mag_field)
        else:
            print(f"'mag_world_field' not found in config, using default: [0.2, 0.9, -0.3]")
            return np.array([0.2, 0.9, -0.3])
    except Exception as e:
        print(f"Error reading config file {config_path}: {e}")
        print(f"Using default world magnetic field: [0.2, 0.9, -0.3]")
        return np.array([0.2, 0.9, -0.3])

def load_ground_truth(csv_path):
    """
    Load EuRoC ground truth CSV.
    Format: #time(ns),px,py,pz,qw,qx,qy,qz,vx,vy,vz,...
    Returns: dict mapping timestamp_ns -> magx,magy,magz
    """

    qw_map = {}
    qx_map = {}
    qy_map = {}
    qz_map = {}

    print(f"Loading ground truth from: {csv_path}")
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        next(reader) #skip header
        count = 0
        for row in reader:
            if row and not row[0].startswith('#'):
                try:
                    t_ns = int(row[0])
                    qw = float(row[4])
                    qx = float(row[5])
                    qy = float(row[6])
                    qz = float(row[7])
                    qw_map[t_ns] = qw
                    qx_map[t_ns] = qx
                    qy_map[t_ns] = qy
                    qz_map[t_ns] = qz
                    count += 1
                except (ValueError, IndexError):
                    pass
        print(f"    -> Loaded {count} q measurements")
        return qw_map, qx_map, qy_map, qz_map
                    

def create_mag_bag_from_truth(input_bag_path, output_bag_path, truth_csv_path, config_path=None, mag_frequency=20.0, mag_noise_std=0.0, duration_seconds=None, skip_topics=None):
    """
    Copy a ROS2 bag and add realistic magnetometer data from ground truth.
    
    Args:
        input_bag_path: Path to input bag directory
        output_bag_path: Path to output bag directory
        truth_csv_path: Path to ground truth CSV file
        config_path: Path to YAML config file (to read world magnetic field)
        mag_frequency: Magnetometer sampling frequency in Hz
        mag_noise_std: Standarddeviation of noise to add (0 = no noise)
        duration_seconds: Maximum duration to copy in seconds (None = copy entire bag)
        skip_topics: List of topics to skip (default: None - don't skip any topics)
    """
    if skip_topics is None:
        skip_topics = []

    # Load world magnetic field from config if provided
    if config_path:
        mag_world = load_world_mag_field_from_config(config_path)
    else:
        mag_world = np.array([0.2, 0.9, -0.3])
        print(f"No config file provided, using default world magnetic field: {mag_world}")

    #print(f"mag world {mag_world}")
    #mag_world = mag_world + np.array([0.1, -0.05, 0.0])
    #mag_world = mag_world / np.linalg.norm(mag_world)
    #print(f"mag world norm {mag_world}")
    print(f"Using normalized world magnetic field: {mag_world}")

    #load ground truth
    qw_map, qx_map, qy_map, qz_map = load_ground_truth(truth_csv_path)
    if not qw_map:
        print("ERROR: Could now load ground truth qw!")
        sys.exit(1)
    if not qx_map:
        print("ERROR: Could now load ground truth qx!")
        sys.exit(1)
    if not qy_map:
        print("ERROR: Could now load ground truth qy!")
        sys.exit(1)
    if not qz_map:
        print("ERROR: Could now load ground truth qz!")
        sys.exit(1)

    # Setup reader
    print(f"\nOpening input bag: {input_bag_path}")
    reader_storage = StorageOptions(uri=str(input_bag_path), storage_id='sqlite3')
    reader_converter = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = SequentialReader()
    reader.open(reader_storage, reader_converter)
    
    # Setup writer
    print(f"Creating output bag: {output_bag_path}")
    if os.path.exists(output_bag_path):
        import shutil
        shutil.rmtree(output_bag_path)
    
    writer_storage = StorageOptions(uri=str(output_bag_path), storage_id='sqlite3')
    writer_converter = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    writer = SequentialWriter()
    writer.open(writer_storage, writer_converter)
    
    # Get all topics and create them in the writer (except skipped ones)
    topic_types = reader.get_all_topics_and_types()
    topic_counts = {}
    
    print(f"\nCopying topics:")
    if skip_topics:
        print(f"  (Skipping: {skip_topics})")
    for topic_metadata in topic_types:
        if topic_metadata.name not in skip_topics:
            writer.create_topic(topic_metadata)
            topic_counts[topic_metadata.name] = 0
            print(f"  - {topic_metadata.name} ({topic_metadata.type})")
        else:
            print(f"  - {topic_metadata.name} (SKIPPED)")
    
    # Add magnetometer topic
    mag_topic = TopicMetadata(
        name='/mag',
        type='sensor_msgs/msg/MagneticField',
        serialization_format='cdr'
    )
    writer.create_topic(mag_topic)
    print(f"  - /mag (sensor_msgs/msg/MagneticField) [NEW - from ground truth]")
    
    # Copy all messages
    print("\nCopying messages...")
    start_time = None
    end_time = None
    cutoff_time = None
    message_count = 0
    
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        
        if start_time is None:
            start_time = timestamp
            if duration_seconds is not None:
                cutoff_time = start_time + int(duration_seconds * 1e9)
                print(f"  Limiting to {duration_seconds} seconds from start")
        
        # Check if we've exceeded the duration limit
        if cutoff_time is not None and timestamp > cutoff_time:
            print(f"  Reached duration limit of {duration_seconds} seconds")
            break
        
        end_time = timestamp
        
        if topic not in skip_topics:
            writer.write(topic, data, timestamp)
            topic_counts[topic] = topic_counts.get(topic, 0) + 1
            message_count += 1
            
            if message_count % 1000 == 0:
                elapsed = (timestamp - start_time) / 1e9
                print(f"  Copied {message_count} messages... ({elapsed:.1f}s)")
    
    print(f"\nCopied {message_count} total messages")
    print("\nMessage counts by topic:")
    for topic, count in sorted(topic_counts.items()):
        print(f"  {topic}: {count}")
    
    # Add magnetometer messages from ground truth
    print(f"\nAdding magnetometer data at {mag_frequency} Hz (from ground truth)...")
    duration_s = (end_time - start_time) / 1e9
    num_mag_samples = int(duration_s * mag_frequency)

    mag_added = 0
    mag_interpolated = 0
    mag_not_found = 0

    print(f"    Ground truth range: {min(qw_map.keys())} to {max(qw_map.keys())} ns")

    mag_used = []

    first_time = True
    R_wb0 = np.array([[np.nan, np.nan, np.nan],[np.nan, np.nan, np.nan],[np.nan, np.nan, np.nan]])
    
    for i in range(num_mag_samples):
        timestamp = start_time + int((i / mag_frequency) * 1e9)

        #try to find exact or nearest q measurement
        if timestamp in qw_map:
            qw_rot = qw_map[timestamp]
            qx_rot = qx_map[timestamp]
            qy_rot = qy_map[timestamp]
            qz_rot = qz_map[timestamp]
            mag_added += 1
        else:
            #Find nearest timestamp in height_map
            nearest_ts = min(qw_map.keys(), key=lambda t: abs(t - timestamp))
            if abs(nearest_ts - timestamp) < 1e9: #within 1 sec
                qw_rot = qw_map[nearest_ts]
                qy_rot = qy_map[nearest_ts]
                qx_rot = qx_map[nearest_ts]
                qz_rot = qz_map[nearest_ts]
                mag_interpolated += 1
            else:
                #No ground truth available at this time
                qw_rot = 0.0
                qx_rot = 0.0
                qy_rot = 0.0
                qz_rot = 0.0
                mag_not_found += 1

        #Add noise if requested
        if mag_noise_std > 0:
            qw_rot += np.random.normal(0, mag_noise_std)
            qx_rot += np.random.normal(0, mag_noise_std)
            qy_rot += np.random.normal(0, mag_noise_std)
            qz_rot += np.random.normal(0, mag_noise_std)

        mag_used.append([qw_rot, qx_rot, qy_rot, qz_rot])

        #Create and write magnetometer message
        #mag_msg = Float32()
        mag_msg = MagneticField()

        #ADDED WHAT MAG WOULD "READ" FROM GIVEN QUATERNION DATA
        #m_b = R_wb.transpose() * m_w
        # Use the world magnetic field loaded from config
        # (already normalized above)

        q_arr = np.array([qw_rot, qx_rot, qy_rot, qz_rot])
        q_arr /= np.linalg.norm(q_arr)
        qw_norm, qx_norm, qy_norm, qz_norm = q_arr

        R_wb = np.array([
        [1 - 2*(qy_norm*qy_norm + qz_norm*qz_norm),     2*(qx_norm*qy_norm - qz_norm*qw_norm),     2*(qx_norm*qz_norm + qy_norm*qw_norm)],
        [    2*(qx_norm*qy_norm + qz_norm*qw_norm), 1 - 2*(qx_norm*qx_norm + qz_norm*qz_norm),     2*(qy_norm*qz_norm - qx_norm*qw_norm)],
        [    2*(qx_norm*qz_norm - qy_norm*qw_norm),     2*(qy_norm*qz_norm + qx_norm*qw_norm), 1 - 2*(qx_norm*qx_norm + qy_norm*qy_norm)]
        ])
    
        #Convert from euroc world to vins world frame, stays constant. Reminder: R_wb = body to world
        R0 = [ [0.327759, -0.0939208, 0.940081], [0, -0.995046, -0.0994122], [0.944761, 0.0325832, -0.326135]] #R0 from vins, may change
        if not math.isnan(R_wb[0][0]) and first_time: #only computes once. Needs to be constant but R_wb will change. 
            #print("IF PASSED")
            R_wb0 = R_wb #this is initial euroc body to world
            first_time = False
        #print(R_wb0)
        #VINS body to world @ Euroc world to body @ Euroc body to world
        R_vins = R0 @ R_wb0.T @ R_wb

        #R_eur_to_vins = R0 @ R_wb0.T
        #print(R_eur_to_vins)

        mag_body = R_vins.T @ mag_world #vins world to body @ mag VINS world

        #For error testing
        yaw_err = np.deg2rad(45) #deg to rad
        R_err = np.array([
        [ np.cos(yaw_err), -np.sin(yaw_err), 0],
        [ np.sin(yaw_err),  np.cos(yaw_err), 0],
        [ 0,                  0,                 1]
        ])
        mag_world_err = R_err @ mag_world
        #mag_body = R_vins.T @ mag_world_err
        #comment to remove



        header = Header()
        header.stamp.sec = int(timestamp // 1_000_000_000)
        header.stamp.nanosec = int(timestamp % 1_000_000_000)
        header.frame_id = 'magnetometer'

        mag_msg.header = header
        #VINS: qx: 0.8137	qy: -0.0277	qz: 0.5793	qw:0.0392
        #Truth: qx: 0.789985 qy: -0.205376 qz: 0.554528 qw: 0.161996
        mag_msg.magnetic_field.x = float(mag_body[0])
        mag_msg.magnetic_field.y = float(mag_body[1])
        mag_msg.magnetic_field.z = float(mag_body[2])

        writer.write('/mag', serialize_message(mag_msg), timestamp)
    
    # Cleanup
    print("\nFinalizing bag...")
    del writer
    del reader

    mag_used = np.array(mag_used)
    print(f"\nMagnetometer Data Statistics:")
    print(f"    Added exact matches:    {mag_added}")
    print(f"    Interpolated (nearest): {mag_interpolated}")
    print(f"    Not found (used 0.0):   {mag_not_found}")
    #Didnt do all seen in baro
    
    print(f"\n✓ Successfully created: {output_bag_path}")
    print(f"  Duration: {duration_s:.2f} seconds")
    print(f"  Total topics: {len(topic_counts) + 1} (including /mag)")

if __name__=="__main__":
    if len(sys.argv) < 4:
        print("Usage: python3 create_mag_bag_truth.py <input bag> <output_bag> <truth_csv> <config_yaml> [mag_freq] [noise_std]")
        print("\nExample:")
        print(" python3 create_mag_bag_truth.py \\")
        print("    test_data/V1_02_easy \\")
        print("    test_data/V1_02_mag_truth \\")
        print("    test_data/V1_02_easy/V1_02_easy.csv \\")
        print("    config/euroc/euroc_stereo_imu_config.yaml \\")
        print("    20.0  # 20 Hz magnetometer")
        print("    0.1   # 0.1 m noise std")
        sys.exit(1)

    input_bag = sys.argv[1]
    output_bag = sys.argv[2]
    truth_csv = sys.argv[3]
    config_file = sys.argv[4]
    mag_freq = float(sys.argv[5]) if len(sys.argv) > 5 else 20.0
    mag_noise = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0

    create_mag_bag_from_truth(
        input_bag,
        output_bag,
        truth_csv,
        config_path=config_file,
        mag_frequency=mag_freq,
        mag_noise_std=mag_noise
    )

#def validate_bag(bag_path):
#    """Print info about a bag file."""
#    print(f"\n{'='*60}")
#    print(f"Validating bag: {bag_path}")
#    print(f"{'='*60}")
#    
#    try:
#        storage = StorageOptions(uri=str(bag_path), storage_id='sqlite3')
#        converter = ConverterOptions(
#            input_serialization_format='cdr',
#            output_serialization_format='cdr'
#        )
#        
#        reader = SequentialReader()
#        reader.open(storage, converter)
#       
#        # Get metadata
#        metadata = reader.get_metadata()
#        topic_types = reader.get_all_topics_and_types()
#        
#        print(f"\nBag info:")
#        print(f"  Duration: {metadata.duration.nanoseconds / 1e9:.2f} seconds")
#        print(f"  Start time: {metadata.starting_time.nanoseconds / 1e9:.2f}")
#        print(f"  Message count: {metadata.message_count}")
#        
#        print(f"\nTopics:")
#        for topic in topic_types:
#            print(f"  {topic.name}")
#            print(f"    Type: {topic.type}")
#        
#        # Count messages per topic
#        topic_counts = {}
#        while reader.has_next():
#            (topic, data, timestamp) = reader.read_next()
#            topic_counts[topic] = topic_counts.get(topic, 0) + 1
#        
#        print(f"\nMessage counts:")
#        for topic, count in sorted(topic_counts.items()):
#            print(f"  {topic}: {count}")
#        
#        del reader
#        print(f"\n✓ Bag is valid")
#        return True
#        
#    except Exception as e:
#        print(f"\n✗ Error validating bag: {e}")
#        return False


#if __name__ == '__main__':
#    # Configuration
#    INPUT_BAG = "test_data/V1_01_easy"
#    OUTPUT_BAG = "test_data/Test_Mag_Zeros_10s"
#    MAG_FREQUENCY = 20.0  # Hz
#    DURATION_SECONDS = 10  # None = entire bag, or set to e.g. 30.0 for 30 seconds
#    SKIP_TOPICS = []  # Empty list = don't skip any topics. Example: ['/fcu/motor_speed', '/some/topic']
#    
#    # You can also pass arguments from command line
#    if len(sys.argv) > 1:
#        INPUT_BAG = sys.argv[1]
#    if len(sys.argv) > 2:
#        OUTPUT_BAG = sys.argv[2]
#    if len(sys.argv) > 3:
#        MAG_FREQUENCY = float(sys.argv[3])
#    if len(sys.argv) > 4:
#        DURATION_SECONDS = float(sys.argv[4])
#    
#    print("="*60)
#    print("ROS2 Bag Copy with Magnetometer Data")
#    print("="*60)
#    print(f"Input:  {INPUT_BAG}")
#    print(f"Output: {OUTPUT_BAG}")
#    print(f"Mag frequency: {MAG_FREQUENCY} Hz")
#    if DURATION_SECONDS is not None:
#        print(f"Duration limit: {DURATION_SECONDS} seconds")
#    else:
#        print(f"Duration limit: None (entire bag)")
#    if SKIP_TOPICS:
#        print(f"Skipping topics: {SKIP_TOPICS}")
#    else:
#        print(f"Skipping topics: None (all topics copied)")
#    print("="*60)
#    
#    try:
#        # Copy bag and add magnetometer data
#        copy_bag_with_mag(
#            input_bag_path=INPUT_BAG,
#            output_bag_path=OUTPUT_BAG,
#            mag_frequency=MAG_FREQUENCY,
#            duration_seconds=DURATION_SECONDS,
#            skip_topics=SKIP_TOPICS
#        )
#        
#        # Validate the output
#        validate_bag(OUTPUT_BAG)
#        
#    except Exception as e:
#        print(f"\n✗ Error: {e}")
#        import traceback
#        traceback.print_exc()
#        sys.exit(1)