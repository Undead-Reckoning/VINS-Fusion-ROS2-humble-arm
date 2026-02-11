#!/usr/bin/env python3
"""
Create a ROS2 bag with REAL barometer data extracted from ground truth Vicon measurements.
This creates realistic barometer data for testing the barometer factor integration.
"""

import sys
import os
from pathlib import Path
import csv
import numpy as np
import rclpy

try:
    from rosbag2_py import SequentialReader, SequentialWriter
    from rosbag2_py import StorageOptions, ConverterOptions, TopicMetadata
    from rclpy.serialization import serialize_message
    from std_msgs.msg import Header
    from baro_msgs.msg import BaroData
except ImportError as e:
    print(f"Error: Missing required Python packages.")
    print(f"Please install: pip install rosbag2-py rclpy")
    sys.exit(1)

def load_ground_truth(csv_path):
    """
    Load EuRoC ground truth CSV.
    Format: #time(ns),px,py,pz,qw,qx,qy,qz,vx,vy,vz,...
    Returns: dict mapping timestamp_ns -> height_z
    """
    height_map = {}
    
    print(f"Loading ground truth from: {csv_path}")
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # skip header
        count = 0
        for row in reader:
            if row and not row[0].startswith('#'):
                try:
                    t_ns = int(row[0])
                    pz = float(row[3])  # Height is column 3 (px, py, pz)
                    height_map[t_ns] = pz
                    count += 1
                except (ValueError, IndexError):
                    pass
    
    print(f"  → Loaded {count} height measurements")
    return height_map

def create_baro_bag_from_truth(input_bag_path, output_bag_path, truth_csv_path, 
                                baro_frequency=20.0, baro_noise_std=0.0, 
                                duration_seconds=None, skip_topics=None):
    """
    Copy a ROS2 bag and add realistic barometer data from ground truth.
    
    Args:
        input_bag_path: Path to input bag directory
        output_bag_path: Path to output bag directory
        truth_csv_path: Path to ground truth CSV file
        baro_frequency: Barometer sampling frequency in Hz
        baro_noise_std: Standard deviation of noise to add (0 = no noise)
        duration_seconds: Maximum duration to copy in seconds (None = copy entire bag)
        skip_topics: List of topics to skip
    """
    if skip_topics is None:
        skip_topics = []
    
    # Load ground truth
    height_map = load_ground_truth(truth_csv_path)
    if not height_map:
        print("ERROR: Could not load ground truth heights!")
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
    
    # Get all topics and create them in the writer
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
    
    # Add barometer topic
    baro_topic = TopicMetadata(
        name='/baro',
        type='baro_msgs/msg/BaroData',
        serialization_format='cdr'
    )
    writer.create_topic(baro_topic)
    print(f"  - /baro (baro_msgs/msg/BaroData) [NEW - from ground truth]")
    
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
    
    # Add barometer messages from ground truth
    print(f"\nAdding barometer data at {baro_frequency} Hz (from ground truth)...")
    duration_s = (end_time - start_time) / 1e9
    num_baro_samples = int(duration_s * baro_frequency)
    
    baro_added = 0
    baro_interpolated = 0
    baro_not_found = 0
    
    print(f"  Ground truth range: {min(height_map.keys())} to {max(height_map.keys())} ns")
    
    heights_used = []
    
    for i in range(num_baro_samples):
        timestamp = start_time + int((i / baro_frequency) * 1e9)
        
        # Try to find exact or nearest height measurement
        if timestamp in height_map:
            height = height_map[timestamp]
            baro_added += 1
        else:
            # Find nearest timestamp in height_map
            nearest_ts = min(height_map.keys(), key=lambda t: abs(t - timestamp))
            if abs(nearest_ts - timestamp) < 1e9:  # Within 1 second
                height = height_map[nearest_ts]
                baro_interpolated += 1
            else:
                # No ground truth available at this time
                height = 0.0
                baro_not_found += 1
        
        # Add noise if requested
        if baro_noise_std > 0:
            height += np.random.normal(0, baro_noise_std)
        
        heights_used.append(height)
        
        # Create and write barometer message
        header = Header()
        header.stamp.sec = int(timestamp // 1_000_000_000)
        header.stamp.nanosec = int(timestamp % 1_000_000_000)
        header.frame_id = 'barometer'
        
        baro_msg = BaroData()
        baro_msg.header = header
        baro_msg.altitude = float(height)
        writer.write('/baro', serialize_message(baro_msg), timestamp)
    
    # Cleanup
    print("\nFinalizing bag...")
    del writer
    del reader
    
    # Print statistics
    heights_used = np.array(heights_used)
    print(f"\nBarometer Data Statistics:")
    print(f"  Added exact matches:     {baro_added}")
    print(f"  Interpolated (nearest):  {baro_interpolated}")
    print(f"  Not found (used 0.0):    {baro_not_found}")
    print(f"  Height range: {np.min(heights_used):.4f} to {np.max(heights_used):.4f} m")
    print(f"  Height mean:  {np.mean(heights_used):.4f} m")
    print(f"  Height std:   {np.std(heights_used):.4f} m")
    
    print(f"\n✓ Successfully created: {output_bag_path}")
    print(f"  Duration: {duration_s:.2f} seconds")
    print(f"  Barometer samples: {num_baro_samples}")
    print(f"  Total topics: {len(topic_counts) + 1} (including /baro)")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 create_baro_bag_from_truth.py <input_bag> <output_bag> <truth_csv> [baro_freq] [noise_std]")
        print("\nExample:")
        print("  python3 create_baro_bag_from_truth.py \\")
        print("    test_data/V1_01_easy \\")
        print("    test_data/V1_01_baro_truth \\")
        print("    test_data/V1_01_easy/V1_01_easy.csv \\")
        print("    20.0  # 20 Hz barometer")
        print("    0.1   # 0.1 m noise std")
        sys.exit(1)
    
    input_bag = sys.argv[1]
    output_bag = sys.argv[2]
    truth_csv = sys.argv[3]
    baro_freq = float(sys.argv[4]) if len(sys.argv) > 4 else 20.0
    baro_noise = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
    
    create_baro_bag_from_truth(
        input_bag, 
        output_bag,
        truth_csv,
        baro_frequency=baro_freq,
        baro_noise_std=baro_noise
    )
