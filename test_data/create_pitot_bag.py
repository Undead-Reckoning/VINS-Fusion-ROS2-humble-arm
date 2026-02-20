#!/usr/bin/env python3
"""
Script to copy a ROS2 bag and add pitot tube airspeed data.
Generates airspeed from ground truth velocity with optional wind model.
"""

import sys
import os
from pathlib import Path
import numpy as np

try:
    from rosbag2_py import SequentialReader, SequentialWriter
    from rosbag2_py import StorageOptions, ConverterOptions, TopicMetadata
    from rclpy.serialization import deserialize_message, serialize_message
    from rosidl_runtime_py.utilities import get_message
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
except ImportError as e:
    print(f"Error: Missing required Python packages.")
    print(f"Please install: pip install rosbag2-py rclpy")
    print(f"Error details: {e}")
    sys.exit(1)

def copy_bag_with_pitot(input_bag_path, output_bag_path, 
                        velocity_topic='/mavros/local_position/odom',
                        wind_velocity=(0.0, 0.0, 0.0),
                        add_noise=True,
                        noise_std=0.1,
                        duration_seconds=None):
    """
    Copy a ROS2 bag and add pitot tube airspeed data.
    
    Args:
        input_bag_path: Path to input bag directory
        output_bag_path: Path to output bag directory
        velocity_topic: Topic containing velocity data (Odometry or TwistStamped)
        wind_velocity: Tuple (x, y, z) of wind in NED frame [m/s]
        add_noise: Whether to add Gaussian noise to airspeed
        noise_std: Standard deviation of noise in m/s
        duration_seconds: Maximum duration to copy in seconds (None = entire bag)
    """
    
    # Setup reader
    print(f"Opening input bag: {input_bag_path}")
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
    type_map = {t.name: t.type for t in topic_types}
    topic_counts = {}
    
    print(f"\nCopying topics:")
    for topic_metadata in topic_types:
        writer.create_topic(topic_metadata)
        topic_counts[topic_metadata.name] = 0
        print(f"  - {topic_metadata.name} ({topic_metadata.type})")
    
    # Check if velocity topic exists
    if velocity_topic not in type_map:
        print(f"\n✗ ERROR: Velocity topic '{velocity_topic}' not found in bag!")
        print(f"Available topics: {list(type_map.keys())}")
        sys.exit(1)
    
    # Add pitot topic
    pitot_topic = TopicMetadata(
        name='/pitot/airspeed',
        type='geometry_msgs/msg/TwistStamped',
        serialization_format='cdr'
    )
    writer.create_topic(pitot_topic)
    print(f"  - /pitot/airspeed (geometry_msgs/msg/TwistStamped) [NEW]")
    
    # Copy all messages and generate pitot data
    print("\nCopying messages and generating pitot data...")
    start_time = None
    cutoff_time = None
    message_count = 0
    pitot_count = 0
    wind_vec = np.array(wind_velocity)
    
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        
        if start_time is None:
            start_time = timestamp
            if duration_seconds is not None:
                cutoff_time = start_time + int(duration_seconds * 1e9)
                print(f"  Limiting to {duration_seconds} seconds from start")
        
        # Check duration limit
        if cutoff_time is not None and timestamp > cutoff_time:
            print(f"  Reached duration limit of {duration_seconds} seconds")
            break
        
        # Write original message
        writer.write(topic, data, timestamp)
        topic_counts[topic] = topic_counts.get(topic, 0) + 1
        message_count += 1
        
        # Generate pitot data from velocity topic
        if topic == velocity_topic:
            msg_type = get_message(type_map[topic])
            vel_msg = deserialize_message(data, msg_type)
            
            # Extract velocity (works for both Odometry and TwistStamped)
            if hasattr(vel_msg, 'twist'):
                if hasattr(vel_msg.twist, 'twist'):  # Odometry
                    vx = vel_msg.twist.twist.linear.x
                    vy = vel_msg.twist.twist.linear.y
                    vz = vel_msg.twist.twist.linear.z
                    header = vel_msg.header
                else:  # TwistStamped
                    vx = vel_msg.twist.linear.x
                    vy = vel_msg.twist.linear.y
                    vz = vel_msg.twist.linear.z
                    header = vel_msg.header
            else:
                continue
            
            # Calculate airspeed: ground_velocity - wind_velocity
            # Simplified: assume velocity is already in body frame or NED
            air_velocity = np.array([vx, vy, vz]) - wind_vec
            
            # Pitot measures forward airspeed (x-component in body frame)
            airspeed = air_velocity[0]
            
            # Add noise if requested
            if add_noise:
                airspeed += np.random.normal(0, noise_std)
            
            # Pitot can't measure negative (it measures dynamic pressure)
            airspeed = max(0.0, airspeed)
            
            # Create pitot message
            pitot_msg = TwistStamped()
            pitot_msg.header.stamp = header.stamp
            pitot_msg.header.frame_id = "body"
            pitot_msg.twist.linear.x = airspeed
            pitot_msg.twist.linear.y = 0.0
            pitot_msg.twist.linear.z = 0.0
            pitot_msg.twist.angular.x = 0.0
            pitot_msg.twist.angular.y = 0.0
            pitot_msg.twist.angular.z = 0.0
            
            # Write pitot message
            writer.write('/pitot/airspeed', serialize_message(pitot_msg), timestamp)
            pitot_count += 1
        
        if message_count % 1000 == 0:
            elapsed = (timestamp - start_time) / 1e9
            print(f"  Copied {message_count} messages ({pitot_count} pitot)... ({elapsed:.1f}s)")
    
    print(f"\nCopied {message_count} total messages")
    print(f"Generated {pitot_count} pitot measurements")
    print("\nMessage counts by topic:")
    for topic, count in sorted(topic_counts.items()):
        print(f"  {topic}: {count}")
    
    # Cleanup
    print("\nFinalizing bag...")
    del writer
    del reader
    
    duration_s = (timestamp - start_time) / 1e9
    print(f"\n✓ Successfully created: {output_bag_path}")
    print(f"  Duration: {duration_s:.2f} seconds")
    print(f"  Wind: ({wind_velocity[0]:.1f}, {wind_velocity[1]:.1f}, {wind_velocity[2]:.1f}) m/s")
    print(f"  Noise: {'Enabled' if add_noise else 'Disabled'} (std={noise_std} m/s)")


def list_topics(bag_path):
    """List all topics in a bag file."""
    print(f"\n{'='*60}")
    print(f"Topics in bag: {bag_path}")
    print(f"{'='*60}")
    
    try:
        storage = StorageOptions(uri=str(bag_path), storage_id='sqlite3')
        converter = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        reader = SequentialReader()
        reader.open(storage, converter)
        
        topic_types = reader.get_all_topics_and_types()
        
        print(f"\nFound {len(topic_types)} topics:")
        for topic in topic_types:
            print(f"  {topic.name}")
            print(f"    Type: {topic.type}")
        
        del reader
        return [t.name for t in topic_types]
        
    except Exception as e:
        print(f"\n✗ Error reading bag: {e}")
        return []


if __name__ == '__main__':
    # Configuration
    INPUT_BAG = "test_data/V1_01_easy"
    OUTPUT_BAG = "test_data/V1_01_easy_with_pitot"
    
    # Which topic has velocity? Common options:
    # - '/mavros/local_position/odom' (PX4/MAVROS)
    # - '/ground_truth/odometry' (simulation)
    # - '/vins_estimator/odometry' (VINS output)
    VELOCITY_TOPIC = '/mavros/local_position/odom'
    
    # Wind settings (NED frame: North, East, Down in m/s)
    WIND_VELOCITY = (0.0, 0.0, 0.0)  # No wind
    # WIND_VELOCITY = (3.0, -2.0, 0.0)  # 3 m/s North, 2 m/s West
    
    # Noise settings
    ADD_NOISE = True
    NOISE_STD = 0.1  # Standard deviation in m/s
    
    # Duration limit
    DURATION_SECONDS = None  # None = entire bag
    
    # Command line arguments
    if len(sys.argv) > 1:
        if sys.argv[1] == '--list-topics':
            if len(sys.argv) > 2:
                list_topics(sys.argv[2])
            else:
                list_topics(INPUT_BAG)
            sys.exit(0)
        INPUT_BAG = sys.argv[1]
    if len(sys.argv) > 2:
        OUTPUT_BAG = sys.argv[2]
    
    print("="*60)
    print("ROS2 Bag Copy with Pitot Tube Data")
    print("="*60)
    print(f"Input:  {INPUT_BAG}")
    print(f"Output: {OUTPUT_BAG}")
    print(f"Velocity topic: {VELOCITY_TOPIC}")
    print(f"Wind: ({WIND_VELOCITY[0]:.1f}, {WIND_VELOCITY[1]:.1f}, {WIND_VELOCITY[2]:.1f}) m/s")
    print(f"Noise: {'Enabled' if ADD_NOISE else 'Disabled'} (std={NOISE_STD} m/s)")
    if DURATION_SECONDS:
        print(f"Duration limit: {DURATION_SECONDS} seconds")
    print("="*60)
    
    try:
        copy_bag_with_pitot(
            input_bag_path=INPUT_BAG,
            output_bag_path=OUTPUT_BAG,
            velocity_topic=VELOCITY_TOPIC,
            wind_velocity=WIND_VELOCITY,
            add_noise=ADD_NOISE,
            noise_std=NOISE_STD,
            duration_seconds=DURATION_SECONDS
        )
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)