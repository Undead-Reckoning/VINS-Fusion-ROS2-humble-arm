#!/usr/bin/env python3
"""
Script to copy a ROS2 bag and add spoofed barometer data.
This is a replacement for the buggy MATLAB ros2bagwriter.
"""

import sys
import os
from pathlib import Path

try:
    from rosbag2_py import SequentialReader, SequentialWriter
    from rosbag2_py import StorageOptions, ConverterOptions, TopicMetadata
    from rclpy.serialization import deserialize_message, serialize_message
    from rosidl_runtime_py.utilities import get_message
    from std_msgs.msg import Float32
except ImportError as e:
    print(f"Error: Missing required Python packages.")
    print(f"Please install: pip install rosbag2-py rclpy")
    print(f"Error details: {e}")
    sys.exit(1)

def copy_bag_with_baro(input_bag_path, output_bag_path, baro_frequency=20.0, duration_seconds=None, skip_topics=None):
    """
    Copy a ROS2 bag and add barometer data.
    
    Args:
        input_bag_path: Path to input bag directory
        output_bag_path: Path to output bag directory
        baro_frequency: Barometer sampling frequency in Hz
        duration_seconds: Maximum duration to copy in seconds (None = copy entire bag)
        skip_topics: List of topics to skip (default: None - don't skip any topics)
    """
    if skip_topics is None:
        skip_topics = []
    
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
    
    # Add barometer topic
    baro_topic = TopicMetadata(
        name='/baro',
        type='std_msgs/msg/Float32',
        serialization_format='cdr'
    )
    writer.create_topic(baro_topic)
    print(f"  - /baro (std_msgs/msg/Float32) [NEW]")
    
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
    
    # Add barometer messages
    print(f"\nAdding barometer data at {baro_frequency} Hz...")
    duration_s = (end_time - start_time) / 1e9
    num_baro_samples = int(duration_s * baro_frequency)
    
    baro_msg = Float32()
    baro_msg.data = 0.0
    
    for i in range(num_baro_samples):
        timestamp = start_time + int((i / baro_frequency) * 1e9)
        writer.write('/baro', serialize_message(baro_msg), timestamp)
    
    print(f"Added {num_baro_samples} barometer messages (constant height = 0.0 m)")
    
    # Cleanup
    print("\nFinalizing bag...")
    del writer
    del reader
    
    print(f"\n✓ Successfully created: {output_bag_path}")
    print(f"  Duration: {duration_s:.2f} seconds")
    print(f"  Total topics: {len(topic_counts) + 1} (including /baro)")


def validate_bag(bag_path):
    """Print info about a bag file."""
    print(f"\n{'='*60}")
    print(f"Validating bag: {bag_path}")
    print(f"{'='*60}")
    
    try:
        storage = StorageOptions(uri=str(bag_path), storage_id='sqlite3')
        converter = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        reader = SequentialReader()
        reader.open(storage, converter)
        
        # Get metadata
        metadata = reader.get_metadata()
        topic_types = reader.get_all_topics_and_types()
        
        print(f"\nBag info:")
        print(f"  Duration: {metadata.duration.nanoseconds / 1e9:.2f} seconds")
        print(f"  Start time: {metadata.starting_time.nanoseconds / 1e9:.2f}")
        print(f"  Message count: {metadata.message_count}")
        
        print(f"\nTopics:")
        for topic in topic_types:
            print(f"  {topic.name}")
            print(f"    Type: {topic.type}")
        
        # Count messages per topic
        topic_counts = {}
        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            topic_counts[topic] = topic_counts.get(topic, 0) + 1
        
        print(f"\nMessage counts:")
        for topic, count in sorted(topic_counts.items()):
            print(f"  {topic}: {count}")
        
        del reader
        print(f"\n✓ Bag is valid")
        return True
        
    except Exception as e:
        print(f"\n✗ Error validating bag: {e}")
        return False


if __name__ == '__main__':
    # Configuration
    INPUT_BAG = "test_data/V1_01_easy"
    OUTPUT_BAG = "test_data/Test_Baro_Zeros_10s"
    BARO_FREQUENCY = 20.0  # Hz
    DURATION_SECONDS = 10  # None = entire bag, or set to e.g. 30.0 for 30 seconds
    SKIP_TOPICS = []  # Empty list = don't skip any topics. Example: ['/fcu/motor_speed', '/some/topic']
    
    # You can also pass arguments from command line
    if len(sys.argv) > 1:
        INPUT_BAG = sys.argv[1]
    if len(sys.argv) > 2:
        OUTPUT_BAG = sys.argv[2]
    if len(sys.argv) > 3:
        BARO_FREQUENCY = float(sys.argv[3])
    if len(sys.argv) > 4:
        DURATION_SECONDS = float(sys.argv[4])
    
    print("="*60)
    print("ROS2 Bag Copy with Barometer Data")
    print("="*60)
    print(f"Input:  {INPUT_BAG}")
    print(f"Output: {OUTPUT_BAG}")
    print(f"Baro frequency: {BARO_FREQUENCY} Hz")
    if DURATION_SECONDS is not None:
        print(f"Duration limit: {DURATION_SECONDS} seconds")
    else:
        print(f"Duration limit: None (entire bag)")
    if SKIP_TOPICS:
        print(f"Skipping topics: {SKIP_TOPICS}")
    else:
        print(f"Skipping topics: None (all topics copied)")
    print("="*60)
    
    try:
        # Copy bag and add barometer data
        copy_bag_with_baro(
            input_bag_path=INPUT_BAG,
            output_bag_path=OUTPUT_BAG,
            baro_frequency=BARO_FREQUENCY,
            duration_seconds=DURATION_SECONDS,
            skip_topics=SKIP_TOPICS
        )
        
        # Validate the output
        validate_bag(OUTPUT_BAG)
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)