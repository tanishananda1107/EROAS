#!/usr/bin/env python3

##################################################
### Translate and dissect data from ROS2 bag  ###
##################################################

import os
import shutil

from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions
from rosbag2_py import ConverterOptions

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TARGET_TOPIC = "/raven/oculus/sonar_image"

# ROS2 bag directories
bag_list = [
    d for d in os.listdir(".")
    if os.path.isdir(d)
]

for file_i, bag_name in enumerate(bag_list):

    print(
        f"Translating {bag_name}... "
        f"({file_i + 1}/{len(bag_list)})"
    )

    reader = SequentialReader()

    storage_options = StorageOptions(
        uri=bag_name,
        storage_id="sqlite3"
    )

    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()

    topic_type_map = {
        topic.name: topic.type
        for topic in topic_types
    }

    topic_name = TARGET_TOPIC

    for topic_info in topic_types:
        if "SonarImage" in topic_info.type:
            topic_name = topic_info.name
            break

    msg_type = get_message(topic_type_map[topic_name])

    if os.path.exists(bag_name):
        shutil.rmtree(bag_name)

    os.makedirs(bag_name)

    counter = 0

    while reader.has_next():

        topic, data, timestamp = reader.read_next()

        if topic != topic_name:
            continue

        msg = deserialize_message(data, msg_type)

        counter += 1

        seq_dir = os.path.join(
            bag_name,
            str(counter)
        )

        os.makedirs(seq_dir, exist_ok=True)

        #
        # sequence
        #
        with open(
            os.path.join(seq_dir, "sequence"),
            "w"
        ) as f:

            if hasattr(msg.header, "seq"):
                f.write(str(msg.header.seq))
            else:
                f.write(str(counter))

        #
        # time
        #
        with open(
            os.path.join(seq_dir, "time"),
            "w"
        ) as f:

            f.write(str(msg.header.stamp.sec))

        #
        # frequency
        #
        with open(
            os.path.join(seq_dir, "frequency"),
            "w"
        ) as f:

            f.write(str(msg.frequency))

        #
        # sound_speed
        #
        with open(
            os.path.join(seq_dir, "sound_speed"),
            "w"
        ) as f:

            f.write(str(msg.sound_speed))

        #
        # azimuth_beamwidth
        #
        with open(
            os.path.join(seq_dir, "azimuth_beamwidth"),
            "w"
        ) as f:

            f.write(str(msg.azimuth_beamwidth))

        #
        # elevation_beamwidth
        #
        with open(
            os.path.join(seq_dir, "elevation_beamwidth"),
            "w"
        ) as f:

            f.write(str(msg.elevation_beamwidth))

        #
        # azimuth_angles
        #
        with open(
            os.path.join(seq_dir, "azimuth_angles"),
            "w"
        ) as f:

            for value in msg.azimuth_angles:
                f.write(f"{value}\n")

        #
        # elevation_angles
        #
        with open(
            os.path.join(seq_dir, "elevation_angles"),
            "w"
        ) as f:

            for value in msg.elevation_angles:
                f.write(f"{value}\n")

        #
        # ranges
        #
        with open(
            os.path.join(seq_dir, "ranges"),
            "w"
        ) as f:

            for value in msg.ranges:
                f.write(f"{value}\n")

        #
        # is_bigendian
        #
        with open(
            os.path.join(seq_dir, "is_bigendian"),
            "w"
        ) as f:

            f.write(str(msg.is_bigendian))

        #
        # data_size
        #
        with open(
            os.path.join(seq_dir, "data_size"),
            "w"
        ) as f:

            f.write(str(msg.data_size))

        #
        # intensities
        #
        with open(
            os.path.join(seq_dir, "intensities"),
            "w"
        ) as f:

            for value in msg.intensities:
                f.write(f"{int(value)}\n")
