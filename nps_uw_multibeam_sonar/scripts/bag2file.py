#!/usr/bin/env python3

##################################################
# Translate and dissect data from ROS2 bag file
##################################################

import os
import shutil

from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions
from rosbag2_py import ConverterOptions

from rclpy.serialization import deserialize_message

from rosidl_runtime_py.utilities import get_message


##################################################
# TARGET MESSAGE TYPE
##################################################

TARGET_MSG_TYPE = 'acoustic_msgs/msg/SonarImage'

##################################################
# FIND ALL ROS2 BAG DIRECTORIES
##################################################

bag_list = []

for item in os.listdir('./'):

    if os.path.isdir(item):

        if os.path.exists(
            os.path.join(item, 'metadata.yaml')
        ):

            bag_list.append(item)

##################################################
# LOOP THROUGH BAG FILES
##################################################

for file_i, bag_name in enumerate(bag_list):

    print(
        f'Translating {bag_name}... '
        f'({file_i+1}/{len(bag_list)})'
    )

    ##################################################
    # OPEN BAG
    ##################################################

    storage_options = StorageOptions(

        uri=bag_name,

        storage_id='sqlite3'
    )

    converter_options = ConverterOptions(

        input_serialization_format='cdr',

        output_serialization_format='cdr'
    )

    reader = SequentialReader()

    reader.open(
        storage_options,
        converter_options
    )

    ##################################################
    # GET TOPICS
    ##################################################

    topic_types = reader.get_all_topics_and_types()

    topic_name = None
    topic_type = None

    for topic_info in topic_types:

        if topic_info.type == TARGET_MSG_TYPE:

            topic_name = topic_info.name
            topic_type = topic_info.type

            break

    if topic_name is None:

        print(
            f'No sonar topic found in {bag_name}'
        )

        continue

    ##################################################
    # CREATE OUTPUT DIRECTORY
    ##################################################

    if os.path.exists(bag_name):

        shutil.rmtree(bag_name)

    os.makedirs(bag_name)

    ##################################################
    # LOAD MESSAGE TYPE
    ##################################################

    msg_class = get_message(topic_type)

    ##################################################
    # READ AND SAVE DATA
    ##################################################

    counter = 0

    while reader.has_next():

        topic, data, t = reader.read_next()

        if topic != topic_name:

            continue

        msg = deserialize_message(
            data,
            msg_class
        )

        counter += 1

        frame_dir = os.path.join(
            bag_name,
            str(counter)
        )

        os.makedirs(frame_dir)

        ##################################################
        # WRITE SEQUENCE
        ##################################################

        with open(
            os.path.join(frame_dir, 'sequence'),
            'w'
        ) as f:

            f.write(str(msg.header.seq))

        ##################################################
        # WRITE TIME
        ##################################################

        with open(
            os.path.join(frame_dir, 'time'),
            'w'
        ) as f:

            f.write(
                str(
                    msg.header.stamp.sec
                )
            )

        ##################################################
        # WRITE FREQUENCY
        ##################################################

        with open(
            os.path.join(frame_dir, 'frequency'),
            'w'
        ) as f:

            f.write(str(msg.frequency))

        ##################################################
        # WRITE SOUND SPEED
        ##################################################

        with open(
            os.path.join(frame_dir, 'sound_speed'),
            'w'
        ) as f:

            f.write(str(msg.sound_speed))

        ##################################################
        # WRITE AZIMUTH BEAMWIDTH
        ##################################################

        with open(
            os.path.join(
                frame_dir,
                'azimuth_beamwidth'
            ),
            'w'
        ) as f:

            f.write(
                str(
                    msg.azimuth_beamwidth
                )
            )

        ##################################################
        # WRITE ELEVATION BEAMWIDTH
        ##################################################

        with open(
            os.path.join(
                frame_dir,
                'elevation_beamwidth'
            ),
            'w'
        ) as f:

            f.write(
                str(
                    msg.elevation_beamwidth
                )
            )

        ##################################################
        # WRITE AZIMUTH ANGLES
        ##################################################

        with open(
            os.path.join(
                frame_dir,
                'azimuth_angles'
            ),
            'w'
        ) as f:

            for value in msg.azimuth_angles:

                f.write(f'{value}\n')

        ##################################################
        # WRITE ELEVATION ANGLES
        ##################################################

        with open(
            os.path.join(
                frame_dir,
                'elevation_angles'
            ),
            'w'
        ) as f:

            for value in msg.elevation_angles:

                f.write(f'{value}\n')

        ##################################################
        # WRITE RANGES
        ##################################################

        with open(
            os.path.join(
                frame_dir,
                'ranges'
            ),
            'w'
        ) as f:

            for value in msg.ranges:

                f.write(f'{value}\n')

        ##################################################
        # WRITE BIGENDIAN
        ##################################################

        with open(
            os.path.join(
                frame_dir,
                'is_bigendian'
            ),
            'w'
        ) as f:

            f.write(
                str(
                    msg.is_bigendian
                )
            )

        ##################################################
        # WRITE DATA SIZE
        ##################################################

        with open(
            os.path.join(
                frame_dir,
                'data_size'
            ),
            'w'
        ) as f:

            f.write(
                str(
                    msg.data_size
                )
            )

        ##################################################
        # WRITE INTENSITIES
        ##################################################

        with open(
            os.path.join(
                frame_dir,
                'intensities'
            ),
            'w'
        ) as f:

            for value in msg.intensities:

                if isinstance(value, int):

                    f.write(f'{value}\n')

                else:

                    f.write(
                        f'{ord(value)}\n'
                    )

    print(
        f'Finished {bag_name}'
    )
