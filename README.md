# Cellumation Snapshotter
![CircleCI](https://img.shields.io/circleci/build/github/cellumation/snapshotter)
![GitHub](https://img.shields.io/github/license/cellumation/snapshotter)

A ROS2 node that subscribes to a list of topics and buffers the messages in a ring buffer. It provides a service to dump the buffer contents into a bag file.
This is a powerful tool for post mortem analysis, debugging and testing of large long running ros systems.

## Features
- Very little memory overhead. (10GB of ram usage result in a 9.8gb bag file)
- Automatically subscribes to new topics as they become available
- Keep latched (transient local) topics
- Exclude topics based on regex
- Include topics based on regex
- Configurable memory usage limit
- Compression of bag files
- Limit cpu usage when compressing/writing by setting the nice value

## Introduction
The idea of the snapshotter was taken from rosbag_snapshot (https://github.com/ros/rosbag_snapshot).
Initially we maintained a fork of rosbag_snapshot but we forked at such an early time and digressed so far away from the original that there was no chance of ever merging back. Over time our fork became hard to manage and messy thus we eventually decided on a complete re-implementation that contains only the features that we need.

## Usage
### Launching
An example launch file and configuration are provided. The configuration is located in the config folder.
```
roslaunch2 snapshotter example.launch
```

### Parameters
```
        # The frequency in hz used to process incomming messages.
        # This also defines the resolution of the receive timestamps, as
        # they are coupled to the time of processing.
        # A higher frequency results in more CPU load so this parameter controls
        # the tradeoff between the accuracy of the timestamps vs the CPU load.
        process_frequency: 1000.0

        # Maximum memory to be used for buffering by the snapshotter (in mb)
        # The snapshotter will take at least max_memory_mb of additional
        # memory while writing a snapshot to disk!
        max_memory_mb: 5000

        # allowed values for bag_compression are "none", "slow", "fast"
        # "none" = no compression, largest file size but very fast writing
        # "slow" = best compression, smallest file size but takes a lot of time to write
        # "fast" = some compression, medium file size but reasonably fast
        bag_compression: "fast"

        # If true the snapshotter will switch the nice value of the thread that is
        # writing a bag file to disk to 19. I.e. the thread will yield to everyone
        # with a higher priority. This enables writing to disk without interrupting
        # other processes.
        nice_on_write: True

        # A list of regexes that should be excluded from logging. Every topic that
        # matches a regex in this list is not logged.
        exclude_topics:
        - /camera_\d+/depth/.*
        - /some/other/topic
        - /or/some/other/regexp.*

        # A list of regexes that should be include even though it matches the exclude pattern.
        include_topics: ['']

```
