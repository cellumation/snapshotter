# Cellumation Snapshotter
A ROS node that subscribes to a list of topics and buffers the messages in a ring buffer. It provides a service to dump the buffer contents into a bag file.
This is a powerful tool for post mortem analysis, debugging and testing of large long running ros systems.

## Features
- Very little memory overhead. (10GB of ram usage result in a 9.8gb bag file)
- Automatically subscribes to new topics as they become available
- Keep latched topics
- Exclude topics based on regex
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
roslaunch snapshotter example.launch
```

### Parameters
```
num_threads: 4

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
  - /vision/camera_\d+/depth/.*
  - /some/topic


```

## Performance Evaluation and Comparision
We evaluated three simple scenarios to evaluate the overall performance of our implementation.
All tests are done on AMD Ryzen 7 3800X 8 Core processor.

#### Scenario 1
Run a gazebo simulation with 5 depth cameras and several small topics.
Limit memory usage to 5GB.

|                    | rosbag_snapshot | snapshotter |
|--------------------|-----------------|-------------|
| cpu usage avg %    | 56%             | 52%         |
| Ram usage          | 5.1GB           | 5.8gb       |
| Resulting Bag Size | 4.0GB           | 4.9GB       |
| Resulting Bag Duration | 4.9s           | 5.7s       |

#### Scenario 2
Run a gazebo simulation with 5 depth cameras and several small topics.
Limit memory usage to 10GB.

|                    | rosbag_snapshot | snapshotter |
|--------------------|-----------------|-------------|
| cpu usage avg %    | 56%             | 52%         |
| Ram usage          | 9.8GB           | 10.0GB      |
| Resulting Bag Size | 8.4GB           | 9.8GB       |
| Resulting Bag Duration | 9.6s           | 10.7s       |

#### Scenario 3
Run a gazebo simulation with only small topics.
Limit memory usage to 10GB.

|                    | rosbag_snapshot | snapshotter |
|--------------------|-----------------|-------------|
| cpu usage avg %    | 10%             | 8%          |
| Ram usage          | 9.8GB           | 10.0GB      |
| Resulting Bag Size | 6.8GB           | 9.8GB       |
| Resulting Bag Duration | 619s          | 954s       |

#### Results
- There is no significant difference in cpu usage between rosbag_snapshot and our implementation
- When logging mostly very large messages (e.g. images) the difference in memory efficiency is small
- When logging mostly small messages the difference in memory efficiency is significant
- The main source of cpu usage for both implementation is message size. The bigger the messages the higher the cpu load.
