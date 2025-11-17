Differences between launch files:

LS = laserscan

PC = pointcloud

## For experiments

- `experiment_**.py`

    Subscribes to the nominal topics ( "/laserscan" and "/livox/lidar" ) using the parameters defined in "sink_**.yaml"





## For testing

### Inter-container

In container1 launch 

-   `only_chatter.py`

    It will take the parameters defined in "fake_source.yaml"

In container2 launch 

-   `listener_inter.py`

    It will take the parameters defined in "fake_source.yaml"

### Intra-container

2 options:

1. Repeat the commands from the Inter-container tests

1. Launch the `with_chatter.py`

    It will take the parameters defined in "fake_source.yaml", and launch both the listener and the chatter.