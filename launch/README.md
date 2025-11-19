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

## Long mode

Launch the source (either the real driver or the chatter)

-   `only_chatter.py`

    It will take the parameters defined in "fake_source.yaml"


- launch the long mode logger with:

```bash
for i in $(seq -f "%03g" 1 18);
do
    ros2 launch topic_tester experiment_**_long.py file_output_prefix:="output/latency_ls_inter_fake_long_$i" 
    sleep 600
done
```

It will take the parameters from inside the launch file `experiment_**_long.py`

In this case the sequence will take 18 experiments, with an interval of 10 minutes between them, totalling almost 3 hours.
