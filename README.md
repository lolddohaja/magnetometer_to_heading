# magnetometer_to_heading

A package that generates an orientation value as a topic based on data from the magnetometer


default ROS2 iron check

```bash
docker build -t zetabank/outside:iron-heading-v0.0.1 -f Dockerfile .
```


```bash
# ros2 run magnetometer_to_heading magnetometer_to_heading
docker run --rm -it zetabank/outside:iron-heading-v0.0.1 ros2 run magnetometer_to_heading magnetometer_to_heading
```