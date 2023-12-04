# ROSA

This repository contains ROSA, a knowledge-based framework for robotics self-adaptation.
ROSA is implemented as a ROS 2-based system, with its knowledge base implemented with TypeDB.

This is still work in progress, therefore the repository is unstable.

This package was tested with ROS 2 Humble and TypeDB 2.24.17

## Installing

[Install ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)

Install TypeDB:

```Bash
sudo apt install software-properties-common apt-transport-https gpg
gpg --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 8F3DA4B5E9AEF44C
gpg --export 8F3DA4B5E9AEF44C | sudo tee /etc/apt/trusted.gpg.d/vaticle.gpg > /dev/null
echo "deb https://repo.vaticle.com/repository/apt/ trusty main" | sudo tee /etc/apt/sources.list.d/vaticle.list > /dev/null

sudo apt update
sudo apt install openjdk-11-jre
sudo apt install typedb-server=2.24.17 typedb-console=2.24.15 typedb-bin=2.24.16
pip3 install typedb-driver==2.24.15
```

Download ROSA:
```Bash
mkdir -p ~/rosa_ws/src
cd ~/rosa_ws/src
git clone git@github.com:kas-lab/rosa.git
```

If you want to be able to run tests, get ros_pytest:
```Bash
cd ~/rosa_ws/src
git clone git@github.com:kas-lab/ros_pytest.git
```

Install dependencies:
```Bash
cd ~/rosa_ws/
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Build ROSA:
```Bash
cd ~/rosa_ws/
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Running

Start typedb:

```Bash
typedb server
```

Run ROSA:
```Bash
ros2 launch rosa_bringup rosa_bringup.launch.py
```

## Tests

Start typedb:

```Bash
typedb server
```

```Bash
colcon test --event-handlers console_cohesion+ --packages-select rosa_kb rosa_plan rosa_execute
```

## Acknowledgments

<a href="https://remaro.eu/">
    <img height="60" alt="REMARO Logo" src="https://remaro.eu/wp-content/uploads/2020/09/remaro1-right-1024.png">
</a>

This work is part of the Reliable AI for Marine Robotics (REMARO) Project. For more info, please visit: <a href="https://remaro.eu/">https://remaro.eu/

<br>

<a href="https://research-and-innovation.ec.europa.eu/funding/funding-opportunities/funding-programmes-and-open-calls/horizon-2020_en">
    <img align="left" height="60" alt="EU Flag" src="https://remaro.eu/wp-content/uploads/2020/09/flag_yellow_low.jpg">
</a>

This project has received funding from the European Union's Horizon 2020 research and innovation programme under the Marie Skłodowska-Curie grant agreement No. 956200.
