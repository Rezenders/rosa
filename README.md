# TypeDB implementation of Metacontrol's new Knowledge Base

This repository contains a proposal for a new Knowledge Base for Metacontrol.
The KB is implemented as a ROS 2 package, and the Knowledge Model is implemented with TypeDB.

This is still work in progress, not all required features are implemented yet, and the repository is unstable.

## Running

For now:

```Bash
typedb server
```

```Bash
cd src/
./metacontrol_uio.py
```

## Tests

run tests:

```Bash
typedb server
```

```Bash
colcon test --event-handlers console_cohesion+ --packages-select metacontrol_kb
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
