# TypeDB implementation of Metacontrol's new Knowledge Base

This repository contains a proposal for a new Knowledge Base for Metacontrol.
The KB is implemented as a ROS 2 package, and the Knowledge Model is implemented with TypeDB.

This is still work in progress, not all required features are implemented yet, and the repository is unstable.

## Tests

run tests:

```Bash
typedb server
```

```Bash
colcon test --event-handlers console_cohesion+ --packages-select metacontrol_kb
```
