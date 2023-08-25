---
title: How to use parameters
description: Getting started with parameters in ROS2.
tags:
  - ros2
---

# How to use parameters

Declare and use parameters in the constructor of the node like this:

```python
self.declare_parameter("some_name", "default_value")
self.my_value = self.get_parameter("some_name").value
```

```cpp
this->declare_parameter("some_name", 100);
int my_value = this->get_parameter("some_name").as_int();
```

Set the parameters during runtime:

```bash
ros2 run my_pkg my_node --ros-args -p some_name:=the_value
```

The type of the value will be automatically interpreted by ROS2 - int, string, double and so on.

List and get the parameters during runtime:

```bash
ros2 param list
```

```bash
ros2 param get /the_node the_param
```
