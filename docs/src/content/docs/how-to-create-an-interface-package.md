---
title: How to create an interface package
---

# How to create an interface package

Create a separate package for message and service definitions, for easier dependency management later on. This package does not need any dependencies, since it won't have any program code. So the first `pkg create` command does not have any dependencies. However, some changes need to be done to the `package.xml` and `CMakeLists.txt` files.

- `ros2 pkg create my_interfaces`
- `cd my_interfaces`
- `rm -rf include/`
- `rm -rf src/`
- `mkdir msg`

Open `package.xml` and add between `<buildtool_depend>` and `<test_depend>`:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Edit `CMakeLists.txt` like this:

- Remove `# Default to C99` section.
- Remove `if(BUILD_TESTING)` block.
- Add to `# find dependencies` section:
  - `find_package(rosidl_default_generators REQUIRED)`

Add file to `msg` folder, e. g. `SomeStatus.msg`:

```
int64 some_int_value
bool is_ready
string debug_message
```

Add to `CMakeLists.txt`, after `# find dependencies` section:

```
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SomeStatus.msg"
)
```

Build the package:

```sh
cd /root/of/project
colcon build --packages-select my_interfaces
```

Verify that it's there:

```sh
source ~/.bashrc
ros2 interface show my_interfaces/msg/SomeStatus
```
