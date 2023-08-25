---
title: How to use launch files
---

# How to use launch files

1. Create a new package for the launch files. Nice to have them separated as in the case of interfaces. And also, the launch files will probably use nodes from different packages, so they don't really fit inside any of the other ones.
   ```bash
   ros2 pkg create my_robot_bringup
   ```
1. Remove `/include` and `/src` directories inside the new package.
1. Create a `/launch` directory inside the new package.
1. In the `CMakeLists.txt` file, remove the `Default to C99` section. Also remove the `if(BUILD_TESTING)` block.
1. Add instructions on how to install the launch files (right after `find_package(...)` section):
   ```
   install(DIRECTORY
     launch
     DESTINATION share/${PROJECT_NAME}
   )
   ```
1. Create a launch file inside the `/launch` folder, e g `my_robot_app.launch.py`.
1. Make it executable: `chmod +x launch/my_robot_app.launch.py`.
1. Add the minimal piece of code to the launch file:

   ```python
   from launch import LaunchDescription

   def generate_launch_description():
     ld = LaunchDescription()
     return ld
   ```

1. Build the package and source your terminal. Then run:
   ```bash
   ros2 launch my_robot_bringup my_robot_app.launch.py
   ```
1. Add configuration on how to run your application in the launch file. See `src/my_robot_bringup/launch/number_app.launch.py` for reference.
1. Add exec dependencies to the `package.xml`. See `src/my_robot_bringup/package.xml` for reference.

Use arguments in launch files like this:

- Show available args: `ros2 launch my_bringup robot.launch.py --show-args`
- Pass args: `ros2 launch my_bringup robot.launch.py my_arg:=goodbye`
- Declare in python code: `my_arg = actions.DeclareLaunchArgument("my_arg", default_value="hello")`. And: `ld.add_action(my_arg)`
- Pass to node: `parameters=[{"my_node_arg": LaunchConfiguration("my_arg")}]`.
