---
title: How to create a simple Python node
description: Empty Python node, getting started.
tags:
  - python
  - ros2
---

# How to create a simple Python node

1. `docker-compose exec dev bash`
1. `cd src`
1. `ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy`
1. Create a Python file: `touch my_py_pkg/my_py_pkg/my_first_node.py`
1. Edit the file and add:

   ```python
   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node


   def main(args=None):
       rclpy.init(args=args)
       node = Node('py_test')
       node.get_logger().info('Hello ROS2')
       rclpy.spin(node)
       rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

1. Make the file executable: `chmod +x the_python_file.py`. Then you can execute by running `./the_python_file.py`
1. Add your node to `src/my_py_pkg/setup.py`
   ```python
   entry_points={
       'console_scripts': [
           'py_node = my_py_pkg.my_first_node:main' # <-- this line
       ],
   },
   ```
1. Go to the root of your workspace: `cd /ros2_ws`
1. Build your package: `colcon build --packages-select my_py_pkg --symlink-install`
1. Source again: `source ~/.bashrc`
1. Run your node manually: `./install/my_py_pkg/lib/my_py_pkg/py_node`
1. Run your node through ROS2 commands (source `~/.bashrc` again if necessary): `ros2 run my_py_pkg py_node`
