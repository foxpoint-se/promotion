---
title: How to create a simple CPP node
description: Empty CPP node, getting started.
tags:
  - c++
  - ros2
---

# How to create a simple CPP node

1. `docker-compose exec dev bash`
1. `cd src`
1. `ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp`
1. Create a CPP file: `touch my_cpp_pkg/src/my_first_node.cpp`
1. Edit the file and add:

   ```cpp
   #include "rclcpp/rclcpp.hpp"

   int main(int argc, char **argv)
   {
     rclcpp::init(argc, argv);
     auto node = std::make_shared<rclcpp::Node>("cpp_test");
     RCLCPP_INFO(node->get_logger(), "Hello Cpp Node");
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
   }
   ```

1. Add to `CMakeLists.txt`:

   ```
   // find_package...

   add_executable(cpp_node src/my_first_node.cpp)
   ament_target_dependencies(cpp_node rclcpp)

   install(TARGETS
     cpp_node
     DESTINATION lib/${PROJECT_NAME}
   )

   // ament_package...
   ```

1. Run the node with: `/ros2_ws/install/my_cpp_pkg/lib/my_cpp_pkg/cpp_node`
1. Source: `source ~/.bashrc`
1. Run with ROS2 command: `ros2 run my_cpp_pkg cpp_node`
