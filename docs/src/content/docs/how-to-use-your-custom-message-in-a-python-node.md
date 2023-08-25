---
title: How to use your custom message in a python node
tags:
  - ros2
---

# How to use your custom message in a python node

1. Create a python node
1. Import message: `from my_robot_interfaces.msg import HardwareStatus`
1. (Fix your VS Code python imports as described above)
1. Add to `package.xml`:

```
<depend>my_robot_interfaces</depend>
```

1. You can now use the message as any other message.
