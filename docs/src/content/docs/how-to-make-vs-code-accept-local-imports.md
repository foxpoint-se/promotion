---
title: How to make VS Code accept your local imports
---

# How to make VS Code accept your local imports

1. Open User settings
1. Search for "python path"
1. Click "Edit in settings.json" under "Auto complete: extra paths"
1. Add to the array `python.autoComplete.extraPaths` for example:

```
"~/learning-ros2/install/my_robot_interfaces/lib/python3.8/site-packages/my_robot_interfaces"
```
