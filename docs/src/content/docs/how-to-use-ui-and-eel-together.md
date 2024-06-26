---
title: How to use the UI and Eel together
description: Complete guide for how to get started with the Eel and connection through Ethernet.
tags:
  - ros2
  - Eel
date: 2023-06-04
updated: 2024-04-17
---

# How to use the UI and Eel together

## Boot stuff

1. Start the Eel Raspberry PI.
1. Start a computer with Ubuntu installed.

## Connection

### Ethernet

1. Make sure you have a ethernet cable connected between the two.
1. Open `Wired network settings` on your computer:
   1. Top right corner.
   1. Click arrow.
   1. Click `Settings`.
   1. Click `Network`.
   1. Under `Wired`, click cog wheel.
1. Go to `IPv4` tab.
1. Select `Manual` under `IPv4 Method`.
1. In the first row under `Addresses`, enter:
   - Address: `192.168.0.100`
   - Netmask: `255.255.255.0`
1. Click `Apply` and close the `Network settings` dialog.
1. Open a terminal:
   - Use for example `Tabby`, either from the `Favorites` panel to the left, or
   - by searching for `Tabby` after pressing the `Windows` button on your keyboard.
1. Type `ssh eel` and press `enter`.
1. If you get something like
   ```bash
   ubuntu@ubuntu >
   ```
   you're good. If you get anything else, I can't help you :)

## Start applications

You will need to have three terminals running, which is why it's good to use `Tabby`, since you can see them all at once.

We will refer to the three terminals as:

- `Remote to RPi 1` (the one which you just connected to over SSH).
- `Remote to RPi 2` (another SSH session just like the one above, which will be running `ROS web bridge`, so that the user interace can communicate with ROS).
- `Ground control` (which will be running the user interface).

## Do this

1. In the `Remote to RPi 1` terminal (which you just setup in the previous section), do the following:
   1. `cd eel`
   1. `git checkout main`
   1. `git pull`
   1. `source source_me.sh`
   1. `make build-sym`
   1. `make start-pid-rudder`
   1. You should now see some output saying that a few nodes have started. If you get any errors, I can't help you :)
1. In another terminal, let's say `Remote to RPi 2`, do the following:
   1. `ssh eel`
   1. `cd eel/docker`
   1. `dc up -d`
   1. `dc logs -f`
   1. You should see Docker starting up and then something like `Rosbridge WebSocket server started...`.
1. In the third terminal, let's say `Ground control`, do the following:
   1. `cd ~/code/ground-control`
   1. `git checkout main`
   1. `git pull`
   1. `make setup`
   1. `make dev`
   1. You should see something like `started server on...`.

At this point (assuming no errors), everything should be up and running, so you can start sending commands from the web user interface.

## Use the web user interface

1. Open a web browser.
1. Navigate to http://localhost:3000.
1. Select "Go to ROS Bridge"
1. If there are no backends to select, then click "Add backend".
   1. Name -> E. g. "Ethernet".
   1. Address -> `192.168.0.101`.
   1. Click "Add backend".
1. Now that there is a backend to select, click "Use backend".

## Shut down

When you're done testing stuff, follow these steps to shut everything down properly:

1. In the `Ground control` terminal, press `Ctrl C`
1. In the `Remote to RPi 1` terminal, press `Ctrl C`
1. In the `Remote to RPi 2` terminal, press `Ctrl C`
   1. Then `dc down`
   1. Then `sudo shutdown -h now`
1. Kill the power to the Eel.
1. Shut down your computer.
