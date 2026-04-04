---
layout: default
title: TurtleBot 4 WiFi and ROS 2 Setup
parent: Tutorials
sort: 12
---

# TurtleBot 4 WiFi and ROS 2 Setup

This guide helps you configure a TurtleBot 4 (Raspberry Pi and Create 3 Base) to operate on a chosen network using a Fast DDS Discovery Server listening on all interfaces.

> Note: If you face any issues during the setup, start a Canvas discussion.


## 1. Preparation: Switch to AP Mode
We will clear the existing WiFi configuration on the Raspberry Pi (RPi) to force it into Access Point (AP) mode. In this mode, the robot broadcasts its own WiFi network for your computer to join.

> **Note:** If you need a MicroSD card adapter, please ask your instructor for a loaner.

### MicroSD Manual Configuration

```note
Please watch the video which shows how to open and close the Turtlebot4 tray.
```

<video width="720" controls>
  <source src="../assets/video/turtlebot4_tray_handling.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

1. Carefully pull out the TurtleBot compute tray.
2. The MicroSD card slot is located on the side of the RPi.
3. Gently remove the card by pulling it sideways; avoid applying pressure to the top or bottom of the card.
4. Insert the SD card into your computer.
5. Navigate to `/etc/netplan/` on the SD card and open `50-cloud-init.yaml` in a text editor.
6. **Remove all lines** starting from (and including) the line `wifis:`. The file should look like below:
```bash
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    eth0:
      optional: true
      dhcp-identifier: "mac"
      dhcp4: true
    usb0:
      addresses:
      - "192.168.186.3/24"
      dhcp4: false
```
7. Save the file. 
8. Eject the MicroSD card safely from your operating system before physical removal.
9. Slide the MicroSD card back into the RPi with the metal contacts facing **up** and pointing toward the RPi board (as shown in the video above).
10.  Turn carefully and slide the tray back in to the turtltebot, and make sure you didn't pull out any wires.

## 2. Connect and SSH into the TurtleBot
1. Power on the robot and wait around 3 minutes for the boot sequence to complete.
2. The TurtleBot should now broadcast a network with the SSID **"Turtlebot4"**. 
3. Connect your computer to this network using the password: **Turtlebot4**
4. Open your terminal and SSH into the robot:
   ```bash
   ssh ubuntu@10.42.0.1
   ```
**You now have SSH access to the turltebot.**

## 3. Setup ROS 2 Configuration

### 3.1. Reset Namespace and Domain ID
We will remove the specific `/robot_xx` namespace and set the Domain ID to 0. 

Domain IDs are used to isolate robots on the same network into their own "networksilos". This prevents you from accidentally controlling someone else's robot or wasting network bandwidth by processing data from other robots' topics.

Since we are no longer in the lab, resetting the namespace to empty and the Domain ID to 0 simplifies communication and compatibility with most standard ROS 2 packages that you will use for your project. 

**This configuration must be updated on both the Raspberry Pi and the Create 3 Base.**

#### Step A: Configure Raspberry Pi
Run the following commands to update the ROS environment variables:
```bash
sudo sed -i 's/export ROS_DOMAIN_ID=.*/export ROS_DOMAIN_ID="0"/' /etc/turtlebot4/setup.bash
sudo sed -i 's|export ROBOT_NAMESPACE=.*|export ROBOT_NAMESPACE=""|' /etc/turtlebot4/setup.bash
```

#### Step B: Configure Create 3 Base
1. **Verify Connectivity:** Run this command in a terminal to ensure the RPi can communicate with the Base:
   ```bash
   bash -c "for i in {1..20}; do if ping -c 1 -W 1 192.168.186.2 > /dev/null && curl -sk http://192.168.186.2/api/about | grep -qi 'I.0.0.FastDDS'; then echo 'Create 3 Base I.0.0 found'; break; fi; echo 'Waiting for Base... attempt \$i'; sleep 5; done"
   ```
   > **Note**: If you see the terminal print out `Create 3 Base I.0.0 found`, it was successful. If you didn't, don't proceed with the rest of the steps and cotact the Instructor.

2. **Apply Configuration:** Once the Base is found, use the Web API to update its ROS settings:
   ```bash
   curl -X POST -d "ros_domain_id=0&ros_namespace=/_do_not_use&rmw_implementation=rmw_fastrtps_cpp&fast_discovery_server_value=192.168.186.3:11811&fast_discovery_server_enabled=true" http://192.168.186.2/ros-config-save-main
   sleep 15
   ```

3. **Reboot Create 3 Base:** Trigger the Base to reboot so the new settings can take effect:
   ```bash
   curl -X POST [http://192.168.186.2/api/reboot](http://192.168.186.2/api/reboot)
   ```

## 4. Configure Discovery Server
The Discovery Server manages how ROS nodes find each other. By default, it often listens only on a local IP. We will change it to listen on `0.0.0.0` so that your remote workstation can also connect to the discovery stream.

While the Create 3 Base reboots, you can still work on your RPi. Update the discovery startup script:
```bash
sudo sed -i 's/^fastdds discovery.*/fastdds discovery -i 0 -l 0.0.0.0 -p 11811/' /usr/sbin/discovery
sleep 90
```

## 5. Optional: Connect to External WiFi (Client Mode)
```warning
Skip this section if you prefer to stay connected directly to the "Turtlebot4" SSID.
```

If you want the robot to join your home WiFi network instead of broadcasting its own:
1. Run the setup tool:
   ```bash
   turtlebot4-setup
   ```
2. Navigate to **Wi-fi Setup** -> **Wi-Fi Mode** and select **[Client]**.
3. Enter the **SSID**, **Password**, and **Band** for your local network.
4. Leave **IP Address** blank and ensure **DHCP** is set to **True**.
5. Select **Save**, then press **ESC** to return to the Main Menu.
6. Select **Apply Settings** and confirm.
7. Select **Exit**.

## 6. Final Restart
To ensure all changes are committed to both the Create 3 Base and Rpi, lets do a final shutdown of all the hardware.
1. Shut down the RPi:
   ```bash
   sudo shutdown -h now
   ```
2. Physically switch off the Create 3 Base using the power button.
3. Wait 10 seconds, then power the robot back on. 

Your TurtleBot 4 is now ready for your project! Test it out by looking at the lidar data and making the robot move.
