# Robot-4191

# Setup
1. [Ubuntu install](https://roboticsbackend.com/install-ubuntu-on-raspberry-pi-without-monitor/#Setup_Wi-Fi_and_ssh_for_your_Raspberry_Pi_4_without_a_monitor)
2. [Ros install](https://roboticsbackend.com/install-ros2-on-raspberry-pi/#Prerequisites_install_Ubuntu_Server_2004) 
3. ```echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc```
4. ```mkdir -p ~/4191_ws/src && cd 4191_ws/ && colcon build ```
4.5 [Add git ssh key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
5. ```git git@github.com:aprit0/Robot-4191.git```
6. ```colcon build```
7. ```pip install -r requirements.txt```
... Setup finished!

# Tutorials
https://docs.ros.org/en/foxy/Tutorials.html


# TO-DO
- Connect ros machines: https://roboticsbackend.com/ros2-multiple-machines-including-raspberry-pi/
