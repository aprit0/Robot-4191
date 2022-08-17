# Robot-4191

# Setup
1. [Ubuntu install](https://roboticsbackend.com/install-ubuntu-on-raspberry-pi-without-monitor/#Setup_Wi-Fi_and_ssh_for_your_Raspberry_Pi_4_without_a_monitor)   
1.5 Install wifi: ```sudo apt-get install network-manager```
2. [Ros install](https://roboticsbackend.com/install-ros2-on-raspberry-pi/#Prerequisites_install_Ubuntu_Server_2004) 
3. ```echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc```
4. ```mkdir -p ~/4191_ws/src && cd 4191_ws/ && colcon build ```   
4.5 [Add git ssh key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
5. ```git git@github.com:aprit0/Robot-4191.git```
6. ```colcon build```   
(((7. ```pip install -r requirements.txt```)))
... Setup finished!

# Tutorials
https://docs.ros.org/en/foxy/Tutorials.html


# TO-DO
- *Done* [Connect ros machines](https://roboticsbackend.com/ros2-multiple-machines-including-raspberry-pi/)

# Connect and run on pi
```  
ssh pi@192.168.217.116  
4191  
```  
4191  
python3 Odometry_1.py  
```
New Tab
```  
4191  
python3 Motor_PWM.py  
```
