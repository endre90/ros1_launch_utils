# ros1_launch_utils

A ROS1 package providing instructions and examples on utilities such as remote and timed roslaunch.

### Prerequisites

In a multi machine scenario, it is important to have a properly setup ROS1 network. Here are some steps on how to ensure that:

1.  Setting IP addresses. It is a good practice to have fixed addresses for remote machines. If an ethernet switch is used, this comes naturally. In the case that a router is used and the machines are phisicaly connected with patch cables, disabling wifi in the remote machines and reserving fixed IP's in the router's configuration for their MAC addresses is easiest. Another option is to alter the dhcp configuration file. If using Raspbian:
'''
sudo nano /etc/dhcpcd.conf
'''
and add:
'''
interface eth0
static ip_address=xxx.xxx.x.xxx/24
static routers=yyy.yyy.y.yyy
static domain_name_servers=yyy.yyy.y.yyy
'''
noting that your interface name might not be called "eth0", where xxx.xxx.x.xxx is the IP address that you want to set your device to (make sure that you leave the /24 at the end), and yyy.yyy.y.yyy is the router's IP. You can add multiple IP addresses here separated with a single space. Else, if using Ubuntu:
'''
sudo nano /etc/dhcp/dhclient.conf 
'''
and add:
'''
alias {
  interface "eth0";
  fixed-address xxx.xxx.x.xxx;
  option subnet-mask zzz.zzz.zzz.zzz;
}

'''

2. Disabling other networks. If the remote machines are connected to the ROS netwotk wirelessly, beside doing step 1 with "wlan0" instead of "eth0", it is a good idea to disable other networks. In the case of connection failure, the machines can connect to other wireless networks if available. To prevent this, if using Rapbian:
'''
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
''' 
and remove all network sections leaving only but one, the ROS Network. After that, the file should look something like this:
'''
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=GB

network={
    ssid="xxxxxxxxx"
    psk="yyyyyyyyy"
    key_mgmt=WPA-PSK
}
'''
where xxxxxxxxx is the ROS Network’s name and yyyyyyyyy is the access password.

3. Defining hostnames. It is also recommended to define hostnames for all machines in all machines, so that they can find each other based on names. Do:
'''
sudo nano /etc/hosts
'''
and add:
'''
xxx.xxx.x.xxx    machine_1_hostname
yyy.yyy.y.yyy    machine_2_hostname
zzz.zzz.z.zzz    machine_3_hostname
and so on...
'''

4. Deleting ROS logs. In some cases, depending on the task of the remote machine, the ROS log files can easily fill the available storage space. To prevent this, check the size of ros logs regularly with:
'''
rosclean check
'''
and delete them with:
'''
rosclean purge
'''
It is also good to automate this by adding this command to a crontab, or if the ROS environment is not sources all the time, by adding:
'''
sudo rm -rf /home/username/.ros/log
'''

5. Defining a rosmaster. On the machine that is chosen to be the rosmaster:
'''
sudo nano ~/.bashrc
'''
and add:
'''
export ROS_MASTER_URI=http://xxx.xxx.x.xxx:11311
export ROS_HOSTNAME=yyyyyy
export ROS_IP=xxx.xxx.x.xxx
'''
where xxx.xxx.x.xxx is the rosmaster’s IP address and yyyyyy is the rosmaster’s hostname that you have defined both in the /etc/hosts and the /etc/hostname files. On all other machines, also open the ./bashrc file and add:
'''
export ROS_MASTER_URI=http://xxx.xxx.x.xxx:11311
export ROS_HOSTNAME=nnnnnn
export ROS_IP=zzz.zzz.z.zzz
'''
where xxx.xxx.x.xxx is the rosmaster’s IP address, nnnnnn is the current machine’s hostname that you have defined both in the /etc/hosts and the /etc/hostname files and zzz.zzz.z.zzz is that machines IP address.

### Remote roslaunch setup

Now that a ROS network is defined with a rosmaster, it is convenient to have the ability to launch ROS nodes on remote machines. Here are some steps on how to do that:

1. Generate a "rsa" key with:
'''
ssh-keygen -t rsa
'''

2. Copy that key to the remote machine with:
'''
ssh-copy-id username@hostname
'''

3. Delete previous non-rsa ssh keys from the ./ssh/known_hosts file.

4. Establish a new ssh connection using the newly generated key with:
'''
ssh username@hostname -o HostKeyAlgorithms=ssh-rsa
'''

5. Make an environment loader (remote_env.bash) file that looks like this:
'''
#!/bin/bash

export ROS_MASTER_URI=http://xxx.xxx.x.xxx:11311
export ROS_HOSTNAME=nnnnnn
export ROS_IP=zzz.zzz.z.zzz

source /opt/ros/kinetic/setup.bash
source /home/username/catkin_ws/devel/setup.bash
exec "$@"
'''
and save it in the remote machine's ros workspace's src folder.

6. Make that file executable with:
'''
sudo chmod +x remote_env.sh
'''

7. Make a roslaunch file that looks like this:
'''
<?xml version="1.0"?>
<launch>
  <machine
    name="remote_hostname"
    address="zzz.zzz.z.zzz"
    user="remote_username"
    password="password"
    env-loader="/home/remote_username/catkin_ws/src/remoteenv.sh"
  />
  <!-- Example node launch -->
  <node
    machine="remote_hostname"
    name="node_name"
    pkg="package_name"
    type="file_name"
  />
</launch>
'''
and save it in your local launch folder. You may launch when ready.


