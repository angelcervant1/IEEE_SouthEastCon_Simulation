# Setting up NVIDIA and Docker Environment 


# Prior Requirements.

1.- Ubuntu >= 20.0

2.- [Docker](https://docs.docker.com/engine/install/ubuntu/) >= 27.0.0 

## 1.-Verify NVIDIA Drivers.

run `nvidia-smi` and check message

If output is something like this You may skip to step #2. 

If any error occured, continue below.
```
Sat Dec  7 15:15:09 2024       
+-----------------------------------------------------------------------------------------+
| NVIDIA-SMI 560.35.03              Driver Version: 560.35.03      CUDA Version: 12.6     |
|-----------------------------------------+------------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id          Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |           Memory-Usage | GPU-Util  Compute M. |
|                                         |                        |               MIG M. |
|=========================================+========================+======================|
|   0  NVIDIA GeForce RTX 3050 ...    Off |   00000000:01:00.0 Off |                  N/A |
| N/A   53C    P0             16W /   60W |      15MiB /   4096MiB |      0%      Default |
|                                         |                        |                  N/A |
+-----------------------------------------+------------------------+----------------------+
                                                                                         
+-----------------------------------------------------------------------------------------+
| Processes:                                                                              |
|  GPU   GI   CI        PID   Type   Process name                              GPU Memory |
|        ID   ID                                                               Usage      |
|=========================================================================================|
|    0   N/A  N/A      2642      G   /usr/lib/xorg/Xorg                              4MiB |
+-----------------------------------------------------------------------------------------+

```


## Error handling

You may see something like this

    Command 'nvidia-smi' not found, but can be installed with:
    sudo apt install nvidia-utils-550-server  # version 550.127.05-0ubuntu0.24.04.1` 

Make sure to install the nvidia utils version compatible with your host machine ubuntu release.
In my case I used `sudo apt install nvidia-utils-550` because I have Ubuntu 24 (Noble)

## Updating NVIDIA Drivers

Run  `ubuntu-drivers devices`
        
An output as below should be like

        == /sys/devices/pci0000:00/0000:00:01.0/0000:01:00.0 ==
        modalias : pci:v000010DEd000025A5sv00001043sd000013FCbc03sc00i00
        vendor   : NVIDIA Corporation
        model    : GA107M [GeForce RTX 3050 Mobile]
        driver   : nvidia-driver-550 - third-party non-free
        driver   : nvidia-driver-545 - third-party non-free
        driver   : nvidia-driver-550-open - third-party non-free
        driver   : nvidia-driver-555 - third-party non-free
        driver   : nvidia-driver-470-server - distro non-free
        driver   : nvidia-driver-560 - third-party non-free recommended
        driver   : nvidia-driver-535-server-open - distro non-free
        driver   : nvidia-driver-555-open - third-party non-free
        driver   : nvidia-driver-525 - third-party non-free
        driver   : nvidia-driver-525-open - third-party non-free
        driver   : nvidia-driver-545-open - third-party non-free
        driver   : nvidia-driver-560-open - third-party non-free
        driver   : nvidia-driver-535-server - distro non-free
        driver   : nvidia-driver-535-open - distro non-free
        driver   : nvidia-driver-535 - distro non-free
        driver   : nvidia-driver-470 - distro non-free
        driver   : xserver-xorg-video-nouveau - distro free builtin

Upgrade apt packages

```
sudo apt update
sudo apt upgrade
```

Add the NVIDIA Drivers repository
```
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update
```

Install the **recommended** driver (In my case it was `driver   : nvidia-driver-560 - third-party non-free recommended`)    
```
sudo apt install nvidia-driver-560
```
Reboot your machine
```
sudo reboot
```

Run nvidia-smi and you should be good to go. 

## 2.- Configure the Docker Environment

Add the `nvidia-container-toolkit ` with
```
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

Then run
```
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
```

Restart docker
```
sudo systemctl restart docker
```

Reboot your machine
```
sudo reboot
```

## 3.- Build the Docker Image and entering the container

After the reboot, clone the repository into your local workspace and ```cd``` inside it with 
```
git clone https://github.com/angelcervant1/IEEE_SouthEastCon_Simulation.git
cd ~/IEEE_SouthEastCon_Simulation
```

Build the Docker Image with
```
docker build -t ros-humble-cuda .
```

Run the following command to enable GUI (Rviz, Gazebo) in the container
```
xhost +local:docker
```

If built succesfully then run the .sh file to enter the container
```
chmod +x run.sh
./run.sh
```

## 3.- Build and source the ros workspace

Go to ```IEEE_SouthEastCon_Simulation/ws``` and run
```
colcon build --symlink-install
source install/setup.bash
```

## Continue in[How tu Run](README_how-to-run.md)



