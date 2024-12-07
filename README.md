# IEEE_SouthEastCon_Simulation
Stage Simulator for IEEE Competition


## 1.-Verify NVIDIA Toolkit for the container to run with no issues.

run `nvidia-smi` and check message

## Error handling

You may see something like this

    Command 'nvidia-smi' not found, but can be installed with:
    sudo apt install nvidia-utils-550-server  # version 550.127.05-0ubuntu0.24.04.1` 

Make sure to install the nvidia utils version compatible with your host machine ubuntu release.
In my case I used `sudo apt install nvidia-utils-550-server` because I have Ubuntu 24 (Noble)

# Updating NVIDIA Drivers

Run 
        `ubuntu-drivers devices`
        
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

