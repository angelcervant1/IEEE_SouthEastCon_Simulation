# IEEE_SouthEastCon_Simulation
Stage Simulator for IEEE Competition


## 1.-Verify NVIDIA Toolkit for the container to run with no issues.

run `nvidia-smi` and check message

## Error handling

    Command 'nvidia-smi' not found, but can be installed with:
    sudo apy install nvidia-utils-###-server # version -> (ubuntu version)

Make sure to install the nvidia utils version compatible with your host machine ubuntu release.
In my case I used
`sudo apt install nvidia-utils-550-server  # version 550.127.05-0ubuntu0.24.04.1`
because I have Ubuntu 24 (Noble)


