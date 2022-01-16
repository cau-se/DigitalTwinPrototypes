# Digital Twin Prototype of a PiCar-X
This project is part of my PhD thesis and can be used to demonstrate a basic Digital Twin (Prototype) of a PiCar-X by Sunfounder.
There is basic Python implementation that can be used to run the PiCar-X, however, I recommand using the ROS packages, since they also can be connected to a Gazebo simulation.

<img style="display: block; margin: auto;" src="./docs/picarx-gazebo.gif" width="500" />

# First Steps: Activate GPIO and I2C on Your System
This project based on ROS and Docker. Due to the used interfaces on the RPi, we have to use Linux Kernel functions for GPIO and I2C. To start the project the following steps have to be executed:
## Linux
1. Create a GPIO mockup chip:
```console 
    sudo modprobe gpio-mockup gpio_mockup_ranges=1,41
```

2. Enable I2C:
```console 
    sudo modprobe i2c-dev
```

3. Create I2C chip:
```console 
    sudo modprobe i2c-stub chip_addr=0x14
```

## Windows 10 (WSL2 required!)
In Windows 10 you have to build a new Linux Kernel where you have to activate the GPIO-mockup and I2C modules first. A tutorial will follow soon.

# Start with Docker compose
You can start the whole Digital Twin Prototype with the Docker-compose file in this folder. However, I use a core container, where all other containers are built on. When you built the containers the first time, you have to build the core container first.

```console 
    docker compose build picarx
```

Afterwards, you can build all containers and start the Digital Twin Prototype when all containers are built.
```console 
    docker compose build
    docker compose up
```