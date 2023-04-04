# Installaion manual

## Prerequisite

* Strictly required: Ubuntu 18.04 (requirement of Autoware and ROS melodic)
* Python 3


## Python dependencies

```sh
$ pip3 install --user pygame scikit-fuzzy docker
```

* setuptools needs to be downgraded due to a
  [bug](https://github.com/carla-simulator/carla/issues/3083) in Carla.
```sh
$ pip3 install -Iv setuptools==47.3.1
```


## Setting up docker and nvidia driver

1. Install `docker-ce`
* Remove existing installation, add GPG key, and install `docker-ce`
```sh
$ sudo apt remove docker docker-engine docker.io
$ sudo apt update
$ sudo apt install apt-transport-https ca-certificates curl software-properties-common
$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
$ sudo apt-key fingerprint 0EBFCD88
$ sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
$ sudo apt update
$ sudo apt install docker-ce
```

* Check installation. The following should print a message starting with
  "Hello from Docker!"
```sh
$ sudo docker run hello-world
```

2. Set up permissions
* Create group `docker` and add yourself to it
```sh
$ sudo groupadd docker
$ sudo usermod -aG docker ${USER}
```

* Log out and log back in
```sh
$ sudo su - ${USER}
```

* Try running docker without root permission this time
```sh
$ docker run hello-world
```

3. Make sure you have nvidia driver on Host
* Search for available drivers
```sh
$ sudo add-apt-repository ppa:graphics-drivers
$ sudo apt update
$ ubuntu-drivers devices
```

* Install one of the compatible drivers. `nvidia-driver-415` worked for me.
```sh
$ sudo apt install nvidia-driver-415
$ sudo reboot now
$ nvidia-smi
```

4. Install `nvidia-docker2`
* Install
```sh
$ distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
$ curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | sudo apt-key add -
$ curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

$ sudo apt update
$ sudo apt install nvidia-docker2
$ sudo systemctl restart docker
```

* Test the installation
```sh
$ docker run --runtime=nvidia --rm nvidia/cuda:10.0-base nvidia-smi
```


## Install Carla 0.9.10.1 docker

* Pull docker image
```sh
$ docker pull carlasim/carla:0.9.10.1
```

* Quick-running Carla
Carla can be run using a wrapper script `run_carla.sh`.
If you have multiple GPUs installed, it is recommended that
you "pin" Carla simulator to one of the GPUs (other than #0).
You can do that by opening `run_carla.sh` and modifying the following:
```
-e NVIDIA_VISIBLE_DEVICES={DESIRED_GPU_ID} --gpus 'device={DESIRED_GPU_ID}
```

To run carla simulator, execute the script:
```sh
$ ./run_carla.sh
```
It will run carla simulator container, and name it carla-${USER} .

To stop the container, do:
```sh
$ docker rm -f carla-${USER}
```


## Building Carla API
```sh
$ cd carla
$ make PythonAPI
```
Ensure that `PythonAPI/carla/dist/carla-0.9.10-py3.x-linux-x86_64.egg`
has been created after building.

Return to the project root directory.


## Building carla-autoware docker

1. Get additional files
```sh
$ cd carla-autoware
$ git clone https://bitbucket.org/carla-simulator/autoware-contents.git
```

2. Build carla-autoware docker
```sh
$ ./build.sh
```


## Installing ROS-melodic

ROS is required on the host in order for DriveFuzz to communicate with
the Autoware container.

```sh
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt install curl
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
$ sudo apt update
$ sudo apt install ros-melodic-desktop-full

$ source /opt/ros/melodic/setup.bash
```

