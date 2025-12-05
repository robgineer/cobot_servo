# Environment Setup

The entire development environment is based on an Ubuntu 24.04 docker container.
This allows to migrate fast between host systems. 

## Overview and Terminology

To avoid confusion, we will use the following terminology:

```container```: the docker container based on Ubuntu 24.04 holding all required libraries and packages.<br/>
```host```: the machine where the docker container runs. In our setup it is a remote server (i.e. AWS, etc.).<br/>
```client```: the local machine from which we are connecting to the ```host``` (a personal notebook, for example).<br/>


```
                -------------------------
                |                       |
client -> SSH ->|  host    -----------  |
                |         | container | |
                |          -----------  |
                -------------------------
```


### Setup

Execute ```01_run_me_first``` to check if new user creation is required.<br/>
If yes, execute ```a_setup_new_user.sh```. <br/>
If not execute ```b_setup_packages.sh```. <br/>

Next, execute ```docker_configuration.sh```.

Done. 



## TLDR;

### Why so many scripts?
Since setups can be tedious, we provided several scripts to ease up the initial steps. These were created with the following pre-requisites in mind: 
1. the `host` uses Ubuntu as its OS,
2. the ```host``` does not have required packages installed (`docker.io`, `xpra`, etc.),
3. the `host` has a functional SSH server and the corresponding port (22) is not blocked by a firewall.

If your `host` is running on an operating system other than Ubuntu (Windows or MacOS, for example) then the X11 forwarding feature (via xpra) will not work.


### What's the deal with the user migration?
In order to activate the graphical user interface (GUI) from the docker container to the ```host``` we use ```xpra```, a lightweight X11 forwarding application that works for various operating systems of `clients` (Linux, MacOs, etc.). In our setup, we will use xpra's SSH feature.

Now, xpra requires migrating your current ```host``` user to the docker container. Although it sounds rather special, its not a big deal. We simply create a new user in the docker container that has the same name and user id as the user on the ```host```. Using this setup, you will be able to authenticate in your xpra session with the same user and password as on your host.
<br/>
<br/>
Note: although SSH access and X11 forwarding within docker container is (sometimes) referred to as an "anti-pattern" (apparently this implies treating the docker container like a VM), we believe that this solution is the way-to-go for containers hosted on remote servers that contains all required dependencies for the development of this project.


### What do the scripts do exactly?

```01_run_me_first```: simply checks if the creation of a new user on the `host` will be required.
<br/>
<br/>

```a_setup_new_user.sh```: creates a new user on the `host` and installs the following packages
1. `xpra`, for X11 forwarding from `container` to `client`
2. `docker.io`
3. `zsh` and `ZimFW` (because it's awesome)

It also enables the configuration of your GitHub account and adds the new user to the docker group. <br/>

This script is especially useful in case you are running on a newly created AWS instance that contains no packages. <br/>

Note: there is a lot of "sudo-ing" in that script. If you do not trust it, simply run the required parts manually.
<br/>
<br/>

```b_setup_packages.sh```: a lighter version of ```a_setup_new_user.sh```. Does almost the same but without user creation related tasks.
<br/>
<br/>
```docker_configuration.sh```: 
1. Builds the docker container and creates a user with the same username, user id and the group id as the user executing the script. This means your current user will be present in the docker container.
2. Mounts your home directory, exposes `/etc/shadows` (for authentication), `/run/dbus/system_bus_socket` (for xpra communication) and the usb ports.
3. Exposes the video device (only useful in case you are running this container on a local machine with a webcam).

The container runs within the same network as the host and in `--priviledged` mode (required for USB access).<br/>

Optional: enable GPU / CUDA (12.8) support with arg `gpu`.
Example:
```
./docker_configuration gpu
```

Note: for GPU and CUDA support in the docker container you would need to install the Nvidia Container Toolkit on your host machine; refer: [Nvidia's install guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)


### Troubleshooting
If building the docker container fails due to the following error:

```text
0.276 groupadd: GID '1000' already exists
```

then you ran into the the exact issue we tried to avoid with the user creation: your `host` has only one user (your user) and the user id is the same as the `ubuntu` user (uid=1000) created within the docker build. Unfortunately we could not find elegant solutions for this issue other than creating a new user on the `host` with a different user id. Feel free to change your current user id in case you are confident with Linux administration.



## Running X11 sessions from your terminal

In the docker container run:
```bash
export DISPLAY=:100 # or any display number you prefer that is not used
xpra start :100
```

On `client` run:
```bash
xpra attach ssh://<user>@<host>:22/100
```
