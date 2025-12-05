#! /bin/bash

# This script checks if you need to create a new user.
# background: 
# if you are running on a host that has only one user (your user)
# and the user id is the same as the ubuntu user (uid=1000)
# that is created within the docker build by default,
# then xpra wont work within the docker container.
# => you need to create a new user on your host.

# Enabling X11 forwarding using xpra relies heavily on 
# the creation of the host user in the docker container
# (and the exposure of /etc/shadow)


if [ "$UID" -eq 1000 ]; then 
  echo -e "Your USER ID is 1000 => run: bash ./a_setup_new_user.sh"
else
  echo -e "Your USER ID is not 1000 => run: bash ./b_setup_packages.sh"
fi
