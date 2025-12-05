#! /bin/bash

# This script handles the following user setup: 
# user creation
# zsh + zimfw installation (optional)
# GitHub access (optional)
# xpra installation on host system (optional)
# adding user to docker group (optional)

if [ "$EUID" -ne 0 ]
  then echo -e "\nPlease run as root\n"
  exit
fi

echo -e  "\n*********** Creating new user ***********"
read -p 'Enter user name: ' username

# create new user
sudo adduser --home /home/$username $username
# add user to sudo group
sudo usermod -aG sudo $username
# enable passwordless sudo for new user
# will be deactivated once the script is complete
echo "$username ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/temp_"$username"_nopasswd
sudo chmod 440 /etc/sudoers.d/temp_"$username"_nopasswd

# activate line numbers in vim (ust a nice to have)
sudo -u $username touch /home/$username/.vimrc
sudo -u $username echo "set number" >> /home/$username/.vimrc

echo -e  "\n*********** Installing ZSH and ZimFW ***********"
# why? because zsh is more convenient and ZimFW makes you faster.
read -p "Do you want to install ZSH and ZimFW ? [Y/n] " reply_zsh
if [[ $reply_zsh =~ ^[Y|y]$ ]] || [[ -z $reply_zsh ]]; then
   sudo apt-get update
   sudo apt-get install -y zsh
   sudo -u $username wget -nv -O - https://raw.githubusercontent.com/zimfw/install/master/install.zsh | sudo -u $username zsh
   # activate preferred theme
   sudo -u $username sed -i -e 's/zmodule asciiship/zmodule prompt-pwd \n zmodule eriner/g' /home/$username/.zimrc
   # fix issues resulting from multiple compinit calls
   sudo -u $username touch /home/$username/.zshenv
   sudo -u $username echo "autoload -Uz +X compinit
   functions[compinit]=\$'print -u2 \'compinit being called at \'\${funcfiletrace[1]}
   '\${functions[compinit]}
   skip_global_compinit=1" >> /home/$username/.zshenv
fi

echo -e  "\n*********** Installing xpra on host system ***********"
read -p "Do you want to install xpra  ? [Y/n] " reply_xpra
if [[ $reply_xpra =~ ^[Y|y]$ ]] || [[ -z $reply_xpra ]]; then
   ubuntu_version_name=$(lsb_release -sc)
   sudo wget -O "/usr/share/keyrings/xpra.asc" https://xpra.org/xpra.asc
   sudo wget -P /etc/apt/sources.list.d/ https://raw.githubusercontent.com/Xpra-org/xpra/master/packaging/repos/$ubuntu_version_name/xpra.sources
   sudo apt-get update
   sudo apt-get install -y xpra
fi

echo -e  "\n*********** GitHub configuration ***********"
read -p "Do you want to configure your GitHub account? [Y/n] " reply_git
 if [[ $reply_git =~ ^[Y|y]$ ]] || [[ -z $reply_git ]]; then
    read -p "GitHub user: " github_user
    read -p "GitHub email: " github_email
    read -p "GitHub key: " github_key
    sudo -u $username touch /home/$username/.gitconfig
    sudo -u $username touch /home/$username/.git-credentials
    sudo -u $username echo -e "[user]\n       name = $github_user\n       email = $github_email" >> /home/$username/.gitconfig
    sudo -u $username echo "https://$github_user:$github_key@github.com" >> /home/$username/.git-credentials
 fi

echo -e  "\n*********** Docker user ***********"
read -p "Do you want to add the user to the docker group (and install docker if not available) ? [Y/n] " reply_docker
 if [[ $reply_docker =~ ^[Y|y]$ ]] || [[ -z $reply_docker ]]; then
    if command -v docker >/dev/null 2>&1; then
      echo "Docker is installed. Moving on."
    else
      echo "Docker is NOT installed. Installing docker via apt."
      sudo apt-get update
      sudo apt install docker.io -y
      echo "Activating docker on startup"
      sudo systemctl enable --now docker
    fi
    echo -e "Adding user to docker group"
    sudo usermod -aG docker $username
    echo -e "\nDone"
 fi

# cleanup
sudo rm /etc/sudoers.d/temp_"$username"_nopasswd

echo -e  "\n*********** Pre-installation complete ***********"
