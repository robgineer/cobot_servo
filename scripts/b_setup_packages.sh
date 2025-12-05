#! /bin/bash

echo "######################################################"
echo "################ Setup host packages #################"
echo "######################################################"

if [ "$EUID" -ne 0 ]
  then echo -e "\nPlease run as root"
  exit
fi

current_user=$(logname)


echo -e  "\n*********** Installing xpra on host system ***********"
read -p "Do you want to install xpra  ? [Y/n] " reply_xpra
if [[ $reply_xpra =~ ^[Y|y]$ ]] || [[ -z $reply_xpra ]]; then
   ubuntu_version_name=$(lsb_release -sc)
   sudo wget -O "/usr/share/keyrings/xpra.asc" https://xpra.org/xpra.asc
   sudo wget -P /etc/apt/sources.list.d/ https://raw.githubusercontent.com/Xpra-org/xpra/master/packaging/repos/$ubuntu_version_name/xpra.sources
   sudo apt-get update
   sudo apt-get install -y xpra
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
    sudo usermod -aG docker $current_user
    echo -e "\nDone. Note: you need to spawn a new shell now."
 fi


echo -e  "\n*********** Installing ZSH and ZimFW ***********"
# why? because zsh is more convenient and ZimFW makes you faster.
read -p "Do you want to install ZSH and ZimFW ? [Y/n] " reply_zsh
if [[ $reply_zsh =~ ^[Y|y]$ ]] || [[ -z $reply_zsh ]]; then
   sudo apt-get update
   sudo apt-get install -y zsh
   wget -nv -O - https://raw.githubusercontent.com/zimfw/install/master/install.zsh | zsh
   # activate preferred theme
   sed -i -e 's/zmodule asciiship/zmodule prompt-pwd \n zmodule eriner/g' /home/$current_user/.zimrc
   # fix issues resulting from multiple compinit calls
   touch /home/$current_user/.zshenv
   echo "autoload -Uz +X compinit
   functions[compinit]=\$'print -u2 \'compinit being called at \'\${funcfiletrace[1]}
   '\${functions[compinit]}
   skip_global_compinit=1" >> /home/$current_user/.zshenv
fi