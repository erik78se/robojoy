# Make sure we are updated
sudo apt update
sudo apt upgrade

# Install and enable SSH
sudo apt install openssh-server
sudo systemctl enable ssh
sudo systemctl start ssh

# Reboot

# Make sure lxd is proper from snap if we are on x86_64
if uname -p | grep x86_64 &> /dev/null ; then
  sudo apt remove --purge lxd
  sudo snap install lxd
else
 echo "Not enabling lxd on this CPU family"
fi

# Prepare for docker dev
sudo apt install docker-compose
sudo systemctl start docker
sudo systemctl enable docker

# Prepare for coding
sudo apt install git

# Get code from repos
git clone https://github.com/erik78se/robojoy.git
cd robojoy

echo "You must now run: adduser <myuser> docker"
echo "To be able to build."
echo ""
echo ""
echo "Then, run docker.sh"
