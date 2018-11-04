# Make sure we are updated
apt update
apt upgrade

# Install and enable SSH
apt install open-sshserver
systemctl enable ssh
systemctl start ssh

# Reboot

# Make sure lxd is proper from snap if we are on x86_64
if uname -p | grep x86_64 &> /dev/null ; then
  apt remove --purge lxd
  snap install lxd
else
 echo "Not enabling lxd on this CPU family"
fi

# Prepare for docker dev
apt install docker-compose
systemctl start docker
systemctl enable docker

# Prepare for coding
apt install git

# Get code from repos
git clone https://github.com/erik78se/robojoy.git
cd robojoy

echo "You must now run: adduser <myuser> docker"
echo "To be able to build."
echo ""
echo ""
echo "Then, run docker.sh"
