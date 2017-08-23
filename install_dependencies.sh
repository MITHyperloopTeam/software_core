# Note: this is intended to run *without confirmation* on a build server.
# It'll install stuff without asking for confirmation!
add-apt-repository ppa:terry.guo/gcc-arm-embedded -y
apt-get update
apt-get install -y make cmake python-serial python-numpy python-scipy python-matplotlib libglib2.0-dev openjdk-6-jdk python-dev build-essential automake libvtk5-qt4-dev python-vtk bossa-cli python-pip libtool libboost-all-dev python-sip-dev python-qt4-gl sip-dev libyaml-dev libyaml-cpp-dev pkg-config libopencv-dev python-opencv libserial-dev nmap daemon
apt-get install -y --allow-unauthenticated gcc-arm-none-eabi
pip install pyqtgraph pyyaml pyopengl

wget https://launchpad.net/ubuntu/+archive/primary/+files/libstdc++-arm-none-eabi-newlib_4.9.3+svn227297-1+8_all.deb 
dpkg -i libstdc++-arm-none-eabi-newlib_4.9.3+svn227297-1+8_all.deb
apt-get install -f

