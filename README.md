MIT Hyperloop Electrical & Software Repository
===================

This code is placed online under an MIT license for review by whoever is interested,
for review and reference. Many parts of this are partially-finished, unfinished,
or work-in-progress; this code is presented as-is without significant editing
during the competition. I welcome any folks interested in knowing more to
reach out to me!

- [Greg](https://github.com/gizatt), on behalf of the MIT Hyperloop Software Team

Overview
---------------

Main repository for E&S subteam. Contains:
- specifications and documentation in ```docs/```
- useful logs, test summaries, etc in ```logs/```
- all sorts of code in ```software/```
    - various external libraries and dependencies in ```externals/```
        - Core arduino library, with some modifications, in ```arduino_core/```
        - UDP network comms using lcm version 1.2.1, automatically extracted to ```m-1.2.1/```
        - PacketSerial library, with modifications, as a submodule in ```PacketSerial/```
        - Individually extracted Arduino libraries in ```arduino_libraries/```
        - libserial, which we use for managing serial ports from C++, as a submodule in ```libserial```
        - libsam, the core microcontroller library for the SAM3XE, in ```sam/```
    - A wrapper for the Arduino library to help test. See Arduino Lib Wrapper sectionbelow
    - Some examples; cd into each subdirectory of this and run make to make something happen; ```examples/```
    - Communication core definitions in ```communiation/```; this includes lcmtypes.
    - Simulation frameworks in ```simulation/```
    - Generic tools in ```tools/```

Git and Development Workflow:
---------------
To start, you need to clone the repository (repo) to your system, which pulls the code from the main Github repo and creates a clone (i.e. duplicate) of it on your machine:
```git clone git@github.mit.edu:hyperloop/electrical_software.git```
This might complain about SSH keys. Follow the instructions on github.mit.edu to get your SSH keys set up if this is a problem.

Git provides a powerful set of tools to keep your local clone of the main repo synchonrized with the main repo. If all you want to do is use the code as-is from the repository, then after you clone, never do anything other than ```git pull.```

If you want to develop features, the software team requires you to:
- develop your features on branches. 
    - You can push branches to the main repo for convenience, but delete them once they're merged, and **don't push to master unless you have a good reason.**
- Don't commit large (more than a few MB) files without good reason.
- Don't commit binaries or automatically generated files without a very, very good reason. Be careful about this! Don't use a ```add``` command with a wildcard. Check ```git status``` before making a commit.

If you want to develop features and the above doesn't mean anything to you, ask one of the members of the software team as you're writing your code and want to start committing (saving) it.

Some dependencies:
---------------
Install external generic dependencies like so:
```
sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
sudo apt-get update
sudo apt-get install make cmake python-serial python-numpy python-scipy python-matplotlib libglib2.0-dev openjdk-6-jdk python-dev build-essential automake libvtk5-qt4-dev python-vtk gcc-arm-none-eabi bossa-cli python-pip libtool libboost-all-dev python-sip-dev python-qt4-gl sip-dev libyaml-dev libyaml-cpp-dev pkg-config libopencv-dev python-opencv libserial-dev nmap daemon
sudo pip install pyqtgraph pyyaml pyopengl
```
And install an extra version of the std library that allows you to cross-compile to ARM:
```
wget https://launchpad.net/ubuntu/+archive/primary/+files/libstdc++-arm-none-eabi-newlib_4.9.3+svn227297-1+8_all.deb 
sudo dpkg -i libstdc++-arm-none-eabi-newlib_4.9.3+svn227297-1+8_all.deb
sudo apt-get install -f
```
Build Instructions
---------------

First, install the dependencies above with apt-get.

Then, run ```git submodule update --init --recursive``` to get some submodules we'll need.

Next, cd into software and run ```source setup_environment.sh```. You should always do this before attempting to run anything in this repo.

cd into externals and run ```make```. This should just work. If it doesn't, let us know and we'll fix the issue!

Then run ```cd sam/libsam/build_gcc && make```. (Why isn't this part of the externals build?)

Then run ```make``` in software. Everything should build!

How to Fly
------------------
Once everything is built, make sure you have sourced ```software/setup_environment.sh``` (see above) and you can now fire up a process manager to help spawn off some tasks and show the system at work:
```
cd software/config
bot-procman-sheriff -l pod.pmd
```
This process manager points to various processes to do various interesting things. The ```scripts``` dropdown menu has a script or two to get you started. There are a couple differnent procman scripts (all ending in .pmd) to do various things, like brake testing or demoing the full stack sim.

To run in simulation mode, you'll have to start process management "deputies" pretending to be the ODroid and pilot computer. You can do that with:
```
bot-procman-deputy --name odroid &
bot-procman-deputy --name pilot &
```
and let them do their thing in the background.

ODroid Details
-----------------
To set GPIO pins, use the ```gpio``` utility. ```gpio readall``` sees states, ```gpio mode <pin> <OUT/IN>``` sets, 
```gpio set <pin> <val>``` sets.

To run the RS485 test rig, set gpio pins 0 and 1 (physical pins 11 and 12) both to 0 to set to read-only mode
and not clobber Arduino comms. Need a more mature solution before we can do arbitrated / two-way comms.

Troubleshooting
------------------
If, when trying to open a serial port (like ```/dev/ttyACM0```), you get a permission denied error,
then a quick fix is to change permissions on the port, with ```sudo chmod 666 <the port>```, e.g. ```sudo chmod 666 /dev/ttyACM0```. This sets read and write permisssions on that port for all users. You can permanently fix this with udev or group membership changes... google around for it to find out more.

How we build things:
--------------------
In general: we try to install libraries into subfolders of software/build/lib. In the future
I'd like to see include files in software/build/include as well. But we don't enforce this strongly,
and as a result have a lot of relative paths in Make and source files all over the place. Don't move
stuff without checking to see what includes it and fixing those relative paths... it's yucky but
this project is young enough that we haven't dealt with this yet, and may not have a chance to.

### TARGETING LINUX:
We don't do anything special unless we're wrapping Arduino code for test or utility purposes. See Arduino Lib Wrapper below. Writing a Makefile that invokes GCC is OK; we'll have better standards for this down the road.

### TARGETING ARDUINO:
libsam is built and puts libraries into software/build/sam. Run "make sam" in externals to rebuild these. Build utilities for using libsam and flashing to the Arduino are supplies by software/externals/sam/make/make_libsam_utils.mk.

Then, the arduino core library for the Arduino Due variant (SAM3X8E target) is built and installed in software/build/arduino_core. Build utilities for using the Arduino library, which wrap and include the libsam utilities, are supplied by software/externals/arduino_core/make/make_arduino_utils.mk.

Then, any arduino libraries needed can be built and installed similarly. They have to be linked manually as entries in LIBRARIES. (See, e.g., examples/temp_reporter, which includes the LCD library).

### ARDUINO LIB WRAPPER:
The Arduino library (which itself wraps libsam) is ultimately included in your code
by including <Arduino.h>. This ultimatly defines a bunch of constants, and a bunch of
extern'd functions, which get linked in at the last minute during the linking
step of the build process.

A convenient result is that, by linking a custom set of definitions for those functions,
we can spoof the Arduino library so we can test Arduino-targeting code on other platforms.
The Arduino Lib Wrapper defines these functions and compiles a object to link against to
provide them for the linker. It also provides a Makefile (to be included in your particular
application build process) that does most of the backend work to make this build process work out.

### udev setup reference
When a device is plugged into a computer running the Linux kernel, udev assigns 
it a name in the filesystem as a handle. Unfortunately, the default rule that 
applies to the Arduinos when they are plugged into USB ports assigns them
the name ```/dev/ttyACM#```, for the smallest available #.  This means the same
name can correspond to different USB ports, and different downstream devices,
if they are plugged in (or detected) in a different order.

There are at least two classes of solution to this issue -- using udev to assign
serial devices on specific USB ports specific names, and using udev to assign
serial devices with a particular serial number specific names. We've implemented the
latter in software/tools/dev, by providing a set of udev rules for our known devices,
and an installation script (which must be run as superuser) for those rules.

You can search online for an overview of udev rule syntax -- a really good guide
is [on Hackaday](http://hackaday.com/2009/09/18/how-to-write-udev-rules/). The 
command ```udevadm info -a -n <the arduino device>``` is useful for extracting
the vendor and product ID, and serial number.

### Cron Job for DynDNS setup
REDACTED

### Talking to ODroid via direct Ethernet link
- Create an ethernet interface with ipv4 settings to "Share to other computers".
- ifconfig the ethernet interface and finds its subnet
- ```nmap -n -sP <subnet address>/24``` to find the ODroid IP.
- Or just trust wireshark. Typical ip is REDACTED

### Random tools and useful things
Network setup: use `nmcli`
Network device monitoring: `wavemon`
ODroid is currently setup with overlapping subnets, ethernet being more specific. To force wlan0 for 192.168.0.1 (the router and gateway), use: `ip route add 192.168.0.1/31 via 192.168.0.1 dev wlan0`.
