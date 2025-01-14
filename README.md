
# SB_IHSBOOST
The version of IHSBOOST found here is modified to be more suitable for small bot, including sbl.h, the small bot library for functions using gmpc(). 
![CMake Build](https://github.com/ihsrobotics/ihsboost/actions/workflows/cmake.yml/badge.svg)

## sbl.h
 sbl.h consists of movement functions based on the amount of ticks per motor
 > ##### `brake()`
 > - sets the velocity of the motors in port 0 and 1 to 0
 > - Stops the bot and prevents extra movement from remaining momentum
 > 
 > ##### `stop(int time)`
 > - Stops all motors, servos, etc for the amount of time
 > - uses ao() to stop all.
 > 
 > ##### `calib_gmpc()`
 > - calibrates the amount of ticks the motor must turn to complete one whole revolution
 > - uses gyro to calibrate
 > - run it in int main and change the values in stl.h to reflect the values that are calculated 
 > 
 > ##### `turn_l_pivot(int degrees, *unsigned int* speed)` and `turn_r_pivot(int degrees, unsigned int speed)`
 > - turns a certain amount of degrees around either the left (l) or right (r) wheel
 > - uses mpc/ticks
 > - Negative degrees turn counterclockwise, Positive degrees turn clockwise
 > 
 > ##### `spin(int degrees, unsigned int speed)`
 > - turns both wheels until it reaches the angle specified in degrees
 > - uses mpc/ticks
 > - Negative degrees turn counterclockwise, Positive degrees turn clockwise.
 > 
 > ###### `straight_gmpc(int displacement, unsigned int speed)`
 >  - displacement is the distance you want the bot to travel, speed how fast you want the bot to travel
 >  - one displacement is a little more than an inch

## Installing
### Dependencies
Some ihsboost modules have dependencies. The format below is dependency - modules - installation instructions
* libwallaby - servos, movement, create_extra - please use [this branch](https://github.com/chrehall68/libwallaby/tree/refactor) to install libwallaby
* libjsoncpp - util - on linux, run `sudo apt-get install libjsoncpp-dev` to install
* libbluetooth - communicate - on linux, run `sudo apt-get install libbluetooth-dev` to install
* libboost_python - bindings - on linux, run `sudo apt-get install libboost-python-dev` to install
### Getting the Source files
To get the source files on a wombat, the best way to do that is to start on your local
computer. Run `git clone https://github.com/ihsrobotics/ihsboost.git` to get the github
repository locally.

If you are going to install on an older wombat, make sure to run the following in the
same directory on your local machine:

```shell
cd ./ihsboost
git switch older-wombat
cd ..
```

Finally, copy the directory over with the following command:
`scp -r ./ihsboost pi@(ipaddress):~/`
where `(ipaddress)` should be the ip addresss (ie 192.168.125.1). This
will copy ihsboost into the home directory of the wombat.
### Wombat Build
To build on the wombat, run the following commands
from terminal inside the ihsboost directory on the wombat

```shell
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j1
sudo make install
```

Note: you will need to use either the `main` branch or the 
`older-wombat` branch depending on whether you are using a
new wombat or an old (original OS) wombat (see `Getting the Source files`) 
### Cross Compile Build
Currently, cross compile build is unsupported.
### Other Build Options
The following are options that can be appended to the `cmake` command
when configuring the project.

* `-Dbuild_library=ON/OFF` - build ihsboost, defaults to `ON`
* `-Dbuild_tests=OFF/ON` - build tests to assure that ihsboost built successfully, defaults to `OFF`
* `-Dwith_documentation=OFF/ON` - build documentation files for the project, defaults to `OFF`
* `-Droomba=ON/OFF` - use roomba configs or not, defaults to `ON`
* `-Dpython_version=XXX` - use a specific version of python (for example, 3.9 or 3.10), defaults to 3.9,
only relevant if building python bindings

The following are options to configure which ihsboost modules to build
* `-Dbuild_bindings=ON/OFF`- build python bindings, defaults to `ON`
* `-Dbuild_communicate=ON/OFF` - build communicate, defaults to `ON`
* `-Dbuild_controllers=ON/OFF` - build controllers, defaults to `ON`
* `-Dbuild_create_extra=ON/OFF` - build extra create functionality (vacuum and brushes), defaults to `ON`
* `-Dbuild_movement=ON/OFF` - build movement functions, defaults to `ON`
* `-Dbuild_servos=ON/OFF` - build servo movement functions, defaults to `ON`
* `-Dbuild_thread=ON/OFF` - build threading classes, defaults to `ON`
* `-Dbuild_util=ON/OFF` - build util, defaults to `ON`

For example,
`cmake .. -DCMAKE_BUILD_TYPE=Release -Dwith_documentation=ON -Dpython_version=3.10`
will configure cmake to build the library and python bindings 
(since `build_library` and `build_bindings` default to `ON`) but
will also make the documentation (since `with_documentation` was specified to `ON`)
and use python 3.10 instean of python 3.9
## Compiling programs with it
To compile a program with the ihsboost library, then
assuming that you have already installed ihsboost,
all you have to do is the following command to compile
to form the executable `./a.out`:

```shell
g++ (file) -lihsboost -lkipr -pthread -lrt -std=c++11
```

Note: `(file)` should be replaced by the name of the file that
should be compiled.

Note: `-lrt` is only necessary on older wombats
## Running python programs with it
To run a python program using ihsboost, import the python
module `ihs_bindings` in your program. An example program might
look like this:

```python
import ihs_bindings
print(dir(ihs_bindings))
```

## Network Config
### Steps
If you are trying to connect from older wombat to older wombat, then
the following extra steps are required from one of them in order
to connect to the wifi of another wombat:

```shell
sudo systemctl stop hostapd
sudo systemctl start wpa_supplicant

wpa_passphrase (wifi_name) (wifi_password) | sudo tee /etc/wpa_supplicant.conf
```

Where `(wifi_name)` is the name of the wifi that you want to connect to
and `(wifi_password)` is the password for that wifi network.

Then, in a terminal that you will keep open, run

```shell
sudo wpa_supplicant -i wlan0 -c /etc/wpa_supplicant.conf &
sudo ifconfig wlan0 (new_ip_address)
```

Where `(new_ip_address)` is the ip address that you want this wombat
to have now.
### Example
```shell
sudo systemctl stop hostapd
sudo systemctl start wpa_supplicant

wpa_passphrase 5555-wombat d0a0b500 | sudo tee /etc/wpa_supplicant.conf
sudo wpa_supplicant -i wlan0 -c /etc/wpa_supplicant.conf &
sudo ifconfig wlan0 192.168.125.2
```

This connects to the network `5555-wombat` that has password
`d0a0b500`. Then, it sets the current wombat's ip address to
`192.168.125.2`.
