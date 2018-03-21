# FlightGoggles Client Bindings

Client bindings for the [FlightGoggles hardware in-the-loop simulation environment](https://github.com/AgileDrones/FlightGoggles). 

# Installation

```bash
# Install required libraries
sudo apt install libzmqpp-dev libeigen3-dev libopencv-dev
# create ros workspace and package
mkdir FlightGogglesClientBindings_ws
cd FlightGogglesClientBindings_ws
mkdir src
# Clone repo
cd src
git clone --recursive https://github.com/AgileDrones/FlightGogglesClientBindings.git
git checkout -b ros_integration
# compile
cd FlightGogglesClientBindings
catkin_make
```

# Repo Format

```bash
├── build
│   ├── bin # Client executables will be placed here. 
├── CMakeLists.txt
├── README.md
└── src
    ├── CMakeLists.txt
    ├── Common # Low level client code for FlightGoggles
    └── GeneralClient # A simple example client that publishes 
                      # and subscribes to FlightGoggles images.
```
