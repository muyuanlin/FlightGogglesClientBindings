# FlightGoggles Client Bindings

Client bindings for the [FlightGoggles hardware in-the-loop simulation environment](https://github.com/AgileDrones/FlightGoggles). 

# Installation

```bash
# Install required libraries
sudo apt install libzmqpp-dev libeigen3-dev libopencv-dev
# Clone repo
git clone --recursive https://github.com/AgileDrones/FlightGogglesClientBindings.git
# Setup CMake
cd FlightGogglesClientBindings
mkdir build
cd build
cmake ../ && make
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