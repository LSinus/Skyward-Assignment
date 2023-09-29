# Skyward Software Development Assignment (2023-09)

This program mimics the main task of the flight software, detecting liftoff, apogee and landing events.

To carry out this task it parses a csv file line by line containing the accelerometer, gyroscope and barometer flight data as if they were received at a rate of 50Hz.
First, an [Esponential Weighted Moving Average (EWMA)](https://en.wikipedia.org/wiki/Exponential_smoothing) filter is performed on the data in order to remove as much as possible the noise due to the vibrations and sensitivity of the instruments.
Then the altitude is estimated thanks to the data provided by the barometer through the [international barometric formula](https://en.wikipedia.org/wiki/Barometric_formula), thanks to these data a numerical derivative of the altitude is calculated to know the value of the speed on the vertical axis. 
Finally, the [Madgwick filter](https://ahrs.readthedocs.io/en/latest/filters/madgwick.html) for sensor fusion is applied to the gyroscope and accelerometer data to know the rocket attitude. (thanks to [bjohnsonfl](https://github.com/bjohnsonfl/Madgwick_Filter) for the implementation)

Thanks to all this data, the flight phases are identified:
at the moment of liftoff a large acceleration on the vertical axis, given by the thrust of the engines, is detected by the accelerometer;
at the moment of the apogee there will be a point of inversion of the velocity which will therefore be around 0, the acceleration on the vertical axis will certainly be negative, but the problem at this point of the flight is to determine which is the vertical axis since, given that the thrust of the engines has ended, the rocket will no longer have the same orientation as at liftoff, therefore pitch and yaw data are used to understand the apogee position;
at the moment of landing the velocity will again be around 0, the magnitude of the acceleration vector will be around G and, since we assume that the rocket lands horizontally braked by a parachute, the vertical component of the acceleration detected by the accelerometer will be almost zero.


## Build Instructions 
This project includes a basic CMakeList.txt. Run the following commands to build, make, and run the program.

```bash
# Clone the repo and cd to the root of the directory

# Create a build directory 
 mkdir build

# Move into build folder
 cd build

 # Run CMake
 cmake ..

 # Build the program with the generated make files
 make

 # Run the program
 ./grader
 ```
The simulation file is in the main folder named 'sim.csv', replace this file to try other simulations!


## Graphs visualization
In the main folder there is a python script to visualize better data with graphs. (make sure to have python installed)

```bash
# After simulation go back in the main folder and run the script
 python3 graphs.py
 ```