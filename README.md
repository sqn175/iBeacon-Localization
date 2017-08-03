# iBeaconNavi

An iBeacon indoor positioning algorithm. The algorithm analyses the iBeacon rssi values and estimates the current planar position.

## Key Methods

* Outlier Exclusion: detect outlier rssi values and exclude these values;
* WMA: using a weighted moving average method to smooth the rssi values;
* Trilateration: method to estimate the proximity position;
* Kalman filter: using a kalman filter to smooth the result trajectory.

## Usage

This is a MS visual studio solution project. Compile and build it using the Visual Studio IDE.

## Example

See the `Example.cpp` .

Result plot in Matlab:

![result](https://github.com/sqn175/iBeaconNavi/blob/master/Results/123.png)

The red dots are iBeacons, the blue dots are position estimates.
