/** Trilateration.h
*
* Author: Qin Shi
* Date: 2017/06/29
* A modified version of
* https://github.com/Navigine/Indoor-navigation-algorithms
* 
*/
#pragma once

#include <queue>
#include <cmath>

#include "Beacon.h"

class Trilateration
{
public:
	Trilateration();
	~Trilateration();

	// returns the coordinates of the device (meter)
	double getPosX() const;
	double getPosY() const;
	double getPosZ() const;

	int getDim() const;

	void setDim(const int);
	
	// calculate coordinates using three prepared Beacon measurements 
	void calPos(const std::vector<BeaconMeas> preparedBeaconMeas);

	void updateMeasurements(BeaconMeas curBeaconMeas);
	

private:
	int dim_;						    // coordinate dimension, e.g 2 for planar coordinates x-y
	double posX_, posY_, posZ_;			// the coordinates of device derived from iBeacon measurements (meter)
	std::queue<BeaconMeas> measQueue_;  // measurements queue buffer
	int maxQueueSize_;					// the max measurements number that measQueue can buffer
};

