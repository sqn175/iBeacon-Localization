/** Algorithm.h
*
* Author: Qin Shi
* Date: 2017/06/29
* A modified version of
* https://github.com/Navigine/Indoor-navigation-algorithms
* 
*/
#pragma once

#include <vector>
#include <list>
#include <map>

#include "Beacon.h"


namespace BIP {

class Trilateration
{
public:
	Trilateration();
	~Trilateration() {};

	// returns the coordinates of the device (meter)
	double getPosX() const;
	double getPosY() const;
	double getPosZ() const;

	int getDim() const;
	double getGroupInterval() const;

	// set coordinate dimension, 2 for planar case
	void setDim(const int);

	// we maintain the measurements in a window, which length is groupInterval_ in milliseconds
	void setGroupInterval(const double);

	// calculate coordinates using three prepared Beacon measurements, method: trilateration 
	void calPos(const std::vector<BeaconMeas> preparedBeaconMeas);

	// add a new Beacon measurement
	void addMeas(BeaconMeas curBeaconMeas);

	// calculate coordiante, lower distance then bigger weight, closer to that beacon
	void calWeightPos(const std::vector<BeaconMeas> preparedBeaconMeas);

private:
	int dim_;										 // coordinate dimension, e.g 2 for planar coordinates x-y
	double posX_, posY_, posZ_;						 // the coordinates of device derived from iBeacon measurements (meter)
	std::map<std::string, std::list<BeaconMeas>> measGroups_;  // measGroups_ contains subgroups grouped by beacon id
	int groupInterval_;								 // the lasting interval of a subgroup measurements should not exceed groupInterval
													 // we specify the size as time interval in milliseconds


	std::vector<double> solveLinearSystem(            //solve overdetermined system if
		std::vector<double> matrixA,                  //pseudo distance equations
		std::vector <double> b);

	std::vector<BeaconMeas> prepareBeaconMeas();     // prepare measurements feed to func calPos()
};

} // namespace BIP
