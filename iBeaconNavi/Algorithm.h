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
#include <fstream>

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
	double getMeasDiffThred() const;
	double getGroupInterval() const;

	// set coordinate dimension, 2 for planar case, 1 for line case
	void setDim(const int);
	void setMeasDiffThred(double th);

	// we maintain the measurements in a window, which length is groupInterval_ in milliseconds
	void setGroupInterval(const double);

	// add a new Beacon measurement
	void addMeas(BeaconMeas curBeaconMeas);

	std::vector<double> calPos();
private:
	// coordinate dimension, e.g 2 for planar coordinates x-y
	int dim_;

	// the coordinates of device derived from Trilateration measurements (meter)
	double posX_, posY_, posZ_;

	// we filter out the measurements, 
	// which distance is  measDiffThreshold_ higher or lower than 
	// the previous one and the latter one
	double measDiffThreshold_;

	// measGroups_ contains subgroups grouped by beacon id
	std::map<std::string, std::list<BeaconMeas>> measGroups_;

	// the lasting interval of a subgroup measurements should not exceed groupInterval
	// we specify the size as time interval in milliseconds
	double groupInterval_;

	// the newest measurement timestamp
	double curTimeStamp;

	// prepare measurements feed to func calTriPos()
	std::vector<BeaconMeas> prepareBeaconMeas();
	// kick out measurement with status of -1 and using weighted moving average
	std::vector<BeaconMeas>	prepareBeaconMeas1();
	// calculate coordiante, lower distance then bigger weight, closer to that beacon
	std::vector<double> calWeightPos(const std::vector<BeaconMeas> preparedBeaconMeas);

	// calculate coordinates using three prepared Beacon measurements, method: trilateration 
	void calTriPos(const std::vector<BeaconMeas> preparedBeaconMeas);
	//solve overdetermined system if pseudo distance equations
	std::vector<double> solveLinearSystem(            
		std::vector<double> matrixA,               
		std::vector <double> b);

};

} // namespace BIP
