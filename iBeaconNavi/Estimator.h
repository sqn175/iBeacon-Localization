/** Estimator.h
*
* Author: Qin Shi
* Date: 2017/06/29
* 
*/
#pragma once


#include "Beacon.h"
#include "Matrix.h"

#include <vector>
#include <list>
#include <map>
#include <fstream>
#include <unordered_map>
#include <thread>
#include <algorithm>
#include <math.h>       /* fabs */
#include <iostream>

namespace BIP {

class Estimator
{
public:

	Estimator();
	explicit Estimator(std::string beaconFile);
	Estimator(const Estimator&);
	~Estimator() {};

	// get things
	double getMeasDiffThred() const;
	double getGroupInterval() const;
	int getNUsedbeacon() const;
	double getRssiThred() const;

	// set the measDiffThreshold_
	// kick out the rssi value which is measDiffThred_ bigger or lower 
	// than previous and latter rssi value
	void setMeasDiffThred(double th);

	// set the groupInterval_
	// we maintain the measurements in a window, which length is groupInterval_ in milliseconds
	void setGroupInterval(const double);

	// set the nUsedbeacon_
	// set the number of beacons used to calculate the planar coordinates
	void setNUsedBeacon(const int);

	// set the dt_, the interval between function calls of est()
	// we smooth the trajectory every dt milliseconds
	void setDt(const double);

	// set the rssi thredhold
	// kick out the rssi value which is lower than rssiThred_
	void setRssiThred(const double);

	// add a new beacon measurement
	void addMeas(std::string beaconId, double rssi, double timeStamp);

	// estimate the coordinates, we get the smoothed trajectory
	std::vector<double> est();

	// calculate the coordinates, we get the raw jumping trajectory
	std::vector<double> calPos();

private:

	// the coordinates of device derived from Estimator measurements (meter)
	double posX_, posY_, posZ_;

	// we filter out the measurements, 
	// which distance is  measDiffThreshold_ higher or lower than 
	// the previous one and the latter one
	double measDiffThreshold_;

	// the lasting interval of a subgroup measurements should not exceed groupInterval
	// we specify the size as time interval in milliseconds
	double groupInterval_;

	// the number of beacons used to calc the pos, if the number of visible Beacons is 
	// less than nUsedbeacon_, use all the visible Beacons.
	// -1, use all beacons to calc the pos
	// we select the nUsedBeacon_ most strong rssi values
	int nUsedBeacon_;

	// we kick out the rssi value which is lower than rssiThred_ when calculating the pos
	double rssiThred_;

	// measGroups_ contains subgroups grouped by beacon id
	std::map<std::string, std::list<BeaconMeas>> measGroups_;

	// the newest measurement timestamp
	double curTimeStamp_;

	// stores the ibeacons information
	// we query this map to get the xy coordinates by beacon id
	std::unordered_map<std::string, IBeacon> iBeaconMap_;
	
	// add a new Beacon measurement
	void addMeas(BeaconMeas curBeaconMeas);

	// prepare measurements feed to func calTriPos()
	// kick out measurement with status of -1 and use weighted moving average
	std::vector<BeaconMeas>	prepareBeaconMeas();

	std::vector<BeaconMeas>	prepareBeaconMeas1();

	// calculate coordiante, lower distance then bigger weight, closer to that beacon
	std::vector<double> calWeightPos(const std::vector<BeaconMeas> preparedBeaconMeas);

	// kalman filter
	Matrix<double, 4, 1> x_;  // States: x(m), y, Vx(m/s), Vy
	Matrix<double, 4, 4> mA_;      // Process matrix: x_k = A*x_{k-1} + w
	Matrix<double, 4, 4> mQ_;	  // Process noise covariance w~N(0,mQ)
	Matrix<double, 2, 4> mH_;	  // measurement matrix z = H*x + v
	Matrix<double, 2, 2> mR_;	  // measurement nosie covariance v~N(0,mR)

	Matrix<double, 4, 4> mP_;	  // Estimate error covariance

	double dt_;					  // Time step, in milliseconds

	int kfState_;			      // 0, wait for initial state guess
								  // 1, we have initial state guess, let's use this kf
	std::vector<std::vector<double>> initPoses_;  // we need the first 4 calculated coordinates to guess the initail pos
};

} // namespace BIP
