/** Beacon.h
*
* Author: Qin Shi
* Date: 2017/06/29
* A modified version of
* https://github.com/Navigine/Indoor-navigation-algorithms
*
*/

#pragma once
#include <string>

namespace BIP {

// Beacon is transmitter emitting the signal, that we can use to obtain pseudoDistances
// and process trilateration navigation algorithms
class Beacon
{
public:
	Beacon();
	Beacon(const Beacon&);    // copy constructor
	virtual ~Beacon() {};

	//const Beacon &operator=(const Beacon &beacon);

	//specify unique beacon identifier that allows us to distinguish beacon 
	// and its measurements
	void setBeaconId(const std::string& beaconId);

	// specify x,y, beacon planar coordinates
	void setX(const double x);
	void setY(const double y);

	// specify beacon height(z coordinate)
	void setHeight(const double);

	// specify propagation model parameters
	void setRssiPref(const double);
	void setPathLoss(const double);

	// get things
	const char* getId() const;
	double getX() const;
	double getY() const;
	double getHeight() const;
	double getRssiRef() const;
	double getPathLoss() const;

private:
	std::string id_;       // id of the beacon (e.g. major+minor+uuid or mac address)
	double x_, y_;         // beacon planar coodinates
	double height_;        // vertical height of the beacon from the device(e.g. from mobile phone)
	double rssiRef_;       // reference rssi value at 1 meter distance
	double pathLoss_;      // pathloss exponents of the Bluetooth signal propagation model
};

// IBeacon is a beacon, invented by apple, that has additional fields:
// Major,Minor, UUID identifiers
class IBeacon : public Beacon
{
public:
	IBeacon();
	IBeacon(const IBeacon&); // copy constructor
	~IBeacon() {};

	void setUuid(const char* uuid);
	void setMajor(const unsigned int major);
	void setMinor(const unsigned int minor);

	const char* getUuid() const;
	int getMajor() const;
	int getMinor() const;

private:
	std::string uuid_;
	unsigned int major_;
	unsigned int minor_;
};

// class that keep measurements from certain transmitter (e.g. beacon)
class BeaconMeas
{
public:
	BeaconMeas();
	BeaconMeas(Beacon* beacon, double rssi, double timeStamp);
	BeaconMeas(const BeaconMeas&);
	~BeaconMeas();

	void setBeaconPtr(const Beacon* beaconPtr);
	void setRssi(const double rssi);
	void setTimeStamp(const double timeStamp);
	void setStatus(const int stat);

	Beacon* getBeaconPtr() const;
	double getRssi() const;
	double getDist() const;
	const double getTimeStamp() const;
	int getStatus() const;

	bool operator<(const BeaconMeas& entry)const;
	bool operator>(const BeaconMeas& entry)const;

	// calculate the corresponding distance of rssi value
	double calcDistFromRssi() const;			 // 3-dimension distance
	double calcPlanarDistFromRssi() const;       // planar distance, height ignored

private:
	Beacon* beaconPtr_;			// pointer to beacon from which we got measurement
	double rssi_;				// RSSI of the measurement
	double dist_;				// the corresponding distance of rssi
	double timeStamp_;			// timestamp of current measurement, epoch time in milliseconds, e.g.1499218065493
	int status_;				// assign an int status to every BeaconMeasurements.
								// 1, is valid
								// -1, is a outlier
								// 0, status unknown
};

} // namespace BIP