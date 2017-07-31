#include "Beacon.h"

#include <math.h>

BIP::Beacon::Beacon()
	: id_("0"), x_(0.), y_(0.), height_(0.0), rssiRef_(-59), pathLoss_(2.0)
{
}

BIP::Beacon::Beacon(const Beacon& beacon)
	: id_(beacon.id_)
	, x_(beacon.x_)
	, y_(beacon.y_)
	, height_(beacon.height_)
	, rssiRef_(beacon.rssiRef_)
	, pathLoss_(beacon.pathLoss_)
	, mac_(beacon.mac_)
{
}

void BIP::Beacon::bindId()
{
	id_ = mac_;
}

void BIP::Beacon::setX(const double x)
{
	x_ = x;
}

void BIP::Beacon::setY(const double y)
{
	y_ = y;
}

void BIP::Beacon::setHeight(const double height)
{
	height_ = height;
}

void BIP::Beacon::setRssiPref(const double rssiRef)
{
	rssiRef_ = rssiRef;
}

void BIP::Beacon::setPathLoss(const double pathLoss)
{
	pathLoss_ = pathLoss;
}

void BIP::Beacon::setMacAddr(const std::string& mac)
{
	mac_ = mac;
}

const char* BIP::Beacon::getId() const
{
	return mac_.c_str();
}

double BIP::Beacon::getX() const
{
	return x_;
}

double BIP::Beacon::getY() const
{
	return y_;
}

double BIP::Beacon::getHeight() const
{
	return height_;
}

double BIP::Beacon::getRssiRef() const
{
	return rssiRef_;
}

double BIP::Beacon::getPathLoss() const
{
	return pathLoss_;
}

BIP::IBeacon::IBeacon()
	:Beacon(), major_(0), minor_(0) 
{
}

BIP::IBeacon::IBeacon(const IBeacon& iBeacon)
	:Beacon(iBeacon), uuid_(iBeacon.uuid_),
major_(iBeacon.major_), minor_(iBeacon.minor_)
{
}

const char* BIP::IBeacon::getId() const
{
	return id_.c_str();
}

void BIP::IBeacon::bindId()
{
	id_ = uuid_ + std::to_string(major_) + std::to_string(minor_);
}

void BIP::IBeacon::setUuid(const char * uuid)
{
	uuid_ = uuid;
}

void BIP::IBeacon::setMajor(const unsigned int major)
{
	major_ = major;
}

void BIP::IBeacon::setMinor(const unsigned int minor)
{
	minor_ = minor;
}

const char * BIP::IBeacon::getUuid() const
{
	return uuid_.c_str();
}


int BIP::IBeacon::getMajor() const
{
	return major_;
}

int BIP::IBeacon::getMinor() const
{
	return minor_;
}

BIP::BeaconMeas::BeaconMeas()
	: rssi_(0.0)
	, dist_(0.0)
	, timeStamp_(0.0)
	, status_(0)
{
}

BIP::BeaconMeas::BeaconMeas(std::string id, double rssi, double timeStamp)
{
	beaconId_ = id;
	rssi_ = rssi;
	timeStamp_ = timeStamp;
	dist_ = this->calcDistFromRssi();
	status_ = 0;
}

BIP::BeaconMeas::BeaconMeas(const BeaconMeas& beaconMeas)
	: beaconId_(beaconMeas.beaconId_)
	, rssi_(beaconMeas.rssi_)
	, dist_(beaconMeas.dist_)
	, timeStamp_(beaconMeas.timeStamp_)
	, status_(beaconMeas.status_)
{
}

BIP::BeaconMeas::~BeaconMeas()
{
	
}

void BIP::BeaconMeas::setBeaconId(const char* id)
{
	beaconId_ = id;
}

void BIP::BeaconMeas::setRssi(const double rssi)
{
	rssi_ = rssi;
	dist_ = calcDistFromRssi();
}

void BIP::BeaconMeas::setTimeStamp(const double timeStamp)
{
	timeStamp_ = timeStamp;
}

void BIP::BeaconMeas::setStatus(const int stat)
{
	status_ = stat;
}

const char* BIP::BeaconMeas::getBeaconId() const
{
	return beaconId_.c_str();
}

double BIP::BeaconMeas::getRssi() const
{
	return rssi_;
}

double BIP::BeaconMeas::getDist() const
{
	return dist_;
}

double BIP::BeaconMeas::getTimeStamp() const
{
	return timeStamp_;
}

int BIP::BeaconMeas::getStatus() const
{
	return status_;
}

bool BIP::BeaconMeas::operator<(const BeaconMeas& entry) const
{
	return rssi_ < entry.rssi_;
}

bool BIP::BeaconMeas::operator>(const BeaconMeas& entry) const
{
	return entry < *this;
}

double BIP::BeaconMeas::calcDistFromRssi() const
{
	// we specify this two parameters
	// in fact, we dont have time to calibrate this two parameters of every iBeacon
	double rssiRef = -59;
	double pathLoss = 2.5;
	return pow(10, (rssiRef - rssi_) / (10 * pathLoss));
}






