#include "Beacon.h"

Beacon::Beacon()
	: x_(0.), y_(0.), id_("0"), rssiRef_(-59), pathLoss_(2.0), height_(0.0)
{
}

Beacon::Beacon(const Beacon& beacon)
	:x_(beacon.x_)
	,y_(beacon.y_)
	,id_(beacon.id_)
	,rssiRef_(beacon.pathLoss_)
	,pathLoss_(beacon.pathLoss_)
	,height_(beacon.height_)
{
}

void Beacon::setBeaconId(const std::string& beaconId)
{
	id_ = beaconId;
}

void Beacon::setX(const double x)
{
	x_ = x;
}

void Beacon::setY(const double y)
{
	y_ = y;
}

void Beacon::setRssiPref(const int pref)
{
	rssiRef_ = pref;
}

void Beacon::setPathLoss(const double pathLoss)
{
	pathLoss_ = pathLoss;
}

void Beacon::setHeight(const double height)
{
	height_ = height;
}

const char* Beacon::getId() const
{
	return id_.c_str();
}

double Beacon::getX() const
{
	return x_;
}

double Beacon::getY() const
{
	return y_;
}

int Beacon::getRssiPref() const
{
	return rssiRef_;
}

double Beacon::getPathLoss() const
{
	return pathLoss_;
}

double Beacon::getHeight() const
{
	return height_;
}

IBeacon::IBeacon()
	:Beacon(), major_(0), minor_(0) 
{
}

IBeacon::IBeacon(const IBeacon& iBeacon)
	:Beacon(iBeacon), uuid_(iBeacon.uuid_),
major_(iBeacon.major_), minor_(iBeacon.minor_)
{
}

void IBeacon::setUuid(const char * uuid)
{
	uuid_ = uuid;
}

void IBeacon::setMajor(const unsigned int major)
{
	major_ = major;
}

void IBeacon::setMinor(const unsigned int minor)
{
	minor_ = minor;
}

const char * IBeacon::getUuid() const
{
	return uuid_.c_str();
}


unsigned IBeacon::getMajor() const
{
	return major_;
}

unsigned IBeacon::getMinor() const
{
	return minor_;
}

BeaconMeas::BeaconMeas()
	:beaconPtr_(nullptr)
{
}

BeaconMeas::BeaconMeas(Beacon* beacon, double rssi)
{
	beaconPtr_ = beacon;
	rssi_ = rssi;
}

BeaconMeas::BeaconMeas(const BeaconMeas& beaconMeas)
	: beaconId_(beaconMeas.beaconId_)
	, beaconPtr_(beaconMeas.beaconPtr_)
	, rssi_(beaconMeas.rssi_)
{
}

BeaconMeas::~BeaconMeas()
{
	beaconPtr_ = nullptr;
}

void BeaconMeas::setBeaconId(const std::string & beaconId)
{
	beaconId_ = beaconId;
}

void BeaconMeas::setBeaconPtr(const Beacon* beaconPtr)
{
	beaconPtr_ = const_cast<Beacon*> (beaconPtr);
}

void BeaconMeas::setRssi(const double rssi)
{
	rssi_ = rssi;
}


const char * BeaconMeas::getBeaconId() const
{
	return beaconId_.c_str();
}

Beacon* BeaconMeas::getBeaconPtr() const
{
	return beaconPtr_;
}

double BeaconMeas::getRssi() const
{
	return rssi_;
}

double BeaconMeas::calDistFromRssi() const
{
	return pow(10, (beaconPtr_->getRssiPref() - rssi_) / (10 * beaconPtr_->getPathLoss()));
}





