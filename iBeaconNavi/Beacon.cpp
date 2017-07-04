#include "Beacon.h"

Beacon::Beacon()
	: id_("0"), x_(0.), y_(0.), height_(0.0), rssiRef_(-59), pathLoss_(2.0)
{
}

Beacon::Beacon(const Beacon& beacon)
	:id_(beacon.id_)
	,x_(beacon.x_)
	,y_(beacon.y_)
	,height_(beacon.height_)
	,rssiRef_(beacon.pathLoss_)
	,pathLoss_(beacon.pathLoss_)
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

void Beacon::setHeight(const double height)
{
	height_ = height;
}

void Beacon::setRssiPref(const double rssiRef)
{
	rssiRef_ = rssiRef;
}

void Beacon::setPathLoss(const double pathLoss)
{
	pathLoss_ = pathLoss;
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

double Beacon::getHeight() const
{
	return height_;
}

double Beacon::getRssiRef() const
{
	return rssiRef_;
}

double Beacon::getPathLoss() const
{
	return pathLoss_;
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


int IBeacon::getMajor() const
{
	return major_;
}

int IBeacon::getMinor() const
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
	: beaconPtr_(beaconMeas.beaconPtr_)
	, rssi_(beaconMeas.rssi_)
{
}

BeaconMeas::~BeaconMeas()
{
	beaconPtr_ = nullptr;
}

void BeaconMeas::setBeaconPtr(const Beacon* beaconPtr)
{
	beaconPtr_ = const_cast<Beacon*> (beaconPtr);
}

void BeaconMeas::setRssi(const double rssi)
{
	rssi_ = rssi;
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
	return pow(10, (beaconPtr_->getRssiRef() - rssi_) / (10 * beaconPtr_->getPathLoss()));
}

double BeaconMeas::calPlanarDistFromRssi() const
{
	double distance = calDistFromRssi();
	return sqrt(distance*distance - getBeaconPtr()->getHeight()*getBeaconPtr()->getHeight());
}





