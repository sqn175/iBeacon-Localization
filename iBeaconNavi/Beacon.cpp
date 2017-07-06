#include "Beacon.h"

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
{
}

void BIP::Beacon::setBeaconId(const std::string& beaconId)
{
	id_ = beaconId;
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

const char* BIP::Beacon::getId() const
{
	return id_.c_str();
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
	:beaconPtr_(nullptr)
{
}

BIP::BeaconMeas::BeaconMeas(Beacon* beacon, double rssi)
{
	beaconPtr_ = beacon;
	rssi_ = rssi;
}

BIP::BeaconMeas::BeaconMeas(const BeaconMeas& beaconMeas)
	: beaconPtr_(beaconMeas.beaconPtr_)
	, rssi_(beaconMeas.rssi_)
{
}

BIP::BeaconMeas::~BeaconMeas()
{
	beaconPtr_ = nullptr;
}

void BIP::BeaconMeas::setBeaconPtr(const Beacon* beaconPtr)
{
	beaconPtr_ = const_cast<Beacon*> (beaconPtr);
}

void BIP::BeaconMeas::setRssi(const double rssi)
{
	rssi_ = rssi;
}

void BIP::BeaconMeas::setTimeStamp(const double timeStamp)
{
	timeStamp_ = timeStamp;
}

BIP::Beacon* BIP::BeaconMeas::getBeaconPtr() const
{
	return beaconPtr_;
}

double BIP::BeaconMeas::getRssi() const
{
	return rssi_;
}

const double BIP::BeaconMeas::getTimeStamp() const
{
	return timeStamp_;
}

double BIP::BeaconMeas::calcDistFromRssi() const
{
	return pow(10, (beaconPtr_->getRssiRef() - rssi_) / (10 * beaconPtr_->getPathLoss()));
}

double BIP::BeaconMeas::calcPlanarDistFromRssi() const
{
	double distance = calcDistFromRssi();
	// TODO: check if distance > height
	return sqrt(distance*distance - getBeaconPtr()->getHeight()*getBeaconPtr()->getHeight());
}





