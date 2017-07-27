#include "Algorithm.h"

#include <algorithm>
#include <math.h>       /* fabs */
#include <iostream>
#include <iomanip>
#include <string.h>

BIP::Trilateration::Trilateration()
	: posX_(0.0), posY_(0.0), posZ_(0.0)
	, groupInterval_(2000)
	, curTimeStamp(0)
	, measDiffThreshold_(10.0)
	, nUsedBeacon_(-1)
	, rssiThred_(-90)
{
}

BIP::Trilateration::Trilateration(const Trilateration& tri)
	: posX_(tri.posX_), posY_(tri.posY_), posZ_(tri.posZ_)
	, groupInterval_(tri.groupInterval_)
	, curTimeStamp(tri.curTimeStamp)
	, measDiffThreshold_(tri.measDiffThreshold_)
	, nUsedBeacon_(tri.nUsedBeacon_)
	, rssiThred_(tri.rssiThred_)
{
}


double BIP::Trilateration::getMeasDiffThred() const
{
	return measDiffThreshold_;
}


void BIP::Trilateration::setMeasDiffThred(double th)
{
	measDiffThreshold_ = th;
}

void BIP::Trilateration::setGroupInterval(const double gi)
{
	groupInterval_ = gi;
}

void BIP::Trilateration::setNUsedBeacon(const int n)
{
	nUsedBeacon_ = n;
}

void BIP::Trilateration::setRssiThred(const double rssiThred)
{
	rssiThred_ = rssiThred;
}

void BIP::Trilateration::addMeas(BeaconMeas curBeaconMeas)
{
	// TODO: filter out the unknown beacon
	// TODO: filter out wrong data e.g. -128dBm

	// we kick out the measurements which timestamp is outdated
	// the corresponding status is also updated 

	// iterate the measGroups_ first
	for (auto it = measGroups_.begin(); it != measGroups_.end(); )
	{
		// iterate the group then, check every measurement
		// measurements stored in list sorted by timestamp
		bool eraseGroup = false;
		auto listBM = &it->second;

		auto itt = listBM->begin();
		while ( itt->getTimeStamp() < curBeaconMeas.getTimeStamp() - groupInterval_ )
		{
			itt = listBM->erase(itt);
			// all the measurement in this list is outdated
			// clear the list 
			if (itt == listBM->end())
			{
				eraseGroup = true;
				break;
			}
		}
		// update the map element
		if (eraseGroup)
		{
			measGroups_.erase(it++);
		}
		else
		{
			++it;
		}
	}

	auto it = measGroups_.find(curBeaconMeas.getBeaconPtr()->getId());
	if ( it != measGroups_.end() )
	{// if measGroups_ contains the Beacon group which emitted this curBeaconMeas, we add the meas to the group
		auto measList = &it->second;
		measList->push_back(curBeaconMeas);
		// the new curBeaconMeas's status is unknown
		auto listIt = --measList->end();
		listIt->setStatus(0);
		// we define the status of the last measurement but one by measDiffThreshold_
		if (measList->size() >= 3)
		{
			bool flag1 = abs((*listIt).getDist() - (*--listIt).getDist()) >= measDiffThreshold_;
			bool flag2 = abs((*listIt).getDist() - (*--listIt).getDist()) >= measDiffThreshold_;
			if (flag1 && flag2)
			{
				(*++listIt).setStatus(-1);
			}
			else
				(*++listIt).setStatus(1);
		}
	}
	else
	{// we insert a new group to measGroup_
		// the first measurement, we set it as valid
		curBeaconMeas.setStatus(0);
		std::list<BeaconMeas> newGroup({curBeaconMeas });
		measGroups_.insert(std::pair<std::string, std::list<BeaconMeas>>(curBeaconMeas.getBeaconPtr()->getId(), newGroup));
	}

	// update curTimeStamp
	curTimeStamp = curBeaconMeas.getTimeStamp();
}

std::vector<double> BIP::Trilateration::calPos()
{

	std::vector<BeaconMeas> smoothedBM = prepareBeaconMeas();
	if (smoothedBM.size() == 0)
		return std::vector<double>(2, 0.0);

	if (smoothedBM.size() > 3)
	{
		std::vector<BeaconMeas> mostStrongMeas;
		for (int i = smoothedBM.size() - 1; i >= 0; --i)
		{
			mostStrongMeas.push_back(smoothedBM.at(i));
		}
		return calWeightPos(mostStrongMeas);
	}

	// log
	/*for (auto item : smoothedBM)
		std::cout << item.getBeaconPtr()->getId() << ": " << std::fixed <<std::setprecision(0)<<item.getTimeStamp() << "  " << item.getRssi() << "\n";
*/
	return calWeightPos(smoothedBM);
}

std::vector<double> BIP::Trilateration::calWeightPos(const std::vector<BeaconMeas> preparedBeaconMeas)
{
	std::vector<double> posXY(2, 0.0);
	double normalizeCoefficient = 0.0;
	//take revert values, because lower distance then bigger weight
	for (unsigned int i = 0; i < preparedBeaconMeas.size(); i++)
                normalizeCoefficient += 1.0 / fabs(preparedBeaconMeas[i].getDist() );

	std::vector <double> weight(preparedBeaconMeas.size(), 0.0);

	for (unsigned int i = 0; i < preparedBeaconMeas.size(); i++)
	{
		if (preparedBeaconMeas[i].getBeaconPtr() == nullptr)
		{
			// TODO: log
		}
		// calculate probability of being at beacons x,y coordinates
		weight[i] += 1.0 / (fabs(preparedBeaconMeas[i].getDist() *
			normalizeCoefficient));

		double beaconX = preparedBeaconMeas[i].getBeaconPtr()->getX();
		double beaconY = preparedBeaconMeas[i].getBeaconPtr()->getY();

		//find final coordinates according to probability
		posXY.at(0) += weight[i] * beaconX;
		posXY.at(1) += weight[i] * beaconY;
	}
	posX_ = posXY.at(0);
	posY_ = posXY.at(1);
	return posXY;
}

// WMA_m = (n*p_m + (n-1)*p_{m-1} + ... + p_{m-n+1})/(n + (n-1) + ... + 1)
std::vector<BIP::BeaconMeas> BIP::Trilateration::prepareBeaconMeas()
{
	std::vector<BeaconMeas> preparedBeaconMeas;

	// TODO: size = 0 case
	if (measGroups_.size() == 0)
		std::cout << "measgroup contain no data.\n";

	int index = 0;
	for (auto it = measGroups_.begin(); it != measGroups_.end(); ++it)
	{
		if (nUsedBeacon_ != -1 && index >= nUsedBeacon_)
			break;
		++index;
		auto measList = it->second;
		int n = measList.size();
		if (n == 0)
			std::cout << "list contain no data.\n";
		auto listIt = measList.begin();
		for (; listIt != measList.end(); ++listIt)
		{
			if (listIt->getStatus() != 1)
				--n;
		}
		double sum = 0;
		--listIt;

		for (int i = n; ; --listIt)
		{
			if (listIt->getStatus() == 1)
			{
				sum += i * listIt->getRssi();
				--i;
			}
			if (listIt == measList.begin())
				break;
		}

		if (n == 0)
			continue;

		double wma = sum / (n * (n + 1) / 2);

		// TODO: remove this log file
		/*if (strcmp(it->second.front().getBeaconPtr()->getId(), "19:18:FC:00:E6:58") == 0)
		{
			std::ofstream curRssiFile("curRssi.txt", std::ofstream::app);
			curRssiFile << wma << "\n";
			curRssiFile.close();
		}*/


		// we have a delay of one measurement
		BeaconMeas tmp(measList.back().getBeaconPtr(), wma, measList.back().getTimeStamp());
		preparedBeaconMeas.push_back(tmp);
	}
	// sort preparedBeaconMeas into ascending order (e.g. from -100 to 0)
	std::sort(preparedBeaconMeas.begin(), preparedBeaconMeas.end());
	return preparedBeaconMeas;
}


double BIP::Trilateration::getGroupInterval() const
{
	return groupInterval_;
}

int BIP::Trilateration::getNUsedbeacon() const
{
	return nUsedBeacon_;
}

double BIP::Trilateration::getRssiThred() const
{
	return rssiThred_;
}
