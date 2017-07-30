#include "Estimator.h"

BIP::Estimator::Estimator()
	: posX_(0.0), posY_(0.0), posZ_(0.0)
	, measDiffThreshold_(10.0)
	, groupInterval_(2000)
	, nUsedBeacon_(-1)
	, rssiThred_(-90)
	, curTimeStamp_(0)
	, dt_(1000)
	, kfState_(0)
{
}

BIP::Estimator::Estimator(std::string beaconFilename)
	: posX_(0.0), posY_(0.0), posZ_(0.0)
	, measDiffThreshold_(10.0)
	, groupInterval_(2000)
	, nUsedBeacon_(-1)
	, rssiThred_(-90)
	, curTimeStamp_(0)
	, dt_(1000)
	, kfState_(0)
{
	std::ifstream beaconFile(beaconFilename);
	if (beaconFile.is_open())
	{
		std::string beaconLine;
		IBeacon ibeacon;
		// same uuid + major
		if (getline(beaconFile, beaconLine))
			ibeacon.setUuid(beaconLine.c_str());
		if (getline(beaconFile, beaconLine))
			ibeacon.setMajor(std::stoi(beaconLine));
		// different minor
		while (getline(beaconFile, beaconLine))
		{
			size_t sz1, sz2 = 0;
			int minor = stoi(beaconLine, &sz1);
			double x = stod(beaconLine.substr(sz1 + 3), &sz2);
			double y = stod(beaconLine.substr(sz1 + 3 + sz2 + 3));
			ibeacon.setMinor(minor);
			ibeacon.bindId();
			ibeacon.setX(x);
			ibeacon.setY(y);
			std::pair<std::string, IBeacon> thisIBeacon(ibeacon.getId(), ibeacon);
			iBeaconMap_.insert(thisIBeacon);
		}
		beaconFile.close();
	}

	else std::cout << "Unable to open file: " << beaconFilename;

}


BIP::Estimator::Estimator(const Estimator& est)
	: posX_(est.posX_), posY_(est.posY_), posZ_(est.posZ_)
	, measDiffThreshold_(est.measDiffThreshold_)
	, groupInterval_(est.groupInterval_)
	, nUsedBeacon_(est.nUsedBeacon_)
	, rssiThred_(est.rssiThred_)
	, curTimeStamp_(est.curTimeStamp_)
	, iBeaconMap_(est.iBeaconMap_)
	, dt_(est.dt_)
{
}


double BIP::Estimator::getMeasDiffThred() const
{
	return measDiffThreshold_;
}


void BIP::Estimator::setMeasDiffThred(double th)
{
	measDiffThreshold_ = th;
}

void BIP::Estimator::setGroupInterval(const double gi)
{
	groupInterval_ = gi;
}

void BIP::Estimator::setNUsedBeacon(const int n)
{
	nUsedBeacon_ = n;
}

void BIP::Estimator::setDt(const double dt)
{
	dt_ = dt;
}

void BIP::Estimator::setRssiThred(const double rssiThred)
{
	rssiThred_ = rssiThred;
}

void BIP::Estimator::addMeas(BeaconMeas curBeaconMeas)
{
	// filter out the unknown beacon
	auto got = iBeaconMap_.find(curBeaconMeas.getBeaconId());
	// this beacon is not listed in beaconMap
	if (got == iBeaconMap_.end())
		return;

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

	auto it = measGroups_.find(curBeaconMeas.getBeaconId());
	if ( it != measGroups_.end() )
	{// if measGroups_ contains the Beacon group which emitted this curBeaconMeas, we add the meas to the group
		auto measList = &it->second;
		measList->push_back(curBeaconMeas);
		// the new curBeaconMeas's status is unknown
		auto listIt = --measList->end();
		listIt->setStatus(0);
		// kick out the rssi value which is lower than rssiThred_
		if (listIt->getRssi() <= rssiThred_)
		{
			listIt->setStatus(-1);
		}

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
		measGroups_.insert(std::pair<std::string, std::list<BeaconMeas>>(curBeaconMeas.getBeaconId(), newGroup));
	}

	// update curTimeStamp_
	curTimeStamp_ = curBeaconMeas.getTimeStamp();
}

std::vector<double> BIP::Estimator::calPos()
{
	std::vector<BeaconMeas> smoothedBM = prepareBeaconMeas1();
	if (smoothedBM.size() == 0)
		return std::vector<double>(2, 0.0);
	// log
	/*for (auto item : smoothedBM)
		std::cout << item.getBeaconPtr()->getId() << ": " << std::fixed <<std::setprecision(0)<<item.getTimeStamp() << "  " << item.getRssi() << "\n";
*/
	return calWeightPos(smoothedBM);
}

std::vector<double> BIP::Estimator::calWeightPos(const std::vector<BeaconMeas> preparedBeaconMeas)
{
	std::vector<double> posXY(2, 0.0);
	double normalizeCoefficient = 0.0;
	//take revert values, because lower distance then bigger weight
	for (unsigned int i = 0; i < preparedBeaconMeas.size(); i++)
                normalizeCoefficient += 1.0 / fabs(preparedBeaconMeas[i].getDist() );

	std::vector <double> weight(preparedBeaconMeas.size(), 0.0);

	for (unsigned int i = 0; i < preparedBeaconMeas.size(); i++)
	{
		auto got = iBeaconMap_.find(preparedBeaconMeas[i].getBeaconId());
		// this beacon is not listed in beaconMap
		if (got == iBeaconMap_.end())
		{
			std::cout << preparedBeaconMeas[i].getBeaconId() << "not found in map\n";
			continue;
		}

		// calculate probability of being at beacons x,y coordinates
		weight[i] += 1.0 / (fabs(preparedBeaconMeas[i].getDist() *
			normalizeCoefficient));

		double beaconX = got->second.getX();
		double beaconY = got->second.getY();

		//find final coordinates according to probability
		posXY.at(0) += weight[i] * beaconX;
		posXY.at(1) += weight[i] * beaconY;
	}
	posX_ = posXY.at(0);
	posY_ = posXY.at(1);
	return posXY;
}


// WMA_m = (n*p_m + (n-1)*p_{m-1} + ... + p_{m-n+1})/(n + (n-1) + ... + 1)
std::vector<BIP::BeaconMeas> BIP::Estimator::prepareBeaconMeas()
{
	std::vector<BeaconMeas> preparedBeaconMeas;

	// TODO: size = 0 case
	if (measGroups_.size() == 0)
		std::cout << "measgroup contain no data.\n";

	for (auto it = measGroups_.begin(); it != measGroups_.end(); ++it)
	{
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
		BeaconMeas tmp(it->first, wma, measList.back().getTimeStamp());
		preparedBeaconMeas.push_back(tmp);
	}
	// sort preparedBeaconMeas into ascending order (e.g. from -100 to 0)
	std::sort(preparedBeaconMeas.begin(), preparedBeaconMeas.end());

	// we get the nUsedBeacon_ most strong rssi values;
	if (nUsedBeacon_ != -1 && nUsedBeacon_ <= preparedBeaconMeas.size())
	{
		std::vector<BeaconMeas> split(preparedBeaconMeas.end() - nUsedBeacon_, preparedBeaconMeas.end());
		return split;
	}
	return preparedBeaconMeas;
}

// exponential moving average£¬EMA
std::vector<BIP::BeaconMeas> BIP::Estimator::prepareBeaconMeas1()
{
	std::vector<BeaconMeas> preparedBeaconMeas;

	// TODO: size = 0 case
	if (measGroups_.size() == 0)
		std::cout << "measgroup contain no data.\n";

	for (auto it = measGroups_.begin(); it != measGroups_.end(); ++it)
	{
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

		double alpha = 2 / (n + 1);

		for (int i = n; ; --listIt)
		{
			if (listIt->getStatus() == 1)
			{
				sum += pow(1 - alpha, n-i) *  listIt->getRssi();
				--i;
			}
			if (listIt == measList.begin())
				break;
		}

		if (n == 0)
			continue;

		double denominator = 0;
		for (int i = 0; i < n; i++)
			denominator += pow(1 - alpha, i);

		double wma = sum / denominator;

		// TODO: remove this log file
		/*if (strcmp(it->second.front().getBeaconPtr()->getId(), "19:18:FC:00:E6:58") == 0)
		{
		std::ofstream curRssiFile("curRssi.txt", std::ofstream::app);
		curRssiFile << wma << "\n";
		curRssiFile.close();
		}*/


		// we have a delay of one measurement
		BeaconMeas tmp(it->first, wma, measList.back().getTimeStamp());
		preparedBeaconMeas.push_back(tmp);
	}
	// sort preparedBeaconMeas into ascending order (e.g. from -100 to 0)
	std::sort(preparedBeaconMeas.begin(), preparedBeaconMeas.end());

	// we get the nUsedBeacon_ most strong rssi values;
	if (nUsedBeacon_ != -1 && nUsedBeacon_ <= preparedBeaconMeas.size())
	{
		std::vector<BeaconMeas> split(preparedBeaconMeas.end() - nUsedBeacon_, preparedBeaconMeas.end());
		return split;
	}
	return preparedBeaconMeas;
}


double BIP::Estimator::getGroupInterval() const
{
	return groupInterval_;
}

int BIP::Estimator::getNUsedbeacon() const
{
	return nUsedBeacon_;
}

double BIP::Estimator::getRssiThred() const
{
	return rssiThred_;
}

void BIP::Estimator::addMeas(std::string beaconId, double rssi, double timeStamp)
{
	BeaconMeas bm(beaconId, rssi, timeStamp);
	addMeas(bm);
}

std::vector<double> BIP::Estimator::est()
{
	std::vector<double> rawPos = calPos();

	// everything is ready, go smoothing the rawPos
	if (kfState_ == 1)
	{
		double z[] = { rawPos[0], rawPos[1] };
		// smooth this position using kf
		// Time update(predict)
		Matrix<double, 4, 1> x_hat = mA_ * x_;
		Matrix<double, 4, 4> mP_hat = mA_ * mP_ * mA_.transpose() + mQ_;
		// Measurement update(correct)
		SquareMatrix<double, 2> S_I = SquareMatrix<double, 2>(mH_ * mP_hat * mH_.transpose() + mR_).I();
		Matrix<double, 4, 2> mK = mP_hat * mH_.transpose() * S_I;
		Matrix<double, 2, 1> mz(z);
		x_ = x_hat + mK*(mz - mH_*x_hat);
		mP_ = mP_hat - mK*mH_*mP_hat;

		// return the filtered result
		std::vector<double> kfOut;
		kfOut.push_back(x_(0, 0));
		kfOut.push_back(x_(1, 0));
		return kfOut;
	}

	// we need to initialize the kf first
	if (kfState_ == 0)
	{
		int len = 4;
		if (static_cast<int>(initPoses_.size()) >= len)
		{ // we get enough initial pos
			std::vector<double> posAvg;
			double sumX = 0, sumY = 0;
			for (auto it = initPoses_.begin() + 1; it != initPoses_.end(); ++it)
			{
				sumX += (*it)[0];
				sumY += (*it)[1];
			}
			posAvg.push_back(sumX / (len-1));
			posAvg.push_back(sumY / (len-1));

			// initialize the kalman filter
			double states[] = { posAvg[0], posAvg[1], 0, 0 };
			x_ = Matrix<double, 4, 1>(states);
			double mA[] = { 1,0,dt_/1000,0,0,1,0,dt_/1000,0,0,1,0,0,0,0,1 };
			mA_ = Matrix<double, 4, 4>(mA);
			double q = 0.1;
			double mQ[] = { q,0,0,0,0,q,0,0,0,0,q,0,0,0,0,q };
			mQ_ = Matrix<double, 4, 4>(mQ);
			double mH[] = { 1,0,0,0,0,1,0,0 };
			mH_ = Matrix<double, 2, 4>(mH);
			double r = 20;
			double mR[] = { r,0,0,r };
			mR_ = Matrix<double, 2, 2>(mR);
			double mP[] = { 0,0,0,0 };
			mP_ = Matrix<double, 4, 4>(mP);

			// kf initialization complete 
			kfState_ = 1;
			initPoses_.clear();
		}
		else
		{
			initPoses_.push_back(rawPos);
		}
		return rawPos;
	}

	return std::vector<double>(2, 0.0);
}
