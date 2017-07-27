#include "Algorithm.h"
#include "Beacon.h"

#include <fstream>
#include <iostream>
#include <vector>
#include <iomanip>
#include <string>

int main()
{
	int measCnt = 0;  // iBeacon measurement count

	std::string fileName = "F:\\iBeacon\\20170726deqing\\scan_bluetoothline4.txt";
	std::vector<BIP::IBeacon> ibeaconsMap;
	// Initialize beacon map
	//// test scenario1. Set 3 beacons on the map
	BIP::IBeacon ibeacon;
	ibeacon.setUuid("uuid");
	ibeacon.setMajor(10103);
	//// iBeacon 1
	//ibeacon.bindId();
	//ibeacon.setX(13435840.64291);
	//ibeacon.setY(3550350.6154879997);
	//ibeaconsMap.push_back(ibeacon);
	//// iBeacon 2
	//ibeacon.setUuid("uuid");
	//ibeacon.setMajor(10080);
	//ibeacon.setMinor(53280);
	//ibeacon.bindId();
	//ibeacon.setX(13435846.923805503);
	//ibeacon.setY(3550349.5415525);
	//ibeaconsMap.push_back(ibeacon);
	//// iBeacon3
	//ibeacon.setUuid("uuid");
	//ibeacon.setMajor(10080);
	//ibeacon.setMinor(53279);
	//ibeacon.bindId();
	//ibeacon.setX(13435853.009440003);
	//ibeacon.setY(3550349.639183);
	//ibeaconsMap.push_back(ibeacon);
	//// iBeacon4
	//ibeacon.setUuid("uuid");
	//ibeacon.setMajor(10080);
	//ibeacon.setMinor(53278);
	//ibeacon.bindId();
	//ibeacon.setX(13435859.062531002);
	//ibeacon.setY(3550349.5740960003);
	//ibeaconsMap.push_back(ibeacon);

	//// Scenerio 2
	//ibeacon.setUuid("uuid");
	//ibeacon.setMajor(10080);
	//ibeacon.setMinor(53225);
	//ibeacon.bindId();
	//ibeacon.setX(13435862.056533003);
	//ibeacon.setY(3550353.6745769996);
	//ibeaconsMap.push_back(ibeacon);

	//ibeacon.setUuid("uuid");
	//ibeacon.setMajor(10080);
	//ibeacon.setMinor(53224);
	//ibeacon.bindId();
	//ibeacon.setX(13435868.109624004);
	//ibeacon.setY(3550353.7396639995);
	//ibeaconsMap.push_back(ibeacon);

	//ibeacon.setUuid("uuid");
	//ibeacon.setMajor(10080);
	//ibeacon.setMinor(53223);
	//ibeacon.bindId();
	//ibeacon.setX(13435874.130171504);
	//ibeacon.setY(3550353.642033499);
	//ibeaconsMap.push_back(ibeacon);

	//ibeacon.setUuid("uuid");
	//ibeacon.setMajor(10080);
	//ibeacon.setMinor(53247);
	//ibeacon.bindId();
	//ibeacon.setX(13435880.053088503);
	//ibeacon.setY(3550353.631185665);
	//ibeaconsMap.push_back(ibeacon);

	//ibeacon.setUuid("uuid");
	//ibeacon.setMajor(10080);
	//ibeacon.setMinor(53274);
	//ibeacon.bindId();
	//ibeacon.setX(13435883.242351502);
	//ibeacon.setY(3550349.530704665);
	//ibeaconsMap.push_back(ibeacon);

	//ibeacon.setUuid("uuid");
	//ibeacon.setMajor(10080);
	//ibeacon.setMinor(53275);
	//ibeacon.bindId();
	//ibeacon.setX(13435877.189260503);
	//ibeacon.setY(3550349.530704665);
	//ibeaconsMap.push_back(ibeacon);

	//ibeacon.setUuid("uuid");
	//ibeacon.setMajor(10080);
	//ibeacon.setMinor(53276);
	//ibeacon.bindId();
	//ibeacon.setX(13435871.103626002);
	//ibeacon.setY(3550349.628335165);
	//ibeaconsMap.push_back(ibeacon);

	//ibeacon.setUuid("uuid");
	//ibeacon.setMajor(10080);
	//ibeacon.setMinor(53277);
	//ibeacon.bindId();
	//ibeacon.setX(13435865.115622003);
	//ibeacon.setY(3550349.5957916644	);
	//ibeaconsMap.push_back(ibeacon);
	// test func calTriPos(const std::vector<BeaconMeas> preparedBeaconMeas)
	// deqing ibeacon map 
	std::ifstream beaconMap("F:\\iBeacon\\20170726deqing\\deqingBeacon.txt");
	std::string beaconLine;
	while( std::getline(beaconMap, beaconLine) )
	{
		std::size_t sz1,sz2 = 0;
		int minor = std::stoi(beaconLine, &sz1);
		double x = std::stod(beaconLine.substr(sz1+3),&sz2);
		double y = std::stod(beaconLine.substr(sz1 + 3 + sz2 + 3));
		ibeacon.setMinor(minor);
		ibeacon.bindId();
		ibeacon.setX(x);
		ibeacon.setY(y);
		ibeaconsMap.push_back(ibeacon);
	}



	BIP::Trilateration tri;
	tri.setGroupInterval(2000);
	tri.setMeasDiffThred(10);
	tri.setNUsedBeacon(100);
	std::vector<BIP::BeaconMeas> preparedBM;
	BIP::BeaconMeas bm;

	// read beacon measurements from file
	std::ifstream bleScanFile( fileName );

	// Result file
	std::ofstream posFile("positionline4.csv");

	// write beacon1 rssi value to file, check the rssi value distribution
	std::ofstream staticRssiFile("rssi.txt");

	if ( !bleScanFile.is_open() )
	{
		std::cerr << "File: " << fileName << " does NOT exist\n";
		std::cout << "Press any key to exit.\n";
		getchar();
		return 0;
	}
	std::string line;
	double timeBefore = 0;
	while ( std::getline(bleScanFile, line) )
	{

		if (measCnt == 243)
		{
			std::cout << "debug\n";
			int c = 0;
		}

		// A new measurement arrrives
		std::string header = line.substr(0, 1);
		if ( strcmp( header.c_str(), "1") == 0 )
		{
			measCnt++;
			std::cout << "------New Measurement:" << measCnt << "------\n";
			BIP::BeaconMeas curBeaconMeas;
			curBeaconMeas.setTimeStamp(stod(line));
			getline(bleScanFile, line);
			getline(bleScanFile, line);
			getline(bleScanFile, line);
			getline(bleScanFile, line);
			// read mac address of the iBeacon emitting this measurement
			std::string minor = line.substr(18, 5);
			std::string major = line.substr(6, 5);
			std::string idNow = "uuid" + major + minor;
			// console the beacon map to determine the iBeacon location
			auto it = ibeaconsMap.begin();
			for (; it < ibeaconsMap.end(); ++it )
			{
				if ( strcmp(idNow.c_str(), it->getId()) == 0)
				{
					curBeaconMeas.setBeaconPtr( &(*it) );
					break;
				}
			}

			// filter out the beacon not in the beacon map
			if (it == ibeaconsMap.end())
				continue;

			getline(bleScanFile, line);
			curBeaconMeas.setRssi(stoi(line.substr(17, 3)));

			if (strcmp(idNow.c_str(), "uuid1008053279") == 0)
			{
				// write rssi value to txt file
				staticRssiFile << line.substr(17, 3) << "\n";
			}
			// Now, we get the new beaconMeas, process it
			std::cout << "- time:" << std::fixed<<std::setprecision(0)<<curBeaconMeas.getTimeStamp() << "-\n";
			std::cout << "- Mac address:" << curBeaconMeas.getBeaconPtr()->getId() << "-\n";
			std::cout << "- rssi:" << curBeaconMeas.getRssi() << "-\n";

			tri.addMeas(curBeaconMeas);

			if (curBeaconMeas.getTimeStamp() > timeBefore + 1000)
			{
				std::vector<double> curPos = tri.calPos();
				std::cout << "Result: x=" << std::fixed << std::setprecision(5) << curPos.at(0) << "  y=" << curPos.at(1) << "\n\n";
				posFile << std::fixed << std::setprecision(6)<< curPos.at(0) << "," << curPos.at(1) << "\n";
				timeBefore = curBeaconMeas.getTimeStamp();
			}
		}
		BIP::Beacon curBeacon;
	}

	bleScanFile.close();
		
	staticRssiFile.close();

	posFile.close();

	return 0;
}
