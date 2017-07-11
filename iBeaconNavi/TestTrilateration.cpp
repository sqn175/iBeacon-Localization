#include "Algorithm.h"
#include "Beacon.h"

#include <fstream>
#include <iostream>
#include <vector>
#include <iomanip>

int main()
{
	int measCnt = 0;  // iBeacon measurement count

	std::string fileName = "F:\\iBeacon\\20170712\\scan_bluetoothstatic.txt";
	std::vector<BIP::IBeacon> ibeaconsMap;
	// Initialize beacon map
	// test scenario. Set 3 beacons on the map
	BIP::IBeacon ibeacon;
	// iBeacon 1
	ibeacon.setBeaconId("19:18:FC:00:E6:58");
	ibeacon.setX(3.60);
	ibeacon.setY(3.855);
	ibeacon.setHeight(1.845); 
	ibeacon.setRssiPref(-59);
	ibeacon.setPathLoss(2.0);
	ibeaconsMap.push_back(ibeacon);
	// iBeacon 2
	ibeacon.setBeaconId("19:18:FC:00:E6:67");
	ibeacon.setX(0.0);
	ibeacon.setY(3.855);
	ibeaconsMap.push_back(ibeacon);
	// iBeacon3
	ibeacon.setBeaconId("19:18:FC:00:E6:3F");
	ibeacon.setX(3.60);
	ibeacon.setY(0.0);
	ibeaconsMap.push_back(ibeacon);

	// test func calTriPos(const std::vector<BeaconMeas> preparedBeaconMeas)
	BIP::Trilateration tri;
	std::vector<BIP::BeaconMeas> preparedBM;
	BIP::BeaconMeas bm;

	// read beacon measurements from file
	std::ifstream bleScanFile( fileName );

	// write beacon1 rssi value to file, check the rssi value distribution
	std::ofstream staticRssiFile("rssi.txt");

	// Result file
	std::ofstream posFile("position.csv");

	if ( !bleScanFile.is_open() )
	{
		std::cerr << "File: " << fileName << " does NOT exist\n";
		std::cout << "Press any key to exit.\n";
		getchar();
		return 0;
	}
	std::string line;
	while ( std::getline(bleScanFile, line) )
	{
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
			// read mac address of the iBeacon emitting this measurement
			std::string macAddress = line.substr(8, 17);
			// console the beacon map to determine the iBeacon location
			auto it = ibeaconsMap.begin();
			for (; it < ibeaconsMap.end(); ++it )
			{
				if ( std::strcmp(macAddress.c_str(), (*it).getId()) == 0)
				{
					curBeaconMeas.setBeaconPtr( &(*it) );
					break;
				}
			}

			// filter out the beacon not in the beacon map
			if (it == ibeaconsMap.end())
				continue;

			getline(bleScanFile, line);
			getline(bleScanFile, line);
			getline(bleScanFile, line);
			curBeaconMeas.setRssi(stoi(line.substr(17, 3)));

			if (strcmp(macAddress.c_str(), "19:18:FC:00:E6:58") == 0)
			{
				// write rssi value to txt file
				staticRssiFile << line.substr(17, 3) << "\n";
			}
			// Now, we get the new beaconMeas, process it
			tri.addMeas(curBeaconMeas);
	/*		std::cout << "- time:" << curBeaconMeas.getTimeStamp() << "-\n";
			std::cout << "- Mac address:" <<curBeaconMeas.getBeaconPtr()->getId() << "-\n";
			std::cout << "- rssi:" << curBeaconMeas.getRssi() << "-\n";*/
			// calculate position
			std::vector<double> curPos = tri.calPos();
			std::cout << "Result: x=" <<std::fixed << std::setprecision(5)<< curPos.at(0) << "  y=" << curPos.at(1)<<"\n\n";
			posFile << curBeaconMeas.getTimeStamp() << "," << curPos.at(0) << "," << curPos.at(1) << "\n";
		}
		BIP::Beacon curBeacon;
	}
	bleScanFile.close();
		
	staticRssiFile.close();

	posFile.close();

	return 0;
}
