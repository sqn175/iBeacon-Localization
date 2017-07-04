#include "Algorithm.h"
#include "Beacon.h"

#include <fstream>
#include <iostream>
#include <vector>

int main()
{
	int measCnt = 0;  // iBeacon measurement count
	std::string fileName = "scan_bluetooth3.txt";
	std::vector<Beacon> beaconsMap;
	// Initialize beacon map
	Beacon beacon;
	// iBeacon 1
	beacon.setBeaconId("19:18:FC:00:E6:58");
	beacon.setX(3.60);
	beacon.setY(3.855);
	beaconsMap.push_back(beacon);
	// iBeacon 2
	beacon.setBeaconId("19:18:FC:00:E6:67");
	beacon.setX(0.0);
	beacon.setY(3.855);
	beaconsMap.push_back(beacon);
	// iBeacon3
	beacon.setBeaconId("19:18:FC:00:E6:3F");
	beacon.setX(3.60);
	beacon.setY(0.0);
	beaconsMap.push_back(beacon);

	// read beacon measurements from file
	std::ifstream bleScanFile( fileName );
	
	if ( !bleScanFile.is_open() )
	{
		std::cerr << "File: " << fileName << " does NOT exist\n";
		std::cout << "Press any key to exit.\n";
		std::getchar();
		return 0;
	}
	std::string line;
	while ( std::getline(bleScanFile, line) )
	{
		// A new measurement arrrives
		std::string header = line.substr(0, 1);
		if ( strcmp( header.c_str(), "-") == 0 )
		{
			measCnt++;
			BeaconMeas curBeaconMeas;
			std::getline(bleScanFile, line);
			std::getline(bleScanFile, line);
			// read mac address of the iBeacon emitting this measurement
			std::string macAddress = line.substr(8, 17);
			// console the beacon map to determine the iBeacon location
			for (auto it = beaconsMap.begin(); it < beaconsMap.end(); ++it )
			{
				if ( std::strcmp(macAddress.c_str(), (*it).getId()) == 0)
				{
					curBeaconMeas.setBeaconId(macAddress);
					curBeaconMeas.setBeaconPtr( &(*it) );
				}
			}
			std::getline(bleScanFile, line);
			std::getline(bleScanFile, line);
			std::getline(bleScanFile, line);
			curBeaconMeas.setRssi(std::stoi(line.substr(17, 3)));
			
		}
		Beacon curBeacon;
	}
	bleScanFile.close();
		

}