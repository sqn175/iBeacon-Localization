#include "Algorithm.h"
#include "Beacon.h"

#include <fstream>
#include <iostream>
#include <vector>

int main()
{
	int measCnt = 0;  // iBeacon measurement count

	std::string fileName = "scan_bluetooth3.txt";
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

	bm.setBeaconPtr(&ibeaconsMap[0]);
	bm.setRssi(-59);
	preparedBM.push_back(bm);

	/*bm.setBeaconPtr(&ibeaconsMap[1]);
	bm.setRssi(-76);
	preparedBM.push_back(bm);*/

	bm.setBeaconPtr(&ibeaconsMap[2]);
	bm.setRssi(-57); 
	preparedBM.push_back(bm); 

//	tri.calWeightPos(preparedBM);

	// read beacon measurements from file
	std::ifstream bleScanFile( fileName );

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
	while ( std::getline(bleScanFile, line) )
	{
		// A new measurement arrrives
		std::string header = line.substr(0, 1);
		if ( strcmp( header.c_str(), "-") == 0 )
		{
			measCnt++;
			BIP::BeaconMeas curBeaconMeas;
			getline(bleScanFile, line);
			getline(bleScanFile, line);
			// read mac address of the iBeacon emitting this measurement
			std::string macAddress = line.substr(8, 17);
			// console the beacon map to determine the iBeacon location
			for (auto it = ibeaconsMap.begin(); it < ibeaconsMap.end(); ++it )
			{
				if ( std::strcmp(macAddress.c_str(), (*it).getId()) == 0)
				{
					curBeaconMeas.setBeaconPtr( &(*it) );
				}
			}
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
			

		}
		BIP::Beacon curBeacon;
	}
	bleScanFile.close();
		
	staticRssiFile.close();
}