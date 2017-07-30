#include "Estimator.h"
#include "Beacon.h"

#include <fstream>
#include <iostream>
#include <vector>
#include <iomanip>
#include <string>

void rawest()
{
	int measCnt = 0;  // iBeacon measurement count

					  // initialize algorithm
	BIP::Estimator tri("deqingBeacon.txt");
	tri.setGroupInterval(1500);
	tri.setMeasDiffThred(10);
	tri.setNUsedBeacon(-1);
	tri.setRssiThred(-100);

	// rssi value file
	std::string fileName = "F:\\iBeacon\\20170726deqing\\scan_bluetoothline11.txt";

	// read beacon measurements from file
	std::ifstream bleScanFile(fileName);

	// Result file
	std::ofstream posFile("posraw.csv");

	// write beacon1 rssi value to file, check the rssi value distribution
	std::ofstream staticRssiFile("rssi.txt");

	if (!bleScanFile.is_open())
	{
		std::cerr << "File: " << fileName << " does NOT exist\n";
		std::cout << "Press any key to exit.\n";
		getchar();
	}

	std::string line;
	double timeBefore = 0;
	while (std::getline(bleScanFile, line))
	{
		// A new measurement arrrives
		std::string header = line.substr(0, 1);
		if (strcmp(header.c_str(), "1") == 0)
		{
			measCnt++;
			std::cout << "------New Measurement:" << measCnt << "------\n";
			double timeStamp = stod(line);
			getline(bleScanFile, line);
			getline(bleScanFile, line);
			getline(bleScanFile, line);
			std::string uuid = line.substr(5);
			getline(bleScanFile, line);
			// read mac address of the iBeacon emitting this measurement
			std::string minor = line.substr(18, 5);
			std::string major = line.substr(6, 5);
			std::string id = uuid + major + minor;

			getline(bleScanFile, line);
			double rssi = stoi(line.substr(17, 3));

			//if (strcmp(id.c_str(), "uuid1008053279") == 0)
			//{
			//	// write rssi value to txt file
			//	staticRssiFile << line.substr(17, 3) << "\n";
			//}

			tri.addMeas(id, rssi, timeStamp);

			if (timeStamp > timeBefore + 1000)
			{
				std::vector<double> curPos = tri.calPos();
				std::cout << "Result: x=" << std::fixed << std::setprecision(5) << curPos.at(0) << "  y=" << curPos.at(1) << "\n\n";
				posFile << std::fixed << std::setprecision(6) << curPos.at(0) << "," << curPos.at(1) << "\n";
				timeBefore = timeStamp;
			}
		}
	}

	bleScanFile.close();

	staticRssiFile.close();

	posFile.close();

}

void smoothest()
{
	int measCnt = 0;  // iBeacon measurement count

					  // initialize algorithm
	BIP::Estimator estimator("deqingBeacon.txt");
	estimator.setGroupInterval(1500);
	estimator.setMeasDiffThred(10);
	estimator.setNUsedBeacon(-1);
	estimator.setRssiThred(-100);
	estimator.setDt(1000);

	// rssi value file
	std::string fileName = "F:\\iBeacon\\20170726deqing\\scan_bluetoothline11.txt";

	// read beacon measurements from file
	std::ifstream bleScanFile(fileName);

	// Result file
	std::ofstream posFile("possmooth.csv");

	// write beacon1 rssi value to file, check the rssi value distribution
	std::ofstream staticRssiFile("rssi.txt");

	if (!bleScanFile.is_open())
	{
		std::cerr << "File: " << fileName << " does NOT exist\n";
		std::cout << "Press any key to exit.\n";
		getchar();
	}

	std::string line;
	double timeBefore = 0;
	while (std::getline(bleScanFile, line))
	{
		// A new measurement arrrives
		std::string header = line.substr(0, 1);
		if (strcmp(header.c_str(), "1") == 0)
		{
			measCnt++;
			std::cout << "------New Measurement:" << measCnt << "------\n";
			double timeStamp = stod(line);
			getline(bleScanFile, line);
			getline(bleScanFile, line);
			getline(bleScanFile, line);
			std::string uuid = line.substr(5);
			getline(bleScanFile, line);
			// read mac address of the iBeacon emitting this measurement
			std::string minor = line.substr(18, 5);
			std::string major = line.substr(6, 5);
			std::string id = uuid + major + minor;

			getline(bleScanFile, line);
			double rssi = stoi(line.substr(17, 3));

			//if (strcmp(id.c_str(), "uuid1008053279") == 0)
			//{
			//	// write rssi value to txt file
			//	staticRssiFile << line.substr(17, 3) << "\n";
			//}

			estimator.addMeas(id, rssi, timeStamp);

			if (timeStamp > timeBefore + 1000)
			{
				std::vector<double> curPos = estimator.est();
				std::cout << "Result: x=" << std::fixed << std::setprecision(5) << curPos.at(0) << "  y=" << curPos.at(1) << "\n\n";
				posFile << std::fixed << std::setprecision(6) << curPos.at(0) << "," << curPos.at(1) << "\n";
				timeBefore = timeStamp;
			}
		}
	}

	bleScanFile.close();

	staticRssiFile.close();

	posFile.close();
	
}

int main()
{
	//rawest();
	smoothest();
	return 0;
}
