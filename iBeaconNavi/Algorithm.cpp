#include "Algorithm.h"

Trilateration::Trilateration()
{
}

void Trilateration::calPos(const std::vector<BeaconMeas> preparedBeaconMeas)
{
	int numVisibleBeacons = preparedBeaconMeas.size();
	// create matrix for triangulation linear system, that we will solve
	// for obtaining the coordinates we need at least dim + 1 beacons
	// index [i][j] = i * dim + j

	std::vector <double> matrixA((numVisibleBeacons - 1) * dim_, 0.0);
	std::vector <double> b(numVisibleBeacons - 1, 0.0);

	// By subtracting the last equation from each other we bring our 
	// nonlinear system to linear matrixA
	BeaconMeas firstBeacon = preparedBeaconMeas.front();

	// we do not consider the height of device, x,y coordinate to be calculated
	if ( dim_ == 2)
	{
		double xFirstBeacon = 0, yFirstBeacon = 0;
		xFirstBeacon = preparedBeaconMeas[0].getBeaconPtr()->getX();
		yFirstBeacon = preparedBeaconMeas[0].getBeaconPtr()->getY();



		double firstBeaconDistance = preparedBeaconMeas.front().getDistance();
		double normaFirstBeacon = xFirstBeacon * xFirstBeacon +
			yFirstBeacon * yFirstBeacon;

		for (int i = 0; i < nOfVisibleBeacons - 1; i++)
		{
			// fill the matrix A and right part of linear system b
			double x = 0.0, y = 0.0;
			x = preparedBeaconMeas[i + 1].getBeaconPtr()->getX();
			y = preparedBeaconMeas[i + 1].getBeaconPtr()->getY();

			double distance = preparedBeaconMeas[i + 1].getDistance();

			matrixA[i * dim_] = 2 * (x - xFirstBeacon);
			matrixA[i * dim_ + 1] = 2 * (y - yFirstBeacon);

			double norma = x * x + y * y;
			b[i] = firstBeaconDistance * firstBeaconDistance - distance * distance -
				normaFirstBeacon + norma;
		}
	}


}


