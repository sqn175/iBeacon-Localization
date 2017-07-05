#include "Algorithm.h"

BIP::Trilateration::Trilateration()
	: dim_(2)
	, posX_(0.0), posY_(0.0), posZ_(0.0)
	, groupInterval_(2000)
{
}

double BIP::Trilateration::getPosX() const
{
	return posX_;
}

double BIP::Trilateration::getPosY() const
{
	return posY_;
}

double BIP::Trilateration::getPosZ() const
{
	return posZ_;
}

int BIP::Trilateration::getDim() const
{
	return dim_;
}

void BIP::Trilateration::setDim(const int dim)
{
	dim_ = dim;
}

void BIP::Trilateration::setGroupInterval(const double gi)
{
	groupInterval_ = gi;
}

void BIP::Trilateration::calPos(const std::vector<BeaconMeas> preparedBeaconMeas)
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

	if ( dim_ == 2)
	{	// we do not consider the height of device, x,y coordinate to be calculated
		// we calculate distance using calcPlanarDistFromRssi()
		double xFirstBeacon = 0, yFirstBeacon = 0;
		xFirstBeacon = preparedBeaconMeas[0].getBeaconPtr()->getX();
		yFirstBeacon = preparedBeaconMeas[0].getBeaconPtr()->getY();

		double firstBeaconDistance = preparedBeaconMeas.front().calcPlanarDistFromRssi();
		double normaFirstBeacon = xFirstBeacon * xFirstBeacon +
			yFirstBeacon * yFirstBeacon;

		for (int i = 0; i < numVisibleBeacons - 1; i++)
		{
			// fill the matrix A and right part of linear system b
			double x = 0.0, y = 0.0;
			x = preparedBeaconMeas[i + 1].getBeaconPtr()->getX();
			y = preparedBeaconMeas[i + 1].getBeaconPtr()->getY();

			double distance = preparedBeaconMeas[i + 1].calcPlanarDistFromRssi();

			matrixA[i * dim_] = 2 * (x - xFirstBeacon);
			matrixA[i * dim_ + 1] = 2 * (y - yFirstBeacon);

			double norma = x * x + y * y;
			b[i] = firstBeaconDistance * firstBeaconDistance - distance * distance -
				normaFirstBeacon + norma;
		}
	}else if (dim_ == 3)
	{	// x,y,z coordinates to be calculated
		// we calculate distance using calcDistFromRssi()
		
	}

	// TODO: three equation case
	// calculate coordinates using OLS method
	std::vector<double> vectorXY = solveLinearSystem(matrixA, b);

	// update device coordinates
	posX_ = vectorXY[0];
	posY_ = vectorXY[1];
}

void BIP::Trilateration::addMeas(BeaconMeas curBeaconMeas)
{
	// we kick out the measurements which timestamp is outdated
	// iterate the measGroups_
	for (auto it = measGroups_.begin(); it != measGroups_.end(); ++it)
	{
		// iterate the group, check every measurement
		for (auto itt = it->second.begin(); itt != it->second.end(); ++itt)
		{
			if (itt->getTimeStamp() < curBeaconMeas.getTimeStamp() - groupInterval_)
			{
				it->second.pop_front();
			}
			else
			{
				break;
			}
		}

	}

	auto it = measGroups_.find(curBeaconMeas.getBeaconPtr()->getId());
	if ( it != measGroups_.end() )
	{// if measGroups_ contains the Beacon group which emitted this curBeaconMeas, we add the meas to the group
		it->second.push_back(curBeaconMeas);
	}
	else
	{// we insert a new group to measGroup_
		std::list<BeaconMeas> newGroup({curBeaconMeas });
		measGroups_.insert(std::pair<std::string, std::list<BeaconMeas>>(curBeaconMeas.getBeaconPtr()->getId(), newGroup));
	}

}

std::vector<double> BIP::Trilateration::solveLinearSystem(std::vector<double> matrixA, std::vector<double> b)
{
	int nOfEquations = b.size();
	int dim = matrixA.size() / nOfEquations;

	std::vector <double> xy(dim, 0.);
	std::vector <double> aTransposeA(dim * dim, 0.);

	// find pseudoInverseMatrix
	for (int row = 0; row < dim; row++)
	{
		for (int col = 0; col < dim; col++)
		{
			for (int inner = 0; inner < nOfEquations; inner++)
			{
				// Multiply the row of A_transpose by the column of A 
				// to get the row, column of multiplyAATranspose.
				aTransposeA[row * dim + col] +=
					matrixA[inner * dim + row] * matrixA[inner * dim + col];
			}
		}
	}

	std::vector <double> revertMatrix(dim * dim, 0.);
	double det = aTransposeA[0] * aTransposeA[3] -
		aTransposeA[2] * aTransposeA[1];

	//simple formula for invert matrix 2x2
	revertMatrix[0] = aTransposeA[3] / det;
	revertMatrix[1] = -aTransposeA[1] / det;
	revertMatrix[2] = -aTransposeA[2] / det;
	revertMatrix[3] = aTransposeA[0] / det;

	//Multiply revertMatrix on A transpose
	std::vector <double> matrix2xN(dim * nOfEquations, 0.0);
	for (int row = 0; row < dim; row++)
	{
		for (int col = 0; col < nOfEquations; col++)
		{
			for (int inner = 0; inner < dim; inner++)
			{
				// Multiply the row of A_transpose by the column of A 
				// to get the row, column of multiplyAATranspose.
				matrix2xN[row * nOfEquations + col] +=
					revertMatrix[row * dim + inner] * matrixA[col * dim + inner];
			}
		}
	}

	//Multiply matrix2xN on B vector 
	for (int col = 0; col < dim; col++)
	{
		for (int inner = 0; inner < nOfEquations; inner++)
		{
			xy[col] += matrix2xN[col * nOfEquations + inner] * b[inner];
		}
	}
	return xy;
}

std::vector<BIP::BeaconMeas> BIP::Trilateration::prepareBeaconMeas()
{
	// currently return the last element of every subgroup of measgroups
	std::vector<BeaconMeas> preparedBeaconMeas;
	for (auto it = measGroups_.begin(); it != measGroups_.end(); ++it)
	{
		preparedBeaconMeas.push_back(it->second.back());
	}
	return preparedBeaconMeas;
}


double BIP::Trilateration::getGroupInterval() const
{
	return groupInterval_;
}
