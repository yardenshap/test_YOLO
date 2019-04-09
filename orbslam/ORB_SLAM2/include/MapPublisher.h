#ifndef MAPPUBLISHER_H
#define MAPPUBLISHER_H

#include <vector>
#include <iostream>

#include "MapPoint.h"
#include "Map.h"
#include <sstream>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{


class MapPublisher
{
public:
	MapPublisher(Map* pMap):mpMap(pMap) {};

	void PublishPoints(int attempts, int redCones, int blueCones)
	{
		cv::Mat REDpoints, BLUEpoints;
		if(MakeConeMap(attempts, redCones, blueCones, REDpoints, BLUEpoints))
		{
			cout << "inside if" << "\n";
			cout << "mappublisher" << REDpoints << endl;
			cout << "mappublisher" << BLUEpoints << endl;
		}
		else
			cout << "not inside if" << endl;
	};

private:
	bool MakeConeMap(int attempts, int redCones, int blueCones, cv::Mat &REDpoints, cv::Mat &BLUEpoints)
	{
		const int dim = 2; // TODO: maybe try with 3
		const vector<MapPoint*> &vpCs = mpMap->GetAllConePoints();
		int N = vpCs.size();

		// number of points from the map
		int NRED = 0, NBLUE = 0;
		// for all cone -> get matrices of RED and BLUE
		for(int i = 0; i < N; i++)
		{
			// postion in world
			cv::Mat cPos = vpCs[i]->GetWorldPos();
			float tmp[dim] = {cPos.at<float>(0, 1), cPos.at<float>(0, 2)};//TODO:reomve line
			cv::Mat dimPos(1, dim, CV_32F, tmp);

			if (vpCs[i]->mnConeType == 1) // RED
			{
				REDpoints.push_back(dimPos);
				NRED++;
				// REDpoints.push_back(Mat1d(*data, *(data+1), *(data+2)))
				// the dim 
			}
			else if(vpCs[i]->mnConeType == 2) // BLUE
			{
				BLUEpoints.push_back(dimPos);
				NBLUE++;
			}
			else
				// TODO:change to ignore
				cout << "big problem, the map point given should be a cone\n";
		}

		if (!NRED && !NBLUE)
			return false;
		return true;
	}

	Map* mpMap;
};





} //namespace ORB_SLAM



#endif // MAPPUBLISHER_H