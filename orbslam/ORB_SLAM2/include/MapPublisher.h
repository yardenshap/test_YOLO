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
		cv::Mat REDcenters, BLUEcenters;
		if(MakeConeMap(attempts, REDcenters, BLUEcenters))
		{
			// cout << BLUEcenters << "\n";
		}
	};

private:
	bool MakeConeMap(int attempts, int redCones, int blueCones, cv::Mat &REDcenters, cv::Mat &BLUEcenters)
	{
		const int dim = 2; // TODO: maybe try with 3
		const vector<MapPoint*> &vpCs = mpMap->GetAllConePoints();
		int N = vpCs.size();

		// make RED/BLUE points matrices
		cv::Mat REDpoints;
		cv::Mat BLUEpoints;
		int NRED = 0, NBLUE = 0;
		// for all cones -> get matrices of RED and BLUE
		for(int i = 0; i < N; i++)
		{
			// postion in world
			cv::Mat cPos = vpCs[i]->GetWorldPos();
			float tmp[dim] = {cPos.at<float>(0, 1), cPos.at<float>(0, 2)};
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
				cout << "big problem, the map point given should be a cone\n";
		}
		if (!NRED && !NBLUE)
			return false;

		// kmeans
		
		// RED
		if(NRED)
		{
			cv::Mat REDlabels;
			cv::kmeans(REDpoints, redCones, REDlabels,
						 cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 10, 1.0),
						 attempts, cv::KMEANS_PP_CENTERS, REDcenters);
		}
		cout << REDcenters<<endl<<endl;

		// BLUE
		if(NBLUE)
		{
			cv::Mat BLUElabels;
			cv::kmeans(BLUEpoints, blueCones, BLUElabels,
						 cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 10, 1.0),
						 attempts, cv::KMEANS_PP_CENTERS, BLUEcenters);
		}

		return true;
	}

	Map* mpMap;
};





} //namespace ORB_SLAM



#endif // MAPPUBLISHER_H