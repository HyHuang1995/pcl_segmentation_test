#include "ParamLoading.h"

#include <fstream>
#include <iostream>
#include <unistd.h>
#include <sstream>

ParamLoading::ParamLoading(const string strInFilePath):
	strFilePath(strInFilePath)
{
	cout << "ParamLoading construncted!" << endl;
	cout << "Input dataset Path is set to:\t" << strFilePath << endl;
	strAssociationFile = strInFilePath + "/associations.txt";
	strRGBPath = strInFilePath + "/rgb.txt";
	strDepthPath = strInFilePath + "/depth.txt";
	strPoseFile = strInFilePath + "/groundtruth.txt";
}

void ParamLoading::LoadImages()
{
	ifstream fin;
	fin.open(strAssociationFile.c_str());
	if (!fin)
		cout << "error!" << endl;

	while (!fin.eof())
	{
		string s;
		getline(fin, s);
		if (!s.empty())
		{
			stringstream ss;
			ss << s;
			double t;
			string sRGB, sD;
			ss >> t;
			vTimestamps.push_back(t);
			ss >> sRGB;
			vstrImageFilenamesRGB.push_back(sRGB);
			ss >> t;
			ss >> sD;
			vstrImageFilenamesDepth.push_back(sD);
		}
	}
	nImages = vstrImageFilenamesRGB.size();
	cout << "Load end! Current number of Images:\t" << nImages << endl;

}

void ParamLoading::LoadPose()
{
	ifstream fin;
	fin.open(strPoseFile.c_str());
	cout << strPoseFile.c_str() << endl;
	if (!fin)
		cout << "error!" << endl;

	string stmp;
	getline(fin, stmp);
	getline(fin, stmp);
	getline(fin, stmp);

	while (!fin.eof())
	{
		string s;
		getline(fin, s);
		if (!s.empty())
		{
			stringstream ss;
			ss << s;
			double tmp;
			for (size_t ii = 0; ii != 4; ii ++)
				ss >> tmp;
			double qx, qy, qz, qw;
			ss >> qx;
			ss >> qy;
			ss >> qz;
			ss >> qw;

			Eigen::Quaterniond q( 0, qx, qy,  qz);
			Eigen::Isometry3d T(q);
			T.pretranslate( Eigen::Vector3d( 0, 0, 0 ));
			vCamPoses.push_back( T );

		}
		getline(fin, s);
		getline(fin, s);
	}
}
