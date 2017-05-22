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
