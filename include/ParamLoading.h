#include <string>
#include <vector>

using namespace std;

class ParamLoading
{
public:
	vector<string> vstrImageFilenamesRGB;
	vector<string> vstrImageFilenamesDepth;
	vector<double> vTimestamps;
	int nImages;

	ParamLoading(const string strInFilePath);
	void LoadImages();
	void LoadPose();

private:
	string strFilePath;
	string strAssociationFile;
	string strRGBPath;
	string strDepthPath;
};