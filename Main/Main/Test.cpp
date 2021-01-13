#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "FSL.h"
#include <string>

int main()
{
	std::string filepath = "E:\\Temp\\In\\TestHotoMobile\\1\\";

	std::string fnames[7]{ "1.jpg","2.jpg", "3.jpg", "4.jpg", "5.jpg", "6.jpg", "7.jpg" };

	cv::Mat Mim[7];
	for (int i = 0; i < 7; i++)
	{
		Mim[i] = cv::imread(filepath + fnames[i], cv::IMREAD_GRAYSCALE);
	}

	unsigned char ***imgs = new uch**[7];

	int maxx = Mim[0].rows, maxy = Mim[0].cols;

	for (int i = 0; i < 7; i++)
	{
		imgs[i] = new unsigned char*[maxx];
		for (int x = 0; x < maxx; x++)
		{
			imgs[i][x] = new unsigned char[maxy]; 
			for (int y = 0; y < maxy; y++) imgs[i][x][y] = Mim[i].at<uch>(x, y);
		}
	}

	fsl::InitFrame(7, imgs, maxx, maxy);
	fsl::Prepare();
	fsl::GetFirsCamPos();
	fsl::GetBestedBorder();
	fsl::GetNewCamPos();
	fsl::GetBorderDisp();
	fsl::UpdateOreint();
	fsl::GetFoot();
	fsl::GetFirstVoxel();
	fsl::BestTop();
};