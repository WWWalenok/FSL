// Main.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include <thread>


namespace fsl
{

	class Img
	{
	public:
		Img(int, int);

		Img(int, int, unsigned char*, int = 0);

		Img(int, int, unsigned char**, int = 0);

		~Img();

		int Get(int, int);

		void Set(int, int, int);

		void Clear();

		unsigned char *&Value();

		int X();

		int Y();

		int L();

		int Get(int _x, int _y)
		{
			_x = (_x >= x) ? x - 1 : ((_x < 0) ? 0 : x);
			_y = (_y >= y) ? y - 1 : ((_y < 0) ? 0 : y);
			return val[_x + x * _y];
		}

		void Set(int _x, int _y, int value)
		{
			_x = (_x >= x) ? x - 1 : ((_x < 0) ? 0 : x);
			_y = (_y >= y) ? y - 1 : ((_y < 0) ? 0 : y);
			val[_x + x * _y] = (value > 255) ? 255 : ((value < 0) ? 0 : value);
		}

		void Clear() { for (int i = 0; i < l; i++) val[i] = 0; }

		int X() { return x; }

		int Y() { return y; }

		int L() { return l; }

		unsigned char *&Value()
		{
			return val;
		}

		Img(int x, int y)
		{
			Img::x = x;
			Img::y = y;
			l = x * y;
			val = new unsigned char[l];
			Clear();
		}

		Img(int x, int y, unsigned char* frame, int oor = 0)
		{
			Img::x = x;
			Img::y = y;
			l = x * y;
			val = new unsigned char[l];
			for (int i = 0; i < x; i++) for (int j = 0; j < y; j++) val[i + x * j] = frame[i + x * j];
		}

		Img(int x, int y, unsigned char** frame, int oor = 0)
		{
			Img::x = x;
			Img::y = y;
			l = x * y;
			val = new unsigned char[l];
			for(int i = 0; i < x; i++) for (int j = 0; j < y; j++) val[i + x * j] = frame[i][j];
		}

		~Img()
		{
			delete[] val;
		}
	private:

		unsigned char *val;
		int x, y, l;
	};


#define VoxelX 400
#define VoxelY 400
#define VoxelZ 256

	unsigned char voxel[VoxelX][VoxelY][VoxelZ / 8];

	std::vector<Img> imgs, blurred, buffer;

	int framecount, immaxX, immaxY;

	std::vector<double*> cams, lists, foots;

	void GetFirsCamPos();

	void GetNewCamPos();

	void Prepare();

	void GetBestedBorder();

	void GetFirstVoxel();

	void GetFoot();

	void GetBestVoxel();

	void Out();

	void MainFunc();

	void SetVoxelPoint(int x, int y, int z, bool val);

	bool GetVoxelPoint(int x, int y, int z);

	void Prepare()
	{

	}

	void MainFunc(int inframecount, unsigned char ***inframes, int frameX, int frameY)
	{
		immaxX = frameX;
		immaxY = frameY;

		framecount = inframecount;

		if (imgs.size() > 0)
		{
			for (int i = 0; i < imgs.size(); i++) imgs[i].~Img();
			imgs.clear();
		}

		for (int f = 0; f < framecount; f++)
		{
			imgs.push_back(Img(frameX, frameY, inframes[f]));
		}

		Prepare();

		GetFirsCamPos();

		GetBestedBorder();

		GetNewCamPos();

		GetFoot();

		GetFirstVoxel();

		GetBestVoxel();

		Out();
	}

	void SetVoxelPoint(int x, int y, int z, bool val)
	{
		voxel[x][y][z >> 3] = (val) ? voxel[x][y][z >> 3] | (1 << (z & 8)) : voxel[x][y][z >> 3] & (~(1 << (z & 8)));
	}

	bool GetVoxelPoint(int x, int y, int z)
	{
		return (voxel[x][y][z >> 3] & (1 << (z & 8)) == 0) ? false : true;
	}

}

using namespace fsl;

int main()
{
    
}