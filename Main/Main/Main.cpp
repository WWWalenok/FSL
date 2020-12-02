// Main.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include <thread>


namespace fsl
{

#define VoxelX 297
#define VoxelY 210
#define VoxelZ 256

	unsigned char voxel[VoxelX][VoxelY][VoxelZ / 8];

	std::vector<unsigned char> **imgs, **blurred, **buffer;

	int framecount;

	std::vector<double*> cams, list;

	void GetFirsCamPos(int num);

	void GetNewCamPos();

	void Prepare();

	void GetBestedBorder();

	void GetFirstVoxel();

	void MainFunc();

	void SetVoxelPoint(int x, int y, int z, bool val);

	bool GetVoxelPoint(int x, int y, int z);

	void MainFunc()
	{
		Prepare();

		std::thread **threads = new std::thread*[framecount - 1];

		double **rets = new double*[framecount];

		for (int i = 1; i < framecount; i++) threads[i - 1] = new std::thread(GetFirsCamPos, std::ref(i));

		GetFirsCamPos(0);

		for (int i = 1; i < framecount; i++) threads[i - 1]->join;

		GetBestedBorder();

		GetNewCamPos();

		GetFirstVoxel();
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