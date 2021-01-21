
#include "FSL.h"

namespace fsl
{

	unsigned char &Voxel::Get(int x, int y, int z)
	{
		x = (x >= sx) ? sx - 1 : ((x < 0) ? 0 : x);
		y = (y >= sy) ? sy - 1 : ((y < 0) ? 0 : y);
		z = (z >= sz) ? sx - 1 : ((z < 0) ? 0 : z);
		return voxel[x][y][z];
	}

	void Voxel::Set(int x, int y, int z, unsigned char value)
	{
		x = (x >= sx) ? sx - 1 : ((x < 0) ? 0 : x);
		y = (y >= sy) ? sy - 1 : ((y < 0) ? 0 : y);
		z = (z >= sz) ? sx - 1 : ((z < 0) ? 0 : z);
		voxel[x][y][z] = value;
	}

	unsigned char &Voxel::USGet(int x, int y, int z)
	{
		return voxel[x][y][z];
	}

	void Voxel::USSet(int x, int y, int z, unsigned char value)
	{
		voxel[x][y][z] = value;
	}

	Voxel::Voxel(int x, int y, int z)
	{
		x = (x < 1) ? 1 : x; y = (y < 1) ? 1 : y; z = (z < 1) ? 1 : z;
		sx = x;
		sy = y;
		sz = z;
		voxel = new unsigned char**[x];
		for (int i = 0; i < x; i++)
		{
			voxel[i] = new unsigned char*[y];
			for (int j = 0; j < y; j++)
			{
				voxel[i][j] = new unsigned char[z];
			}
		}
	}

	void Voxel::Destroy()
	{
		for (int i = 0; i < sx; i++)
		{
			for (int j = 0; j < sy; j++)
			{
				delete[] voxel[i][j];
			}
			delete[] voxel[i];
		}
		delete[] voxel;
	}

	unsigned char*** Voxel::GetLink()
	{
		return voxel;
	}

	/*Voxel::~Voxel()
	{
		delete[] voxel;
	}*/
}