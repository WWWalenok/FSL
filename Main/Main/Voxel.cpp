
#include "FSL.h"
using namespace fsl;

unsigned char &Voxel::Get(int x, int y, int z)
{
	x = (x >= sx) ? sx - 1 : ((x < 0) ? 0 : x);
	y = (y >= sy) ? sy - 1 : ((y < 0) ? 0 : y);
	z = (z >= sz) ? sx - 1 : ((z < 0) ? 0 : z);
	return voxel[x * sy * sz + y * sz + z];
}

void Voxel::Set(int x, int y, int z, unsigned char value)
{
	x = (x >= sx) ? sx - 1 : ((x < 0) ? 0 : x);
	y = (y >= sy) ? sy - 1 : ((y < 0) ? 0 : y);
	z = (z >= sz) ? sx - 1 : ((z < 0) ? 0 : z);
	voxel[x * sy * sz + y * sz + z] = value;
}

unsigned char &Voxel::USGet(int x, int y, int z)
{
	return voxel[x * sy * sz + y * sz + z];
}

void Voxel::USSet(int x, int y, int z, unsigned char value)
{
	voxel[x * sy * sz + y * sz + z] = value;
}

Voxel::Voxel(int x, int y, int z)
{
	x = (x < 1) ? 1 : x; y = (y < 1) ? 1 : y; z = (z < 1) ? 1 : z;
	sx = x;
	sy = y;
	sz = z;
	voxel = new unsigned char[x * y * z];
}

Voxel::~Voxel()
{
	delete[] voxel;
}
