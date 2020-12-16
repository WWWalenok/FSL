#pragma once

namespace fsl
{
	class Img
	{
	public:

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
			for (int i = 0; i < x; i++) for (int j = 0; j < y; j++) val[i + x * j] = frame[i][j];
		}

		~Img()
		{
			delete[] val;
		}
	private:

		unsigned char *val;
		int x, y, l;
	};

	class Voxel
	{
		unsigned char *voxel;

		int sx, sy, sz;

	public:
		unsigned char &Get(int x, int y, int z)
		{
			x = (x >= sx) ? sx - 1 : ((x < 0) ? 0 : x);
			y = (y >= sy) ? sy - 1 : ((y < 0) ? 0 : y);
			z = (z >= sz) ? sx - 1 : ((z < 0) ? 0 : z);
			return voxel[x * sy * sz + y * sz + z];
		}

		void Set(int x, int y, int z, unsigned char value)
		{
			x = (x >= sx) ? sx - 1 : ((x < 0) ? 0 : x);
			y = (y >= sy) ? sy - 1 : ((y < 0) ? 0 : y);
			z = (z >= sz) ? sx - 1 : ((z < 0) ? 0 : z);
			voxel[x * sy * sz + y * sz + z] = value;
		}

		unsigned char &USGet(int x, int y, int z)
		{
			return voxel[x * sy * sz + y * sz + z];
		}

		void USSet(int x, int y, int z, unsigned char value)
		{
			voxel[x * sy * sz + y * sz + z] = value;
		}

		Voxel(int x, int y, int z)
		{
			x = (x < 1) ? 1 : x; y = (y < 1) ? 1 : y; z = (z < 1) ? 1 : z;
			sx = x;
			sy = y;
			sz = z;
			voxel = new unsigned char[x * y * z];
		}

		~Voxel()
		{
			delete[] voxel;
		}

	};

	struct PhisicPoint
	{
		Vector2 loc, v, dv;
	};

#define PI 3.141592653589793
#define VoxelX 400
#define VoxelY 256
#define VoxelZ 256
#define VoxelS 1.0
#define pointsDisp 16
#define pointsDeep 3

#define random rand() / (double)RAND_MAX

	void GetCamPos(int);

	void GetFirsCamPos();

	void GetNewCamPos();

	void Prepare();

	void GetBestedBorder();

	void GetBorderDisp_new();

	void GetBorderDisp();

	void GetFirstVoxel();

	void GetFoot();

	void GetBestVoxel();

	void Out();

	void UpdateOreint();

	void InitEtalon(std::vector<Vector3*> _male, std::vector<int> _malesizes, std::vector<Vector3*> _female, std::vector<int> _femalesizes);

	void InitFrame(int inframecount, unsigned char ***inframes, int frameX, int frameY);
}