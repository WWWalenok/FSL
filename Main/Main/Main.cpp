// Main.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include <thread>
#include "Vectors.h"


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

#define PI 3.141592653589793
#define VoxelX 400
#define VoxelY 400
#define VoxelZ 256
#define LineDisp 16
#define LineDeep 3

#define random rand() / (double)RAND_MAX

	unsigned char voxel[VoxelX][VoxelY][VoxelZ / 8];

	std::vector<Img> imgs, blurred, buffer;

	int framecount, immaxX, immaxY;

	std::vector<Vector3*> cams, lists, foots;

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

		for (int u = 0; u < framecount; u++)
		{
			int t = 0;
			Img temp(immaxX, immaxY, imgs[u].Value());

			int gistogramm[256];
			for (int i = 0; i < 256; i++) gistogramm[i] = 0;

			for (int x = 0; x < immaxX; x++)
			{
				for (int y = 0; x < immaxY; y++)
				{
					if (y != 0 && x != 0 && y != immaxY && x != immaxX)
					{
						t = 0;
						for (int _x = -1; _x < 2; _x++)for (int _y = -1; _y < 2; _y++) t += imgs[u].Get(x + _x, y + _y);
						t = t / 9;
						temp.Set(x, y, t);
					}
					else
						temp.Set(x, y, imgs[u].Get(x, y));

					gistogramm[temp.Get(x, y)]++;
				}
			}

			int max = 255, min = 0, kmax = 0, kmin = 0, k = immaxX * immaxY / 20;

			for (int i = 0; i < 256; i++)
			{
				if (kmin < k)
				{
					min = i;
					kmin += gistogramm[min];
				}
				if (kmax < k)
				{
					max = 255 - i;
					kmax += gistogramm[max];
				}
			}

			double up = (max - min) / 256.0;

			for (int x = 0; x < immaxX; x++)
			{
				for (int y = 0; x < immaxY; y++)
				{
					temp.Set(x, y, (int)((temp.Get(x, y) - min) * up));
				}
			}

			blurred.push_back(temp);
		}



	}

	void GetFirsCamPos()
	{
		for (int u = 0; u < framecount; u++)
		{
			double K = 43.26661530556787151743 / sqrt(immaxX * immaxX + immaxY * immaxY);
			double tr = 17, tr2 = tr * 1, tr3 = tr * 1, w1 = 1, w2 = 0.7, w3 = 0.2, r;
			int t = 0;
			Vector3 *cam = new Vector3[5];
			Vector3 *list = new Vector3[4];
			Vector3 d[2];
			Vector3 s[2], b[2], bb[2], m[2]; // select, best, bestbest, median
			double f, bf, bbf, a, ba, bba, df, mf, da, ma; // focus, angel
			double Krit[5], KritB[5], KritBB[5];
			double Krite[4], KriteB[4], KriteBB[4];
			for (int i = 0; i < 4; i++) KritBB[i] = -1e20;
			for (int i = 0; i < 4; i++) KriteBB[i] = -1e20;
			for (int E = 0; E < 100; E++)
			{
				f = 35; bf = 35; a = 0; ba = 0;
				b[0].x = 0; b[0].y = 0; b[0].z = 350;
				b[1].x = 0; b[1].y = 0; b[1].z = 0;
				s[0] = b[0]; s[1] = b[1]; m[0] = b[0]; m[1] = b[1];
				d[0].x = 500, d[0].y = 500, d[0].z = 200;
				d[1].x = 200, d[1].y = 200, d[1].z = 0;
				da = PI / 12;
				ma = 0;
				df = 35;
				mf = 45;
				for (int i = 0; i < 4; i++) KritB[i] = -1e20;
				for (int i = 0; i < 4; i++) KriteB[i] = -1e20;
				for (int I = 0; I < 400; I++)
				{
					unsigned char buff[LineDisp][LineDeep * 2 + 1];
					Vector3 P, A, B, C, D;
					double _x, _y;
					for (int i = 0; i < 2; i++)
					{
						s[i].x = (random > 0.7) ? b[i].x : m[i].x + (2 - random) * d[i].x;
						s[i].y = (random > 0.7) ? b[i].y : m[i].y + (2 - random) * d[i].y;
						s[i].z = (random > 0.7) ? b[i].z : m[i].z + (2 - random) * d[i].z;
					}
					a = (random > 0.7) ? ba : ma + (2 - random) * da;
					f = (random > 0.7) ? bf : mf + (2 - random) * df;
					cam[0] = s[0];
					cam[3] = s[1] - s[0]; cam[3].SetLenght(1);
					cam[2] = cam[3] / Vector3(0, 0, -1);
					cam[2].z = a; cam[2].SetLenght(1);
					cam[1] = cam[3] / cam[2];
					list[0] = Vector3(198.5, 105, 0) - s[0];
					list[1] = Vector3(-198.5, 105, 0) - s[0];
					list[2] = Vector3(-198.5, -105, 0) - s[0];
					list[3] = Vector3(198.5, -105, 0) - s[0];
					for (int i = 0; i < 4; i++)
					{
						list[i] = Vector3(list[i] * cam[1], list[i] * cam[2], list[i] * cam[3]); list[i] = list[i] / list[i].z * f / K;
						list[i].x -= immaxX / 2; list[i].y -= immaxY / 2;
					}
					for (int i = 0; i < 4; i++)
					{
						A = list[i];
						B = list[(i + 1 > 3) ? i + 1 - 4 : i];
						C = list[(i + 2 > 3) ? i + 2 - 4 : i];
						D = list[(i + 3 > 3) ? i + 3 - 4 : i];
						for (int x = 0; x < LineDisp; x++)
						{
							_x = (x + 1) / (LineDisp + 1.0);
							for (int y = 0; y < LineDeep * 2 + 1; y++)
							{
								_y = (y - LineDeep) / (LineDeep * 30.0);
								P = A * (1 - _x)*(1 - _y) + B * (_x)*(1 - _y) + C * (_x)*(_y)+D * (1 - _x)*(_y);
								if (P.x < 0) { P.x = 0; }
								if (P.y < 0) { P.y = 0; }
								if (P.y > immaxX) { P.y = immaxX - 1; }
								if (P.x > immaxY) { P.x = immaxY - 1; }
								t = blurred[u].Get((int)P.y, (int)P.x);
								buff[x][y] = (t < 0) ? 0 : ((t > 255) ? 255 : t);
							}
						}
						Krit[i] = 0; Krite[i] = 0;
						for (int x = 0; x < LineDisp; x++)
						{
							r = ((buff[x][4] + buff[x][5] + buff[x][6]) / 3.0 - (buff[x][0] + buff[x][1] + buff[x][2]) / 3.0);
							if (r > tr3)
							{
								Krit[i] = Krit[i] + w3;
								if (abs(i - LineDisp / 2) >= (LineDisp*0.25))
									Krite[i] = Krite[i] + w3;
							}
							r = ((buff[x][4] + buff[x][5]) / 2.0 - (buff[x][1] + buff[x][2]) / 2.0);
							if (r > tr2)
							{
								Krit[i] = Krit[i] + w2;
								if (abs(i - LineDisp / 2) >= (LineDisp*0.25))
									Krite[i] = Krite[i] + w2;
							}
							r = ((buff[x][4]) - (buff[x][2]));
							if (r > tr)
							{
								Krit[i] = Krit[i] + w1;
								if (abs(i - LineDisp / 2) >= (LineDisp*0.25))
									Krite[i] = Krite[i] + w1;
							}
						}
					}
					double Kr = 0;
					for (int i = 0; i < 4; i++) Kr += pow(Krit[i], 0.0333);
					if (Kr > KritB[4])
					{
						KritB[4] = Kr;
						for (int i = 0; i < 4; i++) KritB[i] = Krit[i];
						b[0] = s[0]; b[1] = s[1];
						bf = f; ba = a;
					}
					if (I % 2 == 1)
					{
						for (int i = 0; i < 2; i++)
						{
							m[i].x += ((b[i].x > m[i].x) ? d[i].x * 0.01 : -d[i].x * 0.01);
							d[i].x *= 0.99;
							m[i].y += ((b[i].y > m[i].y) ? d[i].y * 0.01 : -d[i].y * 0.01);
							d[i].y *= 0.99;
							m[i].z += ((b[i].z > m[i].z) ? d[i].z * 0.01 : -d[i].z * 0.01);
							d[i].z *= 0.99;
						}
						mf += ((bf > mf) ? df * 0.01 : -df * 0.01);
						df *= 0.99;
						ma += ((ba > ma) ? da * 0.01 : -da * 0.01);
						da *= 0.99;
					}
				}
				if (KritB[4] > KritBB[4])
				{
					KritBB[4] = KritB[4];
					for (int i = 0; i < 4; i++) KritBB[i] = KritB[i];
					bb[0] = b[0]; bb[1] = b[1];
					bbf = bf; bba = ba;
				}
			}
			cam[0] = bb[0];
			cam[3] = bb[1] - bb[0]; cam[3].SetLenght(1);
			cam[2] = cam[3] / Vector3(0, 0, -1);
			cam[2].z = bba; cam[2].SetLenght(1);
			cam[1] = cam[3] / cam[2];

			list[0] = Vector3(198.5, 105, 0) - s[0];
			list[1] = Vector3(-198.5, 105, 0) - s[0];
			list[2] = Vector3(-198.5, -105, 0) - s[0];
			list[3] = Vector3(198.5, -105, 0) - s[0];
			for (int i = 0; i < 4; i++)
			{
				list[i] = Vector3(list[i] * cam[1], list[i] * cam[2], list[i] * cam[3]); list[i] = list[i] / list[i].z * f / K;
				list[i].x -= immaxX / 2; list[i].y -= immaxY / 2;
			}

		}
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