// Main.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include <thread>
#include <cmath>
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

	double min(double a, double b) { return (a > b) ? b : a; }
	double max(double a, double b) { return (a < b) ? b : a; }

	unsigned char voxel[VoxelX][VoxelY][VoxelZ / 8];

	std::vector<Img> imgs, blurred, buffer;

	int framecount, immaxX, immaxY;

	std::vector<Vector3*> cams, lists, foots, inworld;

	void GetCamPos(int);

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
			lists.push_back(list);
			cams.push_back(cam);
		}
	}

	void GetBestedBorder()
	{
		for (int u = 0; u < framecount; u++)
		{
			double tr = 17, tr2 = tr * 1, tr3 = tr * 1, w1 = 1, w2 = 0.7, w3 = 0.2, r;
			double Krit[5], KritB[5], KritBB[5];
			double Krite[4], KriteB[4], KriteBB[4];
			double k1, k2, mk1, mk2, dk1, dk2, k1b, k1bb, k2b, k2bb;
			for (int i = 0; i < 4; i++) KritBB[i] = -1e20;
			for (int i = 0; i < 4; i++) KriteBB[i] = -1e20;
			Vector3 P1, P2, P, A, B, C, D;
			int t;
			for (int trys = 0; trys < 2; trys++) for (int i = 0; i < 4; i++)
			{
				A = lists[u][i];
				B = lists[u][(i + 1 > 3) ? i + 1 - 4 : i];
				C = lists[u][(i + 2 > 3) ? i + 2 - 4 : i];
				D = lists[u][(i + 3 > 3) ? i + 3 - 4 : i];
				for (int E = 0; E < 5; E++)
				{
					P1 = A;
					P2 = B;
					mk1 = 1; mk2 = 2;
					dk1 = 0.5; dk2 = 0.5;
					k1 = 1; k2 = 2;
					for (int i = 0; i < 4; i++) KritB[i] = -1e20;
					for (int i = 0; i < 4; i++) KriteB[i] = -1e20;
					for (int I = 0; I < 200; I++)
					{
						unsigned char buff[LineDisp][LineDeep * 2 + 1];

						double _x, _y;

						k1 = mk1 + (2 - random) * dk1;
						k2 = mk2 + (2 - random) * dk2;

						P1 = D + (A - D) * k1;
						P2 = C + (B - C) * k2;

						for (int x = 0; x < LineDisp; x++)
						{
							_x = (x + 1) / (LineDisp + 1.0);
							for (int y = 0; y < LineDeep * 2 + 1; y++)
							{
								_y = (y - LineDeep) / (LineDeep * 30.0);
								P = P1 * (1 - _x)*(1 - _y) + P2 * (_x)*(1 - _y) + C * (_x)* (_y)+D * (1 - _x)*(_y);
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
						double Kr = 0;

						for (int i = 0; i < 4; i++) Kr += pow(Krit[i], 0.0333);

						if (Kr > KritB[4])
						{
							KritB[4] = Kr;
							for (int i = 0; i < 4; i++) KritB[i] = Krit[i];
							k1b = k1; k2b = k2;
						}
						if (I % 2 == 1)
						{

							mk1 += ((k1b > mk1) ? dk1 * 0.01 : -dk1 * 0.01);
							dk1 *= 0.99;
							mk2 += ((k2b > mk2) ? dk2 * 0.01 : -dk2 * 0.01);
							dk2 *= 0.99;
						}
					}
					if (KritB[4] > KritBB[4])
					{
						KritBB[4] = KritB[4];
						for (int i = 0; i < 4; i++) KritBB[i] = KritB[i];
						k1bb = k1b; k2bb = k2b;
					}
				}
				lists[u][i] = A;
				lists[u][(i + 1 > 3) ? i + 1 - 4 : i] = B;
				lists[u][(i + 2 > 3) ? i + 2 - 4 : i] = C;
				lists[u][(i + 3 > 3) ? i + 3 - 4 : i] = D;
			}
		}
	}

	void GetCamPos(int u)
	{
		Vector3 AT, BT, CT, DT, O;
		double dig = std::sqrt(immaxX * immaxX + immaxY * immaxY);
		double t = 43.26661530556787151743 / dig;
		Vector3 A(lists[u][0].x, lists[u][0].y, 0), B(lists[u][1].x, lists[u][1].y, 0), C(lists[u][2].x, lists[u][2].y, 0), D(lists[u][3].x, lists[u][3].y, 0), R1(immaxX / 2.0, immaxY / 2.0, 0), R2, R3, R4;
		A = (A - R1) * t; B = (B - R1) * t; C = (C - R1) * t; D = (D - R1) * t;
		R1 = A - B;
		R2 = B - C;
		R3 = C - D;
		R4 = D - A;
		double a, b, c, d, d1, d2, l1, l2, dmax = 0;
		double M[3][4], maxerr = 1e30, err;
		double lamda = 50;

		if (min(R1 * R1, R3 * R3) < min(R2 * R2, R4 * R4))
		{
			O = A;
			A = B; B = C; C = D; D = O;
		}
		int rot = 0;
		d = 6;
		Vector3 AO = A, BO = B, CO = C, DO = D;
		M[0][0] = 1; M[0][1] = -1; M[0][2] = 1; M[0][3] = 1;
		M[1][0] = A.x; M[1][1] = -B.x; M[1][2] = C.x; M[1][3] = D.x;
		M[2][0] = A.y; M[2][1] = -B.y; M[2][2] = C.y; M[2][3] = D.y;

		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)if (k != j)
			{
				t = M[k][j] / M[j][j];
				for (int i = j; i < 4; i++)
				{
					M[k][i] = M[k][i] - M[j][i] * t;
				}
			}
		for (int j = 0; j < 3; j++)
		{
			t = 1 / M[j][j];
			M[j][j] = 1;
			M[j][3] *= t;
		}
		A.z = 0; B.z = 0; C.z = 0; D.z = 0;
		a = M[0][3]; b = M[1][3]; c = M[2][3];
		lamda = -((D - A * a) * (D - C * c)) / (1 - a - c + a * c);
		lamda = sqrt(abs(lamda));

		A.z = lamda; B.z = lamda; C.z = lamda, D.z = lamda;
		R1 = A * M[0][3] - B * M[1][3]; R2 = C * M[2][3] - B * M[1][3];
		l1 = sqrt(R1*R1);
		l2 = sqrt(R2*R2);
		d = l1 / l2;

		if (rot == 4) rot = 0;

		R1 = Vector3(1, d, 0);
		R2 = Vector3(1, -d, 0);

		d1 = -(297 + d * 210) / (1 + d * d);
		d2 = -(297 - d * 210) / (1 + d * d);

		R1 = R1 * d1;
		R2 = R2 * d2;

		d1 = R1.GetLenght();
		d2 = R2.GetLenght();

		if (d1 < d2)
		{
			d1 = R1.x;
			d2 = R1.y;
		}
		else
		{
			d1 = R2.x;
			d2 = R2.y;
		}

		double t1 = (297 + d1) / l1, t2 = (210 + d2) / l2;

		d = (t1 + t2) / 2.0;

		switch (rot)
		{
		case 1:
			t = d;
			d = c * t; c = b * t; b = a * t; a = t;
			break;
		case 2:
			t = d;
			t1 = a;
			a = c * t; c = t1 * t; d = b * t; b = t;
			break;
		case 3:
			t = d;
			d = a * t; a = b * t; b = c * t; c = t;
			break;
		case 0:
			a = a * d;	b = b * d;	c = c * d;	d = d;
			break;
		default:
			break;
		}

		A = AO; B = BO; C = CO; D = DO;

		A.z = lamda; B.z = lamda; C.z = lamda, D.z = lamda;
		R1 = A * a - B * b; R2 = C * c - B * b;
		AT = A * a; BT = B * b; CT = C * c; DT = D * d; O = (AT + BT + CT + DT) / 4.0;
		R1 = AT - BT; R1 = R1 / sqrt(R1*R1); R3 = R1 / (DT - AT); R3 = R3 / sqrt(R3*R3) * ((R3.z > 0) ? -1 : 1);
		R2 = R1 / R3; if (R2 * (DT - AT) > 0) R2 = R2 * -1;

		AT = AT - O; BT = BT - O; CT = CT - O; DT = DT - O;;
		O = O * -1;
		O = Vector3(O * R1, O * R2, O * R3);

		Vector3 WA(AT * R1, AT * R2, AT * R3), WB(BT * R1, BT * R2, BT * R3), WC(CT * R1, CT * R2, CT * R3), WD(DT * R1, DT * R2, DT * R3);

		Vector3 X(1, 0, 0), Y(0, 1, 0), Z(0, 0, 1);
		X = Vector3(X * R1, X * R2, X * R3);
		Y = Vector3(Y * R1, Y * R2, Y * R3);
		Z = Vector3(Z * R1, Z * R2, Z * R3);

		if (inworld.size < u + 1)
		{
			if (inworld.size == u)
			{
				Vector3* temp = new Vector3[4]{ WA, WB, WC, WD };
				inworld.push_back(temp);
			}
		}
		else
		{
			inworld[u][0] = WA; inworld[u][1] = WB; inworld[u][2] = WC; inworld[u][3] = WD;
		}

		cams[u][0] = O; cams[u][1] = X; cams[u][2] = Y; cams[u][3] = Z;

	}

	void GetNewCamPos()
	{
		for (int u = 0; u < framecount; u++)
		{
			double tr = 17, tr2 = tr * 1, tr3 = tr * 1, w1 = 1, w2 = 0.7, w3 = 0.2, r;
			double Krit[5], KritB[5], KritBB[5];
			double Krite[4], KriteB[4], KriteBB[4];
			double k1, k2, mk1, mk2, dk1, dk2, k1b, k1bb, k2b, k2bb;
			for (int i = 0; i < 4; i++) KritBB[i] = -1e20;
			for (int i = 0; i < 4; i++) KriteBB[i] = -1e20;
			Vector3 P1, P2, P, A, B, C, D, O, X, Y, Z, U;
			int t, _x, _y;
			GetCamPos(u);
			A = inworld[u][0]; B = inworld[u][1];
			double k = 297 / (A - B).GetLenght();
			A = A * k; B * k;
			C = inworld[u][2] * k; D = inworld[u][3] * k; O = cams[u][0] * k; X = cams[u][1] * k; Y = cams[u][2] * k; Z = cams[u][3] * k;
			P1 = (A + B) / 2.0;
			U = O - P1;
			for (int i = 0; i < 4; i++)
			{
				for (int E = 0; E < 5; E++)
				{
					mk1 = 1;
					dk1 = 0.5;
					k1 = 1;
					for (int i = 0; i < 4; i++) KritB[i] = -1e20;
					for (int i = 0; i < 4; i++) KriteB[i] = -1e20;
					for (int I = 0; I < 100; I++)
					{
						int buff[LineDeep * 2 + 1];
						P2 = P1 + U * k1;
						for (int y = 0; y < LineDeep * 2 + 1; y++)
						{
							_y = (y - LineDeep) / (LineDeep * 30.0);
							P = P2 + U * _y;
							if (P.x < 0) { P.x = 0; }
							if (P.y < 0) { P.y = 0; }
							if (P.y > immaxX) { P.y = immaxX - 1; }
							if (P.x > immaxY) { P.x = immaxY - 1; }
							t = blurred[u].Get((int)P.y, (int)P.x);
							buff[y] = (t < 0) ? 0 : ((t > 255) ? 255 : t);
						}
						Krit[i] = 0; Krite[i] = 0;
						r = ((buff[4] + buff[5] + buff[6]) / 3.0 - (buff[0] + buff[1] + buff[2]) / 3.0);
						if (r > tr3)
						{
							Krit[i] = Krit[i] + w3;
							if (abs(i - LineDisp / 2) >= (LineDisp*0.25))
								Krite[i] = Krite[i] + w3;
						}
						r = ((buff[4] + buff[5]) / 2.0 - (buff[1] + buff[2]) / 2.0);
						if (r > tr2)
						{
							Krit[i] = Krit[i] + w2;
							if (abs(i - LineDisp / 2) >= (LineDisp*0.25))
								Krite[i] = Krite[i] + w2;
						}
						r = ((buff[4]) - (buff[2]));
						if (r > tr)
						{
							Krit[i] = Krit[i] + w1;
							if (abs(i - LineDisp / 2) >= (LineDisp*0.25))
								Krite[i] = Krite[i] + w1;
						}
						double Kr = 0;
						Kr += pow(Krit[i], 0.0333);
						if (Kr > KritB[4])
						{
							KritB[4] = Kr;
							for (int i = 0; i < 4; i++) KritB[i] = Krit[i];
							k1b = k1; k2b = k2;
						}
						if (I % 2 == 1)
						{
							mk1 += ((k1b > mk1) ? dk1 * 0.01 : -dk1 * 0.01);
							dk1 *= 0.99;
							mk2 += ((k2b > mk2) ? dk2 * 0.01 : -dk2 * 0.01);
							dk2 *= 0.99;
						}
					}
					if (KritB[4] > KritBB[4])
					{
						KritBB[4] = KritB[4];
						for (int i = 0; i < 4; i++) KritBB[i] = KritB[i];
						k1bb = k1b; k2bb = k2b;
					}
				}
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