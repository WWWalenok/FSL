// Main.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "FSL.h"
#include "FSLModules.h"

namespace fsl
{
#pragma region LocalVars

	double min(double a, double b) { return (a > b) ? b : a; }
	double max(double a, double b) { return (a < b) ? b : a; }
	double K, devia[8]{ 0,0,0,0,0,0,0,0 };
	Voxel voxel;

	std::vector<Img> imgs, blurred, counter;

	int framecount, immaxX, immaxY, *usedCamera, *oreint;

	std::vector<Vector3*> cams, lists, foots, inworld;
	std::vector<double> focuss;

	std::vector<double***> listsborder;

	Vector3 *sphNoga;
	double *sphNogaR;

	std::vector<Vector3*> males, females;
	std::vector<int> malesizes, femalesizes;

#pragma endregion

	void ToCam(Vector3 &base, int camnum)
	{
		base.x -= cams[camnum][0].x; base.y -= cams[camnum][0].y; base.z -= cams[camnum][0].z;
		double r = base.x * cams[camnum][3].x + base.y * cams[camnum][3].y + base.z * cams[camnum][3].z;
		base.x = (base.x * cams[camnum][1].x + base.y * cams[camnum][1].y + base.z * cams[camnum][1].z) / r * focuss[camnum] / K; base.y = (base.x * cams[camnum][2].x + base.y * cams[camnum][2].y + base.z * cams[camnum][2].z) / r * focuss[camnum] / K; base.z = focuss[camnum] / K;
		base.x -= immaxX / 2; base.y -= immaxY / 2;
	}

	void Prepare()
	{
		K = 43.26661530556787151743 / sqrt(immaxX * immaxX + immaxY * immaxY);
		voxel = Voxel(VoxelX, VoxelY, VoxelZ);

		sphNoga = new Vector3[80];
		double *tx = new double[80]{ -31.586, -54.338, -79.422, -87.392, 20.206, -4.095, -82.568, -81.81, -78.205, -71.239, 12.979, -69.774, 63.242, 52.594, -40.76, -23.899, -101.968, -100.973, 97.261, -53.009, -57.968, -53.987, -50.643, 80.556, 28.291, -37.864, 68.973, 87.525, 50.309, -28.211, -6.579, -93.309, 75.315, -48.179, -68.199, -44.203, 0.674, -25.696, 11.526, -21.359, -38.259, 35.423, 112.634, 40.347, 44.522, -51.104, -49.152, 42.484, 66.355, -34.237, 91.088, 21.634, -54.127, -94.514, -24.88, 108.639, -59.463, -57.292, -11.901, 63.788, 97.003, -33.328, 46.895, 77.757, 39.134, -111.784, -62.769, 54.227, -19.469, -18.397, 3.246, -59.022, 34.878, -63.495, -40.069, -9.402, 81.451, 9.036, -108.271, -5.61 };
		double *ty = new double[80]{ 14.572, 13.641, 3.318, 8.299, 15.781, 7.412, 6.973, 24.587, -10.463, -12.12, 30.125, 11.12, 0.689, 23.322, 33.213, 33.602, 22.22, -5.586, 36.211, 17.51, 2.916, -11.824, -1.476, 37.306, -29.022, -14.361, 23.658, 6.199, -1.547, -4.175, 34.579, 9.631, 19.951, 36.1, 33.673, 0.508, -17.353, 35.608, -27.238, 18.386, 25.757, 34.426, 28.019, -11.214, 4.723, -0.267, 18.205, -28.276, -23.394, 12.726, 20.516, -15.007, -16.721, -8.938, -23.276, 12.541, 32.117, -17.984, -9.194, 39.417, -0.809, -4.225, 38.698, 3.593, 10.399, 9.894, 1.075, -20.057, -19.103, -8.687, 28.758, 20.685, -11.121, 31.731, -20.427, 29.832, -13.495, -9.576, 4.481, -24.733 };
		double *tz = new double[80]{ 57.188, 85.58, 107.826, 50.997, 25.308, 32.217, 77.181, 16.656, 15.795, 43.154, 16.288, 26.974, 16.074, 22.083, 24.913, 21.468, 24.815, 12.305, 11.461, 9.725, 55.144, 62.89, 23.62, 11.175, 9.596, 41.528, 8.805, 16.682, 8.593, 10.401, 33.221, 10.55, 21.86, 40.255, 35.92, 9.209, 26.516, 39.175, 11.086, 9.44, 10.832, 9.618, 11.693, 22.953, 31.34, 39.437, 29.985, 9.873, 9.474, 22.652, 10.584, 23.6, 11.377, 29.797, 11.809, 10.799, 19.415, 30.432, 8.73, 10.911, 10.362, 28.543, 16.865, 8.782, 8.551, 12.404, 9.231, 15.883, 26.228, 42.507, 47.761, 46.331, 8.626, 60.048, 21.544, 12.152, 10.376, 8.098, 29.072, 11.727 };
		sphNogaR = new double[80]{ 25, 25, 25, 25, 25, 25, 25, 16, 16, 16, 16, 16, 16, 16, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11 };

		for (int i = 0; i < 80; i++) sphNoga[i] = Vector3(tx[i], ty[i], tz[i]);

		delete[] tx; delete[] ty; delete[] tz;

		oreint = new int[framecount]; for (int i = 0; i < framecount; i++)oreint[i] = 0;

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
			for (int E = 0; E < 300; E++)
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
				for (int I = 0; I < 1000; I++)
				{
					unsigned char buff[pointsDisp][pointsDeep * 2 + 1];
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
					list[0] = Vector3(198.5, 105, devia[4]) - s[0];
					list[1] = Vector3(-198.5, 105, devia[5]) - s[0];
					list[2] = Vector3(-198.5, -105, devia[6]) - s[0];
					list[3] = Vector3(198.5, -105, devia[7]) - s[0];
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
						for (int x = 0; x < pointsDisp; x++)
						{
							_x = (x + 1) / (pointsDisp + 1.0);
							for (int y = 0; y < pointsDeep * 2 + 1; y++)
							{
								_y = (y - pointsDeep) / (pointsDeep * 30.0);
								P = A * (1 - _x)*(1 - _y) + B * (_x)*(1 - _y) + C * (_x)*(_y)+D * (1 - _x)*(_y);
								if (P.x < 0) { P.x = 0; }
								if (P.y < 0) { P.y = 0; }
								if (P.y > immaxX) { P.y = immaxX - 1; }
								if (P.x > immaxY) { P.x = immaxY - 1; }
								t = blurred[u].Get((int)P.x, (int)P.y);
								buff[x][y] = (t < 0) ? 0 : ((t > 255) ? 255 : t);
							}
						}
						Krit[i] = 0; Krite[i] = 0;
						for (int x = 0; x < pointsDisp; x++)
						{
							r = ((buff[x][4] + buff[x][5] + buff[x][6]) / 3.0 - (buff[x][0] + buff[x][1] + buff[x][2]) / 3.0);
							if (r > tr3)
							{
								Krit[i] = Krit[i] + w3;
								if (abs(i - pointsDisp / 2) >= (pointsDisp*0.25))
									Krite[i] = Krite[i] + w3;
							}
							r = ((buff[x][4] + buff[x][5]) / 2.0 - (buff[x][1] + buff[x][2]) / 2.0);
							if (r > tr2)
							{
								Krit[i] = Krit[i] + w2;
								if (abs(i - pointsDisp / 2) >= (pointsDisp*0.25))
									Krite[i] = Krite[i] + w2;
							}
							r = ((buff[x][4]) - (buff[x][2]));
							if (r > tr)
							{
								Krit[i] = Krit[i] + w1;
								if (abs(i - pointsDisp / 2) >= (pointsDisp*0.25))
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
			focuss.push_back(bbf);
		}
	}

	void UpdateOreint()
	{
		for (int u = 0; u < framecount; u++)
		{
			double r, t;
			Vector3 A = lists[u][0];
			Vector3 B = lists[u][1];
			Vector3 C = lists[u][2];
			Vector3 D = lists[u][3];

			if (u != 4)
			{
				Vector3 *proectednoga = new Vector3[80];
				for (int i = 0; i < 80; i++) proectednoga[i] = sphNoga[i];
				double krbb = -1;
				int borr, blor;
				for (int E = 0; E < 20; E++)
				{
					int orr = (E % 2 == 0) ? 1 : -1, lor = ((E % 4) / 2) ? 1 : -1;
					double G11 = -40, G12 = 40, G21 = -30, G22 = 30, G31 = 0.7, G32 = 1.2, G41 = -15 / 180 * PI, G42 = 15 / 180 * PI;
					double CFx, CFy, CFz, CFf, CFxb = 0, CFyb = 0, CFzb = 1, CFfb = 0;
					double krb = -1;
					for (int I = 0; I < 50; I++)
					{
						CFx = G11 + random * (G12 - G11);
						CFy = G21 + random * (G22 - G21);
						CFz = G31 + random * (G32 - G31);
						CFf = G41 + random * (G42 - G41);
						if (I % 3 == 2)
						{
							r = 0.93;
							if (CFxb < (G11 + G12) / 2.0) G12 = G11 + (G12 - G11)*r; else G11 = G12 - (G12 - G11)*r;
							if (CFyb < (G21 + G22) / 2.0) G22 = G21 + (G22 - G21)*r; else G21 = G22 - (G22 - G21)*r;
							if (CFzb < (G31 + G32) / 2.0) G32 = G31 + (G32 - G31)*r; else G31 = G32 - (G32 - G31)*r;
							if (CFfb < (G41 + G42) / 2.0) G42 = G41 + (G42 - G41)*r; else G41 = G42 - (G42 - G41)*r;
						}
						Vector3 P;
						Quaternion rot(0, 0, 1, CFf);
						for (int i = 0; i < 80; i++)
						{
							proectednoga[i] = sphNoga[i]; proectednoga[i].x *= CFz * orr; proectednoga[i].y *= CFz * orr * lor; proectednoga[i].z *= CFz;
							proectednoga[i] = rot.operator&(proectednoga[i]);
							P = (proectednoga[i] - cams[u][0]); t = P.x * P.x + P.y * P.y + P.z * P.z;
							proectednoga[i].x += CFx; proectednoga[i].y += CFy;
							ToCam(proectednoga[i], u);
							proectednoga[i].z = t;
						}
						if ((proectednoga[52].y > proectednoga[31].y) && ((abs(proectednoga[52].y - proectednoga[31].y)) > 3 * abs(proectednoga[52].x - proectednoga[31].x))) continue;

						double _x, _y;
						uchar buff[30][30][4];

						double avg = 0;

						for (int x = 0; x < 30; x++)
						{
							_x = (x + 1) / (30 + 1.0);
							for (int y = 0; y < 30; y++)
							{
								_y = (y + 1) / (30 + 1.0);
								P = A * (1 - _x)*(1 - _y) + B * (_x)*(1 - _y) + C * (_x)* (_y)+D * (1 - _x)*(_y);
								if (P.x < 0) { P.x = 0; }
								if (P.y < 0) { P.y = 0; }
								if (P.x > immaxX) { P.y = immaxX - 1; }
								if (P.y > immaxY) { P.x = immaxY - 1; }
								t = blurred[u].Get((int)P.x, (int)P.y);
								buff[x][y][2] = P.x;
								buff[x][y][3] = P.y;
								buff[x][y][0] = (t < 0) ? 0 : ((t > 255) ? 255 : t);
								avg += t;
							}
						}
						avg /= (double)(30 * 30);
						double kr = 0;
						for (int x = 0; x < 30; x++)
						{
							for (int y = 0; y < 30; y++)
							{
								if (buff[x][y][1] > avg) buff[x][y][1] = 255;
								else buff[x][y][1] = 0;
								bool isin = false;
								for (int i = 0; i < 80 && !isin; i++)
								{
									t = sphNogaR[i] * 900;
									t = t * t / (proectednoga[i].z);
									P = Vector3(proectednoga[i].x - buff[x][y][2], proectednoga[i].y - buff[x][y][3], 0);
									if (t > P.x * P.x + P.y * P.y)
										isin = true;
								}
								if (isin)
								{
									buff[x][y][1] = 255;
								}
								else
								{
									buff[x][y][1] = 0;
								}
								kr += (buff[x][y][1] == buff[x][y][0]) ? 1 : 0;
							}
						}

						if (kr > krb)
						{
							krb = kr;
							CFxb = CFx; CFyb = CFy; CFzb = CFz; CFfb = CFf;
						}
					}
					if (krb > krbb)
					{
						krbb = krb;
						blor = lor; borr = orr;
					}
				}

				oreint[u] = borr;

				delete[] proectednoga;
			}
			else
			{
				r = (A.y + D.y) - (B.y + C.y);
				if (r > 0) oreint[u] = 1; else oreint[u] = 2;
			}

			if (oreint[u] != 1)
			{
				for (int i = 0; i < 4; i++) { cams[u][i].x = -cams[u][i].x; cams[u][i].y = -cams[u][i].y; }
			}
		}
	}

	void GetBestedBorder()
	{
		double tr = 17, tr2 = tr * 1, tr3 = tr * 1, w1 = 1, w2 = 0.7, w3 = 0.2, r;
		double Krit[5], KritB[5], KritBB[5];
		double Krite[4], KriteB[4], KriteBB[4];
		double k1, k2, mk1, mk2, dk1, dk2, k1b, k1bb, k2b, k2bb;
		for (int i = 0; i < 4; i++) KritBB[i] = -1e20;
		for (int i = 0; i < 4; i++) KriteBB[i] = -1e20;
		Vector3 P1, P2, P, A, B, C, D;
		int t;

		double _x, _y, disp[7]{ -0.71 * 22 * 1.2, -0.71 * 12 * 1.2, -0.71, 0,  0.71 ,0.71 * 12 * 1.2 ,0.71 * 22 * 1.2 };

		for (int u = 0; u < framecount; u++)
		{
			for (int trys = 0; trys < 2; trys++) for (int i = 0; i < 4; i++)
			{
				A = lists[u][i];
				B = lists[u][(i + 1 > 3) ? i + 1 - 4 : i];
				C = lists[u][(i + 2 > 3) ? i + 2 - 4 : i];
				D = lists[u][(i + 3 > 3) ? i + 3 - 4 : i];
				double r1 = (A - D).GetLenght();
				double r2 = (B - C).GetLenght();
				for (int E = 0; E < 5; E++)
				{
					P1 = A;
					P2 = B;
					mk1 = 1; mk2 = 2;
					dk1 = 0.5; dk2 = 0.5;
					k1 = 1; k2 = 2;
					for (int i = 0; i < 4; i++) KritB[i] = -1e20;
					for (int i = 0; i < 4; i++) KriteB[i] = -1e20;
					for (int I = 0; I < 400; I++)
					{
						double val = (1 - 0.5 * I / (200.0 + I));
						unsigned char buff[pointsDisp][7];
						k1 = mk1 + (2 - random) * dk1;
						k2 = mk2 + (2 - random) * dk2;
						P1 = D + (A - D) * k1;
						P2 = C + (B - C) * k2;
						for (int x = 0; x < pointsDisp; x++)
						{
							_x = (x + 1) / (pointsDisp + 1.0);
							r = (r1 * k1 * (1 - _x) + r2 * k2 * _x);
							for (int y = 0; y < 7; y++)
							{
								_y = disp[y] / r * ((abs(y - 3) > 1) ? val : 1);
								P = P1 * (1 - _x)*(1 - _y) + P2 * (_x)*(1 - _y) + C * (_x)* (_y)+D * (1 - _x)*(_y);
								if (P.x < 0) { P.x = 0; }
								if (P.y < 0) { P.y = 0; }
								if (P.x > immaxX) { P.y = immaxX - 1; }
								if (P.y > immaxY) { P.x = immaxY - 1; }
								t = blurred[u].Get((int)P.x, (int)P.y);
								buff[x][y] = (t < 0) ? 0 : ((t > 255) ? 255 : t);
							}
						}
						Krit[i] = 0; Krite[i] = 0;
						for (int x = 0; x < pointsDisp; x++)
						{
							r = ((buff[x][4] + buff[x][5] + buff[x][6]) / 3.0 - (buff[x][0] + buff[x][1] + buff[x][2]) / 3.0);
							if (r > tr3)
							{
								Krit[i] = Krit[i] + w3;
								if (abs(i - pointsDisp / 2) >= (pointsDisp*0.25))
									Krite[i] = Krite[i] + w3;
							}
							r = ((buff[x][4] + buff[x][5]) / 2.0 - (buff[x][1] + buff[x][2]) / 2.0);
							if (r > tr2)
							{
								Krit[i] = Krit[i] + w2;
								if (abs(i - pointsDisp / 2) >= (pointsDisp*0.25))
									Krite[i] = Krite[i] + w2;
							}
							r = ((buff[x][4]) - (buff[x][2]));
							if (r > tr)
							{
								Krit[i] = Krit[i] + w1;
								if (abs(i - pointsDisp / 2) >= (pointsDisp*0.25))
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
				A = D + (A - D) * k1bb;
				B = C + (B - C) * k2bb;
				lists[u][i] = A;
				lists[u][(i + 1 > 3) ? i + 1 - 4 : i] = B;
				lists[u][(i + 2 > 3) ? i + 2 - 4 : i] = C;
				lists[u][(i + 3 > 3) ? i + 3 - 4 : i] = D;
			}

			GetCamPos(u);
		}
	}

	void GetBorderDisp_new()
	{
		double Krit, KritB, KritBB;
		double Krite, KriteB, KriteBB;
		double k, h, x0, a, kb, hb, x0b, ab, kbb, hbb, x0bb, abb, km, hm, x0m, am, kd, hd, x0d, ad;
		Vector3 P1, P2, P, A, B, C, D;
		int t;
		double _x, _y, r;
		for (int u = 0; u < framecount; u++)
		{
			while (listsborder.size() <= u)
			{
				double*** t = new double**[4]; for (int i = 0; i < 4; i++) { t[i] = new double*[20]; for (int j = 0; j < 20; j++) t[i][j] = new double[10]; }
				listsborder.push_back(t);
			}
			unsigned char buff[20][40];
			for (int i = 0; i < 4; i++)
			{
				A = lists[u][i];
				B = lists[u][(i + 1 > 3) ? i + 1 - 4 : i];
				C = lists[u][(i + 2 > 3) ? i + 2 - 4 : i];
				D = lists[u][(i + 3 > 3) ? i + 3 - 4 : i];
				double r1 = (A - D).GetLenght();
				double r2 = (B - C).GetLenght();
				for (int x = 0; x < 20; x++)
				{
					_x = (x + 1) / (20 + 1.0);
					r = (r1 * (1 - _x) + r2 * _x);
					for (int y = -20; y <= 20; y++)
					{
						_y = y / r;
						P = A * (1 - _x)*(1 - _y) + B * (_x)*(1 - _y) + C * (_x)* (_y)+D * (1 - _x)*(_y);
						if (P.x < 0) { P.x = 0; }
						if (P.y < 0) { P.y = 0; }
						if (P.x > immaxX) { P.y = immaxX - 1; }
						if (P.y > immaxY) { P.x = immaxY - 1; }
						t = blurred[u].Get((int)P.x, (int)P.y);
						buff[x][y + 20] = (t < 0) ? 0 : ((t > 255) ? 255 : t);
					}
					KritBB = 1e20;
					for (int E = 0; E < 10; E++)
					{
						km = 0; kd = 50; am = 1.6; ad = 1.55;
						x0m = 0; x0d = 20; hm = 127; hd = 126;
						KritB = 1e20;
						for (int I = 0; I < 400; I++)
						{
							k = km + (2 - random) * kd;
							x0 = x0m + (2 - random) * x0d;
							a = am + (2 - random) * ad;
							h = hm + (2 - random) * hd;

							double Kr = 0;

							for (int y = -20; y <= 20; y++)
							{
								t = abs(buff[x][y + 20] - h - k / (1 + exp(-(y - x0) / a)));
								Kr += t * t;
							}

							double disp = 0.02;
							if (Kr < KritB)
							{
								KritB = Kr;
								kb = k; hb = h; ab = a; x0b = x0;
							}
							if (I % 2 == 1)
							{

								km += ((kb > km) ? kd * disp : -kd * disp);
								kd *= (1 - disp);
								hm += ((hb > hm) ? hd * disp : -hd * disp);
								hd *= (1 - disp);
								am += ((ab > am) ? ad * disp : -ad * disp);
								ad *= (1 - disp);
								x0m += ((x0b > x0m) ? x0d * disp : -x0d * disp);
								x0d *= (1 - disp);
							}
						}
						if (KritB < KritBB)
						{
							KritBB = KritB;
							kbb = kb; hbb = hb; abb = ab; x0bb = x0b;
						}
					}
					listsborder[u][i][x][6] = kbb;
					listsborder[u][i][x][7] = x0bb;
					listsborder[u][i][x][8] = abb;
					listsborder[u][i][x][9] = hbb;
					_y = x0bb / r;
					P = A * (1 - _x)*(1 - _y) + B * (_x)*(1 - _y) + C * (_x)* (_y)+D * (1 - _x)*(_y);
					listsborder[u][i][x][0] = P.x;
					listsborder[u][i][x][1] = P.y;
					listsborder[u][i][x][2] = 0;

					P = cams[u][0];
					P.x = listsborder[u][i][x][0] - P.x;
					P.y = listsborder[u][i][x][1] - P.y;
					P.z = 0 - P.z;
					P2 = A - B;
					P2 = P2 / Vector3(0, 0, 1);
					P2 = P2 / P2.x;
					P1 = P; P1.z = 0;
					P1 = P1 / Vector3(0, 0, 1);
					P1 = P1 / P1.x;
					listsborder[u][i][x][3] = (P1.y * listsborder[u][i][x][0] + listsborder[u][i][x][1] - P1.y * A.x - A.y) / (P1.y - P2.y);
					listsborder[u][i][x][4] = (P2.y * (A.x - listsborder[u][i][x][3]) + A.y);
					P = P / abs(P.x);
					listsborder[u][i][x][5] = (listsborder[u][i][x][3] - listsborder[u][i][x][0]) * P.z;
				}
				devia[i] = 0;
				for (int x = 1, y = 0; x < 20; x++)
				{
					P.x = listsborder[u][i][x][3] - listsborder[u][i][y][3]; P.y = listsborder[u][i][x][4] - listsborder[u][i][y][4]; P.z = listsborder[u][i][x][5] - listsborder[u][i][y][5];
					if (P.z < 6 * sqrt(x - y))
					{
						devia[i] += P.GetLenght();
						y++;
					}
				}
			}
			A = lists[u][0];
			B = lists[u][1];
			C = lists[u][2];
			D = lists[u][3];
			double iAB = (A - B).GetLenght(), iBC = (C - B).GetLenght(), iCD = (C - D).GetLenght(), iDA = (A - D).GetLenght(), AB, BC, CD, DA;
			for (int temp = 0; temp < 100; temp++)
			{

			}
		}
	}

	void GetBorderDisp()
	{
		double Krite, KriteB, KriteBB;
		Vector3 P1, P2, P, A, B, C, D;
		int t;
		double _x, _y, r, q, fi;
		for (int u = 0; u < framecount; u++)
		{
			while (listsborder.size() <= u)
			{
				double*** t = new double**[4]; for (int i = 0; i < 4; i++) { t[i] = new double*[20]; for (int j = 0; j < 20; j++) t[i][j] = new double[10]; }
				listsborder.push_back(t);
			}
			unsigned char buff[20][61][3];
			for (int i = 0; i < 4; i++)
			{
				A = lists[u][i];
				B = lists[u][(i + 1 > 3) ? i + 1 - 4 : i];
				C = lists[u][(i + 2 > 3) ? i + 2 - 4 : i];
				D = lists[u][(i + 3 > 3) ? i + 3 - 4 : i];
				double r1 = (A - D).GetLenght();
				double r2 = (B - C).GetLenght();
				for (int x = 0; x < 20; x++)
				{
					_x = (x + 1) / (20 + 1.0);
					r = (r1 * (1 - _x) + r2 * _x);
					for (int y = -30; y <= 30; y++)
					{
						_y = 2 * y / r;
						P = A * (1 - _x)*(1 - _y) + B * (_x)*(1 - _y) + C * (_x)* (_y)+D * (1 - _x)*(_y);
						if (P.x < 0) { P.x = 0; }
						if (P.y < 0) { P.y = 0; }
						if (P.y > immaxY - 1) { P.y = immaxY - 1; }
						if (P.x > immaxX - 1) { P.x = immaxX - 1; }
						t = blurred[u].Get((int)P.x, (int)P.y);
						buff[x][y + 30][0] = (t < 0) ? 0 : ((t > 255) ? 255 : t);
						buff[x][y + 30][1] = (int)P.x;
						buff[x][y + 30][2] = (int)P.y;
					}
				}
				double krb = 0, kr;
				for (int x = 0; x < 20; x++)
				{
					int best = 10000;
					krb = 0;
					for (int y = -25; y <= 25; y++)
					{
						kr = 0;
						for (int k = y + 1; k < y + 6; k++) kr += buff[x][k + 30][0];
						for (int k = y - 5; k < y; k++)		kr -= buff[x][k + 30][0];
						if (kr > krb)
						{
							krb = kr; best = y;
						}
					}
					listsborder[u][i][x][0] = best;
					listsborder[u][i][x][1] = buff[x][best][1];
					listsborder[u][i][x][2] = buff[x][best][2];
					listsborder[u][i][x][9] = krb;
					//_x = buff[x][best][1] - cams[u][0].x;
					//_y = buff[x][best][1] - cams[u][0].y;
					//r = sqrt(_x * _x + _y * _y);
					//fi = atan(cams[u][0].z / (r + 1e-20));
					_x = buff[x][best][1] - buff[x][30][1];
					_y = buff[x][best][1] - buff[x][30][2];
					q = sqrt(_x * _x + _y * _y);
					//if (best < 0) q = -q;
					if (best > 0) q = -q;
					//listsborder[u][i][x][3] = q / sin(PI / 2 - fi);
					listsborder[u][i][x][3] = q;
				}
				double x0, x1;
				krb = 0;
				for (int _x0 = -5; _x0 < 6; _x0++)for (int _x1 = -5; _x1 < 6; _x1++)
				{
					kr = 0;
					for (int x = 1; x < 19; x++)
					{
						r = _x0 * 2.0 + (_x1 - _x0) * i / 10.0;
						kr += sqrt(sqrt(abs(r - listsborder[u][i][x][3])));
					}
					if (kr < krb)
					{
						x0 = _x0; x1 = _x1;
						krb = kr;
					}
				}
				for (int x = 1; x < 19; x++)
				{
					r = x0 * 2 + (x1 - x0) * i / 10.0;
					if (abs(r - listsborder[u][i][x][3]) > 6)
					{
						listsborder[u][i][x][3] = 1e10;
					}
				}

			}
			A = lists[u][0];
			B = lists[u][1];
			C = lists[u][2];
			D = lists[u][3];

			r = 0;
			q = 0;
			for (int x = 1; x < 19; x++) if (listsborder[u][0][x][3] < 1e9)
			{
				q = q + 1;
				r = r + abs(listsborder[u][0][x][3]);
			}
			devia[0] = r / (q + 1e-20);

			r = 0;
			q = 0;
			for (int x = 1; x < 19; x++) if (listsborder[u][1][x][3] < 1e9)
			{
				q = q + 1;
				r = r + abs(listsborder[u][1][x][3]);
			}
			devia[1] = r / (q + 1e-20);

			r = 0;
			q = 0;
			for (int x = 11; x < 19; x++) if (listsborder[u][3][x][3] < 1e9)
			{
				q = q + 1;
				r = r + abs(listsborder[u][3][x][3]);
			}
			devia[4] = r / (q + 1e-20);
			devia[4] = devia[4] * 0.35 + devia[0] * 0.65;
			devia[4] = devia[4] * 7.50777 - 0.31157;
			devia[4] = devia[4] * 2 - 6; devia[4] = (devia[4] < 0) ? 0 : ((devia[4] > 12) ? 12 : devia[4]);
			devia[5] = devia[4] / 2;

			r = 0;
			q = 0;
			for (int x = 1; x < 11; x++) if (listsborder[u][3][x][3] < 1e9)
			{
				q = q + 1;
				r = r + abs(listsborder[u][3][x][3]);
			}
			devia[7] = r / (q + 1e-20);
			devia[7] = devia[7] * 0.35 + devia[1] * 0.65;
			devia[7] = devia[7] * 7.50777 - 0.31157;
			devia[7] = devia[7] * 2 - 6; devia[7] = (devia[7] < 0) ? 0 : ((devia[7] > 12) ? 12 : devia[7]);
			devia[6] = devia[7] / 2;

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
		while (inworld.size() <= u)
		{
			Vector3* temp = new Vector3[4];
			inworld.push_back(temp);
		}
		inworld[u][0] = WA; inworld[u][1] = WB; inworld[u][2] = WC; inworld[u][3] = WD;
		focuss[u] = lamda;
		cams[u][0] = O; cams[u][1] = X; cams[u][2] = Y; cams[u][3] = Z;
	}

	void GetNewCamPos()
	{
		for (int u = 0; u < framecount; u++)
		{
			int t = 0;
			Vector3 *cam = new Vector3[5];
			Vector3 *list = new Vector3[4];
			Vector3 d[2];
			Vector3 s[2], b[2], bb[2], m[2]; // select, best, bestbest, median
			double f, bf, bbf, a, ba, bba, df, mf, da, ma, r; // focus, angel

			Vector3 se[2]{ Vector3(120, 120, 120), Vector3(60,60,0) }, P;
			double fe = 20, ae = PI * 0.5 * 0.6;

			double spu[7];

			double Krit[5], KritB[5], KritBB[5];
			for (int i = 0; i < 4; i++) KritBB[i] = 1e20;
			for (int E = 0; E < 300; E++)
			{
				if (E < 160)
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
				}
				else
				{
					r = 0.7;
					for (int i = 0; i < 2; i++)
					{
						if (E % 20 == 0) se[i] = se[i] * r;
						m[i] = bb[i];
						d[i] = se[i];
					}
					if (E % 20 == 0) ae = ae * r;
					if (E % 20 == 0) fe = fe * r;
					ma = bba;
					mf = bbf;
					df = fe;
					da = ae;
				}
				for (int i = 0; i < 4; i++) KritB[i] = 1e20;
				for (int I = 0; I < 1000; I++)
				{
					for (int i = 0; i < 2; i++)
					{
						s[i].x = (random > 0.7) ? b[i].x : m[i].x + (2 - random) * d[i].x;
						s[i].y = (random > 0.7) ? b[i].y : m[i].y + (2 - random) * d[i].y;
						s[i].z = (random > 0.7) ? b[i].z : m[i].z + (2 - random) * d[i].z;
					}
					a = (random > 0.7) ? ba : ma + (2 - random) * da;
					f = (random > 0.7) ? bf : mf + (2 - random) * df;
					double koof = 0, tf;
					for (int W = 0; W < 280; W++)
					{
						koof = 0.333 * (280 - W) / 280.0;
						for (int i = 0; i < 7; i++)spu[0] = 0;
						switch (W % 14)
						{
						case  0: spu[0] = se[0].x * koof; break;
						case  1: spu[0] = -se[0].x * koof; break;
						case  2: spu[1] = se[0].y * koof; break;
						case  3: spu[1] = -se[0].y * koof; break;
						case  4: spu[2] = se[0].z * koof; break;
						case  5: spu[2] = -se[0].z * koof; break;
						case  6: spu[3] = se[1].x * koof; break;
						case  7: spu[3] = -se[1].x * koof; break;
						case  8: spu[4] = se[1].y * koof; break;
						case  9: spu[4] = -se[1].y * koof; break;
						case 10: spu[5] = ae * koof; break;
						case 11: spu[5] = -ae * koof; break;
						case 12: spu[6] = fe * koof; break;
						case 13: spu[6] = -fe * koof; break;
						default: break;
						}
						cam[0] = s[0]; cam[0].x += spu[0]; cam[0].y += spu[1]; cam[0].z += spu[2];
						cam[3] = s[1]; cam[3].x += spu[3]; cam[3].y += spu[4];
						cam[3] = cam[3] - cam[0]; cam[3].SetLenght(1);
						cam[2] = cam[3] / Vector3(0, 0, -1);
						cam[2].z = a + spu[5]; cam[2].SetLenght(1);
						cam[1] = cam[3] / cam[2];
						list[0] = Vector3(198.5, 105, devia[4]) - s[0];
						list[1] = Vector3(-198.5, 105, devia[5]) - s[0];
						list[2] = Vector3(-198.5, -105, devia[6]) - s[0];
						list[3] = Vector3(198.5, -105, devia[7]) - s[0];
					}
					tf = f + spu[6];
					for (int i = 0; i < 4; i++)
					{
						list[i] = Vector3(list[i] * cam[1], list[i] * cam[2], list[i] * cam[3]); list[i] = list[i] / list[i].z * tf / K;
						list[i].x -= immaxX / 2; list[i].y -= immaxY / 2;
					}
					double Kr = 0;
					for (int i = 0; i < 4; i++)
					{
						P = list[i] - lists[u][i];
						Kr = P.x*P.x + P.y*P.y;
					}

					if (Kr < KritB[4])
					{
						KritB[4] = Kr;
						for (int i = 0; i < 4; i++) KritB[i] = Krit[i];
						b[0] = s[0]; b[1] = s[1];
						bf = f; ba = a;
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
			for (int i = 0; i < 4; i++)
			{
				list[i] = Vector3(list[i] * cam[1], list[i] * cam[2], list[i] * cam[3]); list[i] = list[i] / list[i].z * f / K;
				list[i].x -= immaxX / 2; list[i].y -= immaxY / 2;
			}
			//lists.push_back(list);
			delete[] cams[u];
			cams[u] = cam;
			focuss[u] = bbf;
		}
	}

	void GetFoot()
	{
		Vector3 P, A, B, C, D;
		Vector2 Ko, Te, Li, V1, V2, X, Y;
		for (int u = 0; u < framecount; u++)
		{
			P = Vector3(-55, 0, 300);
			P = Vector3(P * cams[u][1], P * cams[u][2], P * cams[u][3]); P = P / P.z * focuss[u] / K;
			P.x -= immaxX / 2; P.y -= immaxY / 2;
			Ko = Vector2(P.x, P.y);
			P = Vector3(-75, 0, 0);
			P = Vector3(P * cams[u][1], P * cams[u][2], P * cams[u][3]); P = P / P.z * focuss[u] / K;
			P.x -= immaxX / 2; P.y -= immaxY / 2;
			Li = Vector2(P.x, P.y);
			Li = Li - Ko;
			PhisicPoint* points = new PhisicPoint[400];
			for (int i = 0; i < 4; i++)
			{
				A = lists[u][i];
				B = lists[u][(i + 1 > 3) ? i + 1 - 4 : i];
				C = lists[u][(i + 2 > 3) ? i + 2 - 4 : i];
				D = lists[u][(i + 3 > 3) ? i + 3 - 4 : i];
				for (int k = 0; k < 50; k++)
					points[i * 50 + k].loc = Vector2(A.x * (1 - k / 50.0) + B.x * (k / 50.0), A.y * (1 - k / 50.0) + B.y * (k / 50.0));
			}
			Te = Ko - points[0].loc;
			double min = Te * Te, t, r;
			int freez = 0;
			for (int i = 1; i < 200; i++) { Te = Ko - points[i].loc; t = Te * Te; if (t < min) { min = t; freez = i; } }
			while (Ko.x < 0 || Ko.y < 0 || Ko.x >= immaxX || Ko.y >= immaxY) // Проверка на вне кадра
			{
				if (Ko.x < 0)
				{
					Li = Li / Li.x;
					Ko = Ko + Li * Ko.x;
				}
				if (Ko.y < 0)
				{
					Li = Li / Li.y;
					Ko = Ko + Li * Ko.y;
				}
				if (Ko.x >= immaxX)
				{
					Li = Li / Li.x;
					Ko = Ko + Li * (immaxX - Ko.x - 1);
				}
				if (Ko.y >= immaxY)
				{
					Li = Li / Li.y;
					Ko = Ko + Li * (immaxY - Ko.y - 1);
				}
			}
			points[freez].loc = Ko;
			double c = 150, d = 30, dt = 0.01, g = 50, rmids = ((A - B).GetLenght() + (B - C).GetLenght() + (C - D).GetLenght() + (D - A).GetLenght()) / 150.0;
			double ots[4]{ 0,5,13,25 }, val, w1 = 0.6, w2 = 0.25, w3 = 0.15;
			unsigned char buff[7];
			for (int time = 0; time < 10000; time++)
			{
				val = (1 - 0.5 * time / (1250.0 + time));
				double  rmid = rmids * val;
				for (int i = 1; i < 200; i++) { points[i].dv.x = 0; points[i].dv.y = 0; }
				for (int i0 = 200 - 1, i1 = 0, i2 = 1; i1 < 200; i1++, i2 = (i2 == 200 - 1) ? 0 : i2 + 1, i0 = (i0 == 200 - 1) ? 0 : i0 + 1)
				{
					V1 = points[i1].loc - points[i2].loc;
					r = V1.GetLenght();
					t = -c * (r - rmid) / r;
					V2 = V1 * t - (points[i1].v - points[i2].v) * d;
					points[i1].dv = points[i1].dv + V2;
					points[i2].dv = points[i2].dv - V2;

					V2 = (points[i0].loc + points[i2].loc) * 0.5;
					Y = points[i2].loc - points[i0].loc; Y.SetLenght(1);
					X = (!Y) * -1;
					for (int i = -3; i < 4; i++)
					{
						V1 = points[i1].loc + X * ots[abs(i)] * val;
						t = imgs[u].Get((int)V1.x, (int)V1.y);
						buff[i + 3] = (t < 0) ? 0 : ((t > 255) ? 255 : t);
					}
					double k = 1;
					if (((points[i2].loc.x - points[i0].loc.x)*(points[i1].loc.y - points[i0].loc.y)
						- (points[i2].loc.y - points[i0].loc.y)*(points[i1].loc.x - points[i0].loc.x)) > 0) k = -1; else k = 1;
					double fi = g * (w1 * (buff[2] - 2 * buff[3] + buff[4]) + w2 * (buff[1] - 2 * buff[3] + buff[5]) + w3 * (buff[0] - 2 * buff[3] + buff[6]))*k;

					points[i1].dv = points[i1].dv + X * fi + (points[i0].v - points[i1].v * 2 + points[i2].v) * 20;
				}
				double ddt = dt * dt * 0.5;
				for (int i1 = 0; i1 < 200; i1++) if (i1 != freez)
				{
					points[i1].loc = points[i1].loc + points[i1].v * dt + points[i1].dv * ddt;
					points[i1].v = points[i1].v + points[i1].dv * dt;
				}
			}
			freez *= 2;
			PhisicPoint T = points[0];
			for (int i1 = 200 - 1; i1 >= 0; i1--)
			{
				points[i1 * 2] = points[i1];
				points[i1 * 2 + 1].loc = (T.loc + points[i1 * 2].loc) * 0.5;
				points[i1 * 2].v.x = 0; points[i1 * 2].v.y = 0; points[i1 * 2].dv.x = 0; points[i1 * 2].dv.x = 0;
				points[i1 * 2 + 1].v.x = 0; points[i1 * 2 + 1].v.y = 0; points[i1 * 2 + 1].dv.x = 0; points[i1 * 2 + 1].dv.x = 0;
			}
			double rad[61][3];
			for (int i0 = 400 - 1, i1 = 0, i2 = 1; i1 < 400; i1++, i2 = (i2 == 400 - 1) ? 0 : i2 + 1, i0 = (i0 == 400 - 1) ? 0 : i0 + 1)
			{
				bool isina4 = true;
				for (int i = 0; i < 4; i++)
				{
					A = lists[u][i];
					B = lists[u][(i + 1 > 3) ? i + 1 - 4 : i];
					C = lists[u][(i + 2 > 3) ? i + 2 - 4 : i];
					double dx = B.y - A.y;
					double dy = A.x - B.x;
					double Q = A.x*dx + A.y*dy;
					if ((points[i1].loc.x*dx + points[i1].loc.y*dy + Q) * (C.x*dx + C.y*dy + Q) < 0) isina4 = false;
				}
				if (!isina4 || i1 == freez) continue;
				Y = points[i2].loc - points[i0].loc; Y.SetLenght(1);
				X = !Y; 
				for (int o, int i = -30; i <= 30; i++)
				{
					V1 = points[i1].loc + X * (i * 0.5);
					if (V1.x < 1) { V1.x = 1; }
					if (V1.y < 1) { V1.y = 1; }
					if (V1.y > immaxY - 2) { V1.y = immaxY - 2; }
					if (V1.x > immaxX - 2) { V1.x = immaxX - 2; }
					t = 0;
					for (int x = -1; x < 2; x++)for (int y = -1; y < 2; y++)
					{
						o = abs(x) + abs(y);
						t += blurred[u].Get((int)V1.x, (int)V1.y)*((y == 0) ? 0.2 : (y == 1) ? 0.12 : 0.08);
					}
					rad[i + 30][0] = (t < 0) ? 0 : ((t > 255) ? 255 : t);
					rad[i + 30][1] = (int)P.x;
					rad[i + 30][2] = (int)P.y;
				}
				double Krit, KritB, KritBB;
				double k, h, x0, a, kb, hb, x0b, ab, kbb, hbb, x0bb, abb, km, hm, x0m, am, kd, hd, x0d, ad;
				KritBB = 1e20;
				for (int E = 0; E < 10; E++)
				{
					km = 0; kd = 50; am = 1.6; ad = 1.55;
					x0m = 0; x0d = 20; hm = 127; hd = 126;
					KritB = 1e20;
					for (int I = 0; I < 400; I++)
					{
						k = km + (2 - random) * kd;
						x0 = x0m + (2 - random) * x0d;
						a = am + (2 - random) * ad;
						h = hm + (2 - random) * hd;
						double r;
						double Kr = 0;
						for (int i = -30; i <= 30; i++)
						{

							if (k * (i - x0) > 0)
								r = 0.75 + 0.25 * exp(-(i - x0)*(i - x0) * 0.25);
							else
								r = 0.22 + 0.78 * exp(-(i - x0)*(i - x0) * 0.25);

							t = abs(rad[i + 30][0] - h - k / (1 + exp(-(i - x0) / a)));
							Kr += r * t * t;
						}

						double disp = 0.02;
						if (Kr < KritB)
						{
							KritB = Kr;
							kb = k; hb = h; ab = a; x0b = x0;
						}
					}
					if (KritB < KritBB)
					{
						KritBB = KritB;
						kbb = kb; hbb = hb; abb = ab; x0bb = x0b;
					}
				}
				points[i1].loc.x = rad[(int)x0bb][1];
				points[i1].loc.y = rad[(int)x0bb][2];
			}
			while (counter.size() < u - 1)
			{
				counter.push_back(Img(immaxX, immaxY));
			}
			for (int x = 0; x < immaxX; x++) for (int y = 0; y < immaxY; y++)
			{
				counter[u].Set(x, y, 0);
			}
			P = Vector3(-75, 0, 0);
			P = Vector3(P * cams[u][1], P * cams[u][2], P * cams[u][3]); P = P / P.z * focuss[u] / K;
			P.x -= immaxX / 2; P.y -= immaxY / 2;
			Li = Vector2(P.x, P.y);
			Li = (Li + Ko) * 0.5;
			for (int i1 = 0, i2 = 1; i1 < 400; i1++, i2 = (i2 == 400 - 1) ? 0 : i2 + 1)
			{
				V1 = points[i2].loc - points[i1].loc;
				r = abs(V1.x);
				V1 = V1 / r;
				V2 = points[i1].loc;
				r = (int)r + 1;
				for (int i = 0; i <= r; i++)
				{
					counter[u].Set((int)V2.x, (int)V2.y, 255);
				}
			}
			counter[u].Set((int)Li.x, (int)Li.y, 100);
			int count = 1;
			while(count != 0)
			{
				for (int n1 = 0; n1 < 2; n1++)for (int n2 = 0; n2 < 2; n2++)if (count != 0)
				{
					count = 0;
					for (int k1 = 1,x = (n1 == 0) ? k1 : immaxX - 1 - k1; k1 < immaxX - 1; k1++, x = (n1 == 0) ? k1 : immaxX - 1 - k1) for (int k2 = 1, y = (n2 == 0) ? k2 : immaxY - 1 - k2; k2 < immaxY - 1; k2++, y = (n2 == 0) ? k2 : immaxY - 1 - k2)
					{
						if (counter[u].Get(x, y) == 0)
						{
							if (counter[u].Get(x + 1, y) == 100) counter[u].Set(x, y, 100);
							else if (counter[u].Get(x - 1, y) == 100) counter[u].Set(x, y, 100);
							else if (counter[u].Get(x, y + 1) == 100) counter[u].Set(x, y, 100);
							else if (counter[u].Get(x, y - 1) == 100) counter[u].Set(x, y, 100);
							else if (counter[u].Get(x + 1, y) == 150) counter[u].Set(x, y, 100);
							else if (counter[u].Get(x - 1, y) == 150) counter[u].Set(x, y, 100);
							else if (counter[u].Get(x, y + 1) == 150) counter[u].Set(x, y, 100);
							else if (counter[u].Get(x, y - 1) == 150) counter[u].Set(x, y, 100);
							if (counter[u].Get(x, y) == 100)count++;
						}
						if (counter[u].Get(x, y) == 100)
						{
							count++;
							counter[u].Set(x, y, 150);
						}
						if (counter[u].Get(x, y) == 150)
						{
							count++;
							counter[u].Set(x, y, 255);
						}
					}
				}
			}
		}
	}

	bool isInCounter(Vector3 P, int u)
	{
		return counter[u].Get((int)P.x, (int)P.y) > 0;
	}

	void GetFirstVoxel()
	{
		double boost = 4, bsize = boost / VoxelS;
		int varcount = framecount + 1;
		unsigned char **variant = new uchar*[framecount + 1];
		for (int i = 0; i < framecount + 1; i++)
			variant[i] = new uchar[framecount];
		for (int i = 0; i < framecount; i++)variant[0][i] = 1;
		for (int i = 1; i < framecount + 1; i++) for (int j = 0; j < framecount; j++) variant[i][j] = (j == i - 1) ? 0 : 1;
		int NS = 35;
		double ix[35]{ -120.415, -118.223, -107.241, -92.491, -76.224, -59.869, -43.488, -27.181, -10.797, 5.474, 21.693, 37.937, 54.23, 70.581, 86.815, 102.924, 118.374, 126.633, 123.11, 114.115, 102.974, 89.941, 76.335, 61.995, 45.691, 29.353, 13.098, -3.187, -19.441, -35.626, -51.862, -68.153, -84.333, -99.562, -112.899 };
		double iy[35]{ 4.29, 20.134, 32.102, 38.86, 40.352, 40.602, 40.873, 42.078, 41.615, 43.627, 46.018, 48.231, 50.013, 50.06, 47.825, 46.354, 40.912, 27.898, 12.412, -1.136, -13.152, -22.505, -31.278, -38.64, -40.317, -40.481, -38.396, -36.517, -34.39, -31.79, -29.515, -27.687, -25.138, -19.178, -9.744 };
		Vector2 T[35][7];
		Vector3 P;
		double r, xA, yA, xB, yB, fi, q;
		for (int i = 0; i < NS; i++)
		{
			T[i][0].x = ix[i];
			T[i][0].y = iy[i];
			xA = ix[i]; yA = iy[i];
			xB = 1;
			yB = -(ix[i + 1] - ix[i - 1]) / (iy[i + 1] - iy[i - 1] + 1e-20);
			fi = sqrt(xB * xB + yB * yB);
			q = (ix[i + 1] - ix[i - 1])*(yB)-(iy[i + 1] - iy[i - 1])*(xB);
			for (int j = 0, _j = -3; j < 7; j++, _j++)
			{

				if (abs(_j) == 1)  r = 2.5;
				if (abs(_j) == 2)  r = 5.5;
				if (abs(_j) == 3)  r = 8.5;
				if (_j != 0)
				{
					T[i][j].x = xA + xB / fi * r*_j*(-q / abs(_j*q));
					T[i][j].y = yA + yB / fi * r*_j*(-q / abs(_j*q));
				}
				else
				{
					T[i][j].x = xA;
					T[i][j].y = yA;
				}
			}
		}
		int best = 0;
		for (int x = 0; x < VoxelX; x++)for (int y = 0; y < VoxelY; y++)for (int z = 0; z < VoxelZ; z++) voxel.USGet(x, y, z) = 1;
		double *KritsVar = new double[varcount];
		int *oreinVar = new int[varcount];
		for (int u = 0; u < varcount; u++)			// Поиск лучшего
		{
			int z = 7;
			int size = 0;
			for (int k = 0; k < framecount; k++) if (variant[u][k] == 1)
			{
				for (int x = 0; x < VoxelX; x += bsize) for (int y = 0; y < VoxelY; y += bsize) if (voxel.USGet(x, y, u) == 1)
				{
					P.x = (x - VoxelX / 2.0) * VoxelS; P.y = (y - VoxelY / 2.0) * VoxelS; P.z = z;
					ToCam(P, k);
					if (!isInCounter(P, k)) for (int i = 0; i < bsize; i++)for (int j = 0; j < bsize; j++)
					{
						voxel.USGet(x + i, y + j, u) = 0;
					}
					else
					{
						size += boost * boost;
					}
				}
			}

			double dXbb, dYbb, dSbb, dYSbb, dFIbb;
			KritsVar[u] = -1e20;
			for (int E = 0; E < 60; E++)
			{
				double KritB = -1e20;
				double dX0 = 30, dX1 = 120, dY0 = 20, dY1 = 90, dS0 = 0.6, dS1 = 1.6, dYS0 = 0.6, dYS1 = 1.6, dFI0 = -PI / 6.0, dFI1 = PI / 6.0;
				double dX, dY, dS, dYS, dFI;
				double dXb, dYb, dSb, dYSb, dFIb;
				double tx, ty, t, si, co;
				int ore = 0;
				for (int I = 0; I < 300; I++)
				{
					dX = dX0 + random * (dX1 - dX0);
					dY = dY0 + random * (dY1 - dY0);
					dS = dS0 + random * (dS1 - dS0);
					dYS = dYS0 + random * (dYS1 - dYS0);
					dFI = dFI0 + random * (dFI1 - dFI0);
					if ((I + 1) % 10 == 0)
					{
						if (dXb * 2 < dX0 + dX1) dX1 = dX0 + 0.9 * (dX1 - dX0); else dX0 = dX0 + 0.9 * (dX1 - dX0);
						if (dYb * 2 < dY0 + dY1) dY1 = dY0 + 0.9 * (dY1 - dY0); else dY0 = dY0 + 0.9 * (dY1 - dY0);
						if (dSb * 2 < dS0 + dS1) dS1 = dS0 + 0.9 * (dS1 - dS0); else dS0 = dS0 + 0.9 * (dS1 - dS0);
						if (dYSb * 2 < dYS0 + dYS1) dYS1 = dYS0 + 0.9 * (dYS1 - dYS0); else dYS0 = dYS0 + 0.9 * (dYS1 - dYS0);
						if (dFIb * 2 < dFI0 + dFI1) dFI1 = dFI0 + 0.9 * (dFI1 - dFI0); else dFI0 = dFI0 + 0.9 * (dFI1 - dFI0);
					}

					si = sin(dFI);
					co = cos(dFI);

					double Kr = 0;

					for (int i = 0; i < 35; i++) for (int j = 0, _j = -3; j < 7; j++, _j++) if (_j != 0)
					{
						tx = T[i][j].x * dS;
						ty = T[i][j].y * dS * dYS * ((E < 30) ? 1 : -1);
						t = tx;
						tx = dX + t * co - ty * si;
						ty = dY + t * si + ty * co;
						Kr += voxel.USGet(u, (int)tx, (int)ty) * _j / (abs(_j));
					}
					t = 4600 * dS * dS * dYS; t = 1 - (2 * abs(t - size) / (t + size)); t = t * t*t*t;
					Kr *= t;
					if (Kr > KritB)
					{
						KritB = Kr;
						dXb = dX;
						dYb = dY;
						dSb = dS;
						dYSb = dYS;
						dFIb = dFI;
						ore = (E < 30) ? 2 : 1;
					}
				}
				if (KritB > KritsVar[u])
				{
					KritsVar[u] = KritB;
					dXbb = dXb;
					dYbb = dYb;
					dSbb = dSb;
					dYSbb = dYSb;
					dFIbb = dFIb;
					oreinVar[u] = ore;
				}
			}
		}

		for (int i = 0; i < varcount; i++) KritsVar[i] *= ((i > 1 && i < framecount + 1) ? 0.94 : ((i == 1) ? 1 : 0.88)) * ((variant[i][0] == 0) ? 0.96 : 1)* ((variant[i][framecount - 1] == 0) ? 0.96 : 1) * ((variant[i][framecount / 2 + 1] == 0) ? 0.94 : 1);

		best = 0;

		for (int i = 0; i < varcount; i++)if (KritsVar[best] < KritsVar[i]) best = i;

		for (int x = 0; x < VoxelX; x++)for (int y = 0; y < VoxelY; y++)for (int z = 0; z < VoxelZ; z++) voxel.USGet(x, y, z) = 1;

		for (int k = 0; k < framecount; k++) if (variant[best][k] == 1)
		{

			for (int x = 0; x < VoxelX; x++)for (int y = 0; y < VoxelY; y++)for (int z = 0; z < VoxelZ; z++) if (voxel.USGet(x, y, z) == 1)
			{
				P.x = (x - VoxelX / 2.0) * VoxelS; P.y = (y - VoxelY / 2.0) * VoxelS; P.z = (z - 5) * VoxelS;
				ToCam(P, k);
				if (!isInCounter(P, k))
				{
					voxel.USGet(x, y, z) = 0;
				}
			}
		}

		for (int x = 1; x < VoxelX - 1; x++)for (int y = 1; y < VoxelY - 1; y++)for (int z = 1; z < VoxelZ - 1; z++) if (voxel.USGet(x, y, z) == 1)
		{
			for (int i = -1; i < 2; i++)for (int j = -1; j < 2; j++)for (int k = -1; k < 2; k++)if (voxel.USGet(x, y, z) >= 13) if (voxel.USGet(x + i, y + j, z + k) == 1)
				voxel.USGet(x, y, z) += 1;
			voxel.USGet(x, y, z) -= 1;
		}
		for (int x = 1; x < VoxelX - 1; x++)for (int y = 1; y < VoxelY - 1; y++)for (int z = 1; z < VoxelZ - 1; z++) if (voxel.USGet(x, y, z) >= 12) voxel.USGet(x, y, z) = 1; else voxel.USGet(x, y, z) = 0;

		usedCamera = new int[framecount]; for (int i = 0; i < framecount; i++) usedCamera[i] = variant[best][i];
	}

	void GetBestVoxel()
	{
		// Проходим по всем точкам шаблона, строим перпендикулярный ряд for i: = 1 to 30 do что за 30?

	}

	void InitFrame(int inframecount, unsigned char ***inframes, int frameX, int frameY)
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

	}

	void InitEtalon(std::vector<Vector3*> _male, std::vector<int> _malesizes, std::vector<Vector3*> _female, std::vector<int> _femalesizes)
	{
		males = _male;
		females = _female;
		malesizes = _malesizes;
		femalesizes = _femalesizes;
	}

	void Run()
	{
		Prepare();

		GetFirsCamPos();

		GetBestedBorder();

		GetNewCamPos();

		GetBorderDisp();

		GetNewCamPos();

		UpdateOreint();

		GetFoot();

		GetFirstVoxel();

		GetBestVoxel();

		Out();
	}
}

using namespace fsl;
