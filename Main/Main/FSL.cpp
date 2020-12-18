#include "FSL.h"
#include "FSLModules.h"

//GetBestVoxel:
//Проходим по всем точкам шаблона, строим перпендикулярный ряд for i: = 1 to 30 do что за 30?

//BestStopa:
//if Orientation=1 then e:=-1 else e:=1;
//Krit: = e * Krit;
// Какого черта ?

#define for3(k) for (int i0 = k - 1, i1 = 0, i2 = 1; i1 < k; i1++, i2 = (i2 == k - 1) ? 0 : i2 + 1, i0 = (i0 == k - 1) ? 0 : i0 + 1)

namespace fsl
{
#pragma region Local / Global Vars
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

	bool ismale;

	int Orientation;

#pragma endregion

#pragma region Support function

	void ToCam(Vector3 &base, int camnum)
	{
		base.x -= cams[camnum][0].x; base.y -= cams[camnum][0].y; base.z -= cams[camnum][0].z;
		double r = base.x * cams[camnum][3].x + base.y * cams[camnum][3].y + base.z * cams[camnum][3].z;
		base.x = (base.x * cams[camnum][1].x + base.y * cams[camnum][1].y + base.z * cams[camnum][1].z) / r * focuss[camnum] / K; base.y = (base.x * cams[camnum][2].x + base.y * cams[camnum][2].y + base.z * cams[camnum][2].z) / r * focuss[camnum] / K; base.z = focuss[camnum] / K;
		base.x -= immaxX / 2; base.y -= immaxY / 2;
	}

	bool isInCounter(Vector3 P, int u)
	{
		return counter[u].Get((int)P.x, (int)P.y) > 0;
	}

#pragma endregion

#pragma region Work with camera

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

#pragma endregion

#pragma region Work with edges

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

#pragma endregion

#pragma region Work with voxel

#define NS 35
	void GetFirstVoxel()
	{
		double *ix = new double[NS] { -120.415, -118.223, -107.241, -92.491, -76.224, -59.869, -43.488, -27.181, -10.797, 5.474, 21.693, 37.937, 54.23, 70.581, 86.815, 102.924, 118.374, 126.633, 123.11, 114.115, 102.974, 89.941, 76.335, 61.995, 45.691, 29.353, 13.098, -3.187, -19.441, -35.626, -51.862, -68.153, -84.333, -99.562, -112.899 };
		double *iy = new double[NS] { 4.29, 20.134, 32.102, 38.86, 40.352, 40.602, 40.873, 42.078, 41.615, 43.627, 46.018, 48.231, 50.013, 50.06, 47.825, 46.354, 40.912, 27.898, 12.412, -1.136, -13.152, -22.505, -31.278, -38.64, -40.317, -40.481, -38.396, -36.517, -34.39, -31.79, -29.515, -27.687, -25.138, -19.178, -9.744 };
		Vector2 T[NS][7];

		double boost = 4, bsize = boost / VoxelS;
		int varcount = framecount + 1;
		unsigned char **variant = new uchar*[framecount + 1];
		for (int i = 0; i < framecount + 1; i++)
			variant[i] = new uchar[framecount];
		for (int i = 0; i < framecount; i++)variant[0][i] = 1;
		for (int i = 1; i < framecount + 1; i++) for (int j = 0; j < framecount; j++) variant[i][j] = (j == i - 1) ? 0 : 1;

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
		delete[] ix; delete[] iy;
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

		ClearVoxel();

		usedCamera = new int[framecount]; for (int i = 0; i < framecount; i++) usedCamera[i] = variant[best][i];
	}

	void GetBestVoxel()
	{
		Centrovka();
		BestTop();
		BestStopa();
		ClearVoxel();
		Podgonka();
	}

	void ClearVoxel()
	{
		for (int x = 1; x < VoxelX - 1; x++)for (int y = 1; y < VoxelY - 1; y++)for (int z = 1; z < VoxelZ - 1; z++) if (voxel.USGet(x, y, z) == 1)
		{
			for (int i = -1; i < 2; i++)for (int j = -1; j < 2; j++)for (int k = -1; k < 2; k++)if (voxel.USGet(x, y, z) >= 13) if (voxel.USGet(x + i, y + j, z + k) == 1)
				voxel.USGet(x, y, z) += 1;
			voxel.USGet(x, y, z) -= 1;
		}
		for (int x = 1; x < VoxelX - 1; x++)for (int y = 1; y < VoxelY - 1; y++)for (int z = 1; z < VoxelZ - 1; z++) if (voxel.USGet(x, y, z) >= 12) voxel.USGet(x, y, z) = 1; else voxel.USGet(x, y, z) = 0;
	}

	void BestStopa()
	{
		std::vector<Vector3*> &ch = males, correct;
		std::vector<int> &chsizes = malesizes;
		if (!ismale)
		{
			ch = females;
			chsizes = femalesizes;
		}

		for (int i = 0; i < ch.size(); i++)
		{
			Vector3 *t = new Vector3[chsizes[i]];
			for (int j = 0; j < chsizes[i]; j++) t[j] = ch[i][j];
			correct.push_back(t);
		}

		double *kriteries = new double[ch.size()];
		double *koof = new double[ch.size()];

		Vector2 chablon[30][7];

		for (int n = 0; n < ch.size(); n++)			// поиск критериев
		{

			double maxX = -1e20, maxY = 1e20, minX = 1e20, minY = -1e20;
			for (int i = 0; i < chsizes[n]; i++)
			{
				if (ch[n][i].z < 30)
				{
					if (ch[n][i].x < minX) minX = ch[n][i].x;
					if (ch[n][i].x > maxX) maxX = ch[n][i].x;
					if (ch[n][i].y < minY) minY = ch[n][i].y;
					if (ch[n][i].y > maxY) maxY = ch[n][i].y;
				}
			}
			// создание шаблона
			//if (maxX - minX > maxY - minY)
			//{
			//	chablon[0][3].x = maxX; chablon[0][3].y = maxY;
			//	for (int i = 1; i < 8; i++) { chablon[i][3].x = maxX - i * (maxX - minX) / 7.0; chablon[i][3].y = maxY; }
			//	chablon[8][3].x = minX; chablon[8][3].y = maxY;
			//	for (int i = 9; i < 15; i++) { chablon[i][3].x = minX; chablon[i][3].y = maxY - i * (maxY - minY) / 6.0; }
			//	chablon[15][3].x = minX; chablon[15][3].y = minY;
			//	for (int i = 16; i < 23; i++) { chablon[i][3].x = minX - i * (minX - maxX) / 7.0; chablon[i][3].y = minY; }
			//	chablon[23][3].x = maxX; chablon[23][3].y = minY;
			//	for (int i = 24; i < 30; i++) { chablon[i][3].x = maxX; chablon[i][3].y = minY - i * (minY - maxY) / 6.0; }
			//}
			//else
			//{
			//	chablon[0][3].x = maxX; chablon[0][3].y = maxY;
			//	for (int i = 1; i < 8; i++) { chablon[i][3].x = maxX - i * (maxX - minX) / 6.0; chablon[i][3].y = maxY; }
			//	chablon[7][3].x = minX; chablon[8][3].y = maxY;
			//	for (int i = 8; i < 15; i++) { chablon[i][3].x = minX; chablon[i][3].y = maxY - i * (maxY - minY) / 7.0; }
			//	chablon[15][3].x = minX; chablon[15][3].y = minY;
			//	for (int i = 16; i < 22; i++) { chablon[i][3].x = minX - i * (minX - maxX) / 6.0; chablon[i][3].y = minY; }
			//	chablon[22][3].x = maxX; chablon[23][3].y = minY;
			//	for (int i = 24; i < 30; i++) { chablon[i][3].x = maxX; chablon[i][3].y = minY - i * (minY - maxY) / 7.0; }
			//}
			//for (int k = 0; k < 30; k++)
			//{
			//	double minr = 1e30, temp = 0;
			//	int minnum = 0;
			//	for (int i = 0; i < chsizes[n]; i++)
			//	{
			//		if (ch[n][i].z < 30)
			//		{
			//			temp = (ch[n][i].x - chablon[k][3].x) * (ch[n][i].x - chablon[k][3].x) + (ch[n][i].y - chablon[k][3].y) * (ch[n][i].y - chablon[k][3].y);
			//			if (temp < minr) { minr = temp; minnum = i;}
			//		}
			//	}
			//	chablon[k][3].x = ch[n][minnum].x;
			//	chablon[k][3].y = ch[n][minnum].y;
			//}
			// конец создани шаблона

			//double **buffer = new double*[100];
			//for (int i = 0; i < 100; i++)
			//{
			//	buffer[i] = new double[100];
			//}
			//double x, y, dx = maxX - minX, dy = maxY - minY; dx /= 100.0; dy /= 100.0;
			//for (int i = 0; i < 100; i++)
			//{
			//	for (int j = 0; j < 100; j++)
			//	{
			//		buffer[i][j] = 0;
			//		for (int i = 0; i < chsizes[n]; i++) if (ch[n][i].z < 30)
			//		{
			//			x = (ch[n][i].x - minX) / dx - 0.5; // -0.5 : Смещение к центру сегмента
			//			y = (ch[n][i].y - minY) / dy - 0.5;
			//			buffer[i][j] += 1.0 / (1.0 + (x - i) * (x - i) + (y - j) * (y - j));
			//		}
			//	}
			//}
			//for (int i = 0; i < 100; i++)
			//{
			//	delete[]buffer[i];
			//}
			//delete[]buffer;

			for (int k = 0; k < 30; k++)
			{
				chablon[k][3].x = ch[n][k].x;
				chablon[k][3].y = ch[n][k].y;
			}

			//for (int i0 = 30 - 1, i1 = 0, i2 = 1; i1 < 30; i1++, i2 = (i2 == 30 - 1) ? 0 : i2 + 1, i0 = (i0 == 30 - 1) ? 0 : i0 + 1)
			for3(30)
			{
				Vector2 Y = chablon[i2][3] - chablon[i0][3]; Y.SetLenght(1);
				double t = Y * chablon[i1][3];
				Vector2 X(Y.y, -(t + Y.x)); X.SetLenght(1);
				if (X * Y < 0) X = X * -1;
				for (int i = -3; i <= 3; i++) if(i != 0)
					chablon[i1][i] = chablon[i1][0] + X * 2.0 * i;
			}

			double
				G11 = -40, G12 = 40,
				G21 = -30, G22 = 30,
				G31 = -25 / 180 * PI, G32 = 25 / 180 * PI,
				G41 = 0.7, G42 = 1.3,
				Kritb = -1e20,
				xb, yb, fib, zb,
				r, co, si;

			Vector2 Cen, P; for (int i = 0; i < 30; i++) Cen = Cen + chablon[i][3];

			Cen = Cen / 30.0;

			Vector2 used[30][7];

			uchar temp = 0;

			for (int E = 0; E < 100; E++)
			{
				double x, y, fi, z;
				for (int I = 0; I < 120; I++)
				{
					x = G11 + random * (G12 - G11);
					y = G21 + random * (G22 - G21);
					fi = G31 + random * (G32 - G31);
					z = G41 + random * (G42 - G41);
					P = Vector2(x, y);
					co = cos(fi);
					si = sqrt(1 - co * co);
					double Kr = 0;
					for (int i = 0; i < 30; i++)
					{
						for (int j = 0; j < 7; j++)
						{
							used[i][j] = (chablon[i][j] - Cen) * z;
							used[i][j] = Cen + P + Vector2(used[i][j].x * co - used[i][j].y * si, used[i][j].x * si + used[i][j].y * co);
						}
						for (int j1 = 1, j2 = -1; j1 < 4; j1++, j2--)
						{
							for (int z = 0; z < 20; z++)
							{
								temp = voxel.Get((int)used[i][j1].x, (int)used[i][j1].y, z);
								if (temp > 0)
								{
									Kr += (temp - voxel.Get((int)used[i][j2].x, (int)used[i][j2].y, z)) * (1 + j1) * 0.25;
									z = 21;
								}
							}
						}
					}

					//if Orientation=1 then e:=-1 else e:=1;
					//Krit: = e * Krit;
					// Какого черта ?

					if (Kr > Kritb)
					{
						Kritb = Kr;
						xb = x;
						yb = y;
						fib = fi;
						zb = z;
					}
				}
				r = 0.97;
				if (xb < ((G11 + G12)*0.5)) G12 = G11 + r * (G12 - G11); else G11 = G12 - r * (G12 - G11);
				if (yb < ((G21 + G22)*0.5)) G22 = G21 + r * (G22 - G21); else G21 = G22 - r * (G22 - G21);
				if (fib < ((G31 + G32)*0.5)) G32 = G31 + r * (G32 - G31); else G31 = G32 - r * (G32 - G31);
				if (zb < ((G41 + G42)*0.5)) G42 = G41 + r * (G42 - G41); else G41 = G42 - r * (G42 - G41);
			}

			kriteries[n] = Kritb;

			Vector3 Cen3(Cen.x, Cen.y, 0), P3(xb, yb, 0);
			co = cos(fib);
			si = sqrt(1 - co * co);
			for (int i = 0; i < chsizes[n]; i++)
			{
				correct[n][i] = (correct[n][i] - Cen3) * zb;
				correct[n][i] = Cen3 + P3 + Vector3(correct[n][i].x * co - correct[n][i].y * si, correct[n][i].x * si + correct[n][i].y * co, correct[n][i].z);
			}

			double x0, x1, z0, z1;

			int len[VoxelZ];

			for (int block = 0; block < 20; block++)
			{
				x0 = minX + block * (maxX - minX) / 40.0;
				x1 = minX + (block + 1) * (maxX - minX) / 40.0;
				int x = (int)((x1 + x0) / 2.0 * VoxelS) + VoxelX / 2;

				int max = 0, lenght = 0;
				for (int z = 0; z < VoxelZ; z++)
				{
					len[z] = 0;
					for (int y = 0; y < VoxelY; y++) if (voxel.USGet(x, y, z) > 0) len[z]++;
					if (len[z] > max)
					{
						max = len[z];
						z1 = (z > VoxelZ / 3.0) ? 0 : z * 1.5;
					}
				}
				for (int z = 0; z < VoxelZ; z++) if (len[z] > max * 0.7) { z0 = (z > VoxelZ / 3.0) ? 0 : z; break; }

				double irad = 0, counteroflenght = 0;

				for (int i = 0; i < chsizes[n]; i++) if(correct[n][i].x > x0 && correct[n][i].x < x1 && correct[n][i].z > z0 && correct[n][i].z < z1)
				{
					uchar base = voxel.Get(correct[n][i].y, correct[n][i].y, correct[n][i].y);
					for (int rad = 0; rad < 30; rad++)
					{
						int V = 0, V2 = 0;
						for (int _x = -rad, rad2 = 0; _x <= rad; _x++, rad2 = sqrt(rad * rad - _x * _x)) for (int _y = -rad2, rad3 = 0; _y <= rad2; _y++, rad3 = sqrt(rad * rad - _x * _x - _y * _y)) for (int _z = -rad3; _z <= rad3; _z++)
						{
							V++;
							if (voxel.Get(correct[n][i].x + _x, correct[n][i].y + _y, correct[n][i].z + _z) == base) V2++;
						}
						if (V != V2)
						{
							counteroflenght++;
							irad += rad;
							rad = 1000000;
							break;
						}
					}
				}
				if(counteroflenght != 0)
					irad /= (double)counteroflenght;
				irad = 16 / (irad + 1e-6);
				if (irad > 1000)irad = 1000;
				kriteries[n] += irad;
			}
		}

		double maxkrit = kriteries[0], minkrit = kriteries[0];

		for (int n = 1; n < ch.size(); n++)
		{
			if (maxkrit < kriteries[n])maxkrit = kriteries[n];
			if (minkrit > kriteries[n])minkrit = kriteries[n];
		}

		double summ = 0;

		for (int n = 0; n < ch.size(); n++)
		{
			kriteries[n] = (kriteries[n] - minkrit) / (maxkrit - minkrit);
			kriteries[n] = kriteries[n] * kriteries[n];
			if (kriteries[n] < 0.1) kriteries[n] = 0;
			summ += kriteries[n];
		}

		for (int n = 0; n < ch.size(); n++)
			kriteries[n] /= summ;

		for (int n = 0; n < ch.size(); n++)
			kriteries[n] *= 0.5;
		int maxn1 = 0;
		maxkrit = 1e10;
		
		for (int n = 0; n < ch.size(); n++)
			if (kriteries[n] > kriteries[maxn1] && kriteries[n] < 0.5) maxn1 = n;
		int maxn2 = 0;
		maxkrit = 1e10;
		for (int n = 0; n < ch.size(); n++)
			if (kriteries[n] > kriteries[maxn2] && kriteries[n] < kriteries[maxn1]) maxn2 = n;

		for (int n = 0; n < ch.size(); n++)
			if (kriteries[n] < kriteries[maxn2]) kriteries[n] = 0;

		kriteries[maxn1] = 0.3333;

		kriteries[maxn2] = 0.1667;

		int stx = 150;

		for (int i = VoxelX - 1, r = 0; i >= 0; i++, r = 0)
		{
			for (int j = 0; j < VoxelY; j++) for (int k = min(0.8 * VoxelZ, 140 * VoxelS); k < VoxelZ; k++) if (voxel.USGet(i, j, k) == 1) r = 1;
			if (r == 1) { stx = i + 2; break; }
		}

		for (int x = stx; x < VoxelX; x++) for (int y = 0; y < VoxelY; y++) for (int z = VoxelZ - 1; z <= 0; z--) if(voxel.USGet(x, y, z) > 0)
		{
			double TZ = 0;
			for (int n = 0; n < ch.size(); n++) if(kriteries[n] > 1e-3)
			{
				#define trpcount 3
				int points[trpcount];
				double rads[trpcount + 1], r, k = 0, w;
				rads[0] = 0;
				for (int h = 0; h < trpcount; h++)
				{
					rads[h + 1] = 1e20;
					for (int i = 0; i < chsizes[n]; i++)
					{
						r = (correct[n][i].x - x * VoxelS) * (correct[n][i].x - x * VoxelS) + (correct[n][i].y - y * VoxelS) * (correct[n][i].y - y * VoxelS) + (correct[n][i].z - z * VoxelS) * (correct[n][i].z - z * VoxelS);
						if (r > rads[h] && r < rads[h + 1])
						{
							rads[h + 1] = r;
							points[h] = i;
						}
					}
				}
				r = 0;
				for (int h = 0; h < trpcount; h++)
				{
					rads[h] = sqrt(rads[h + 1]);
					w = exp(-rads[h] / (150));
					r += correct[n][points[h]].z * w;
					k += w;
				}
				#undef trpcount 
				r /= k;
				TZ += r / VoxelS * koof[n];
			}

			if (TZ > z)
			{
				for (int i = z; i < TZ + 1; i++)
					voxel.Get(x, y, i) = 1;
			}
			if (TZ < z)
			{
				for (int i = TZ + 1; i <= z; i++)
					voxel.Get(x, y, i) = 0;
			}
		}

		for (int i = 0; i < ch.size(); i++)
		{
			delete[] correct[i];
		}
		correct.clear();
		delete[]kriteries;
	}

	void BestTop()
	{

		double *ix = new double[NS] { -120.415, -118.223, -107.241, -92.491, -76.224, -59.869, -43.488, -27.181, -10.797, 5.474, 21.693, 37.937, 54.23, 70.581, 86.815, 102.924, 118.374, 126.633, 123.11, 114.115, 102.974, 89.941, 76.335, 61.995, 45.691, 29.353, 13.098, -3.187, -19.441, -35.626, -51.862, -68.153, -84.333, -99.562, -112.899 };
		double *iy = new double[NS] { 4.29, 20.134, 32.102, 38.86, 40.352, 40.602, 40.873, 42.078, 41.615, 43.627, 46.018, 48.231, 50.013, 50.06, 47.825, 46.354, 40.912, 27.898, 12.412, -1.136, -13.152, -22.505, -31.278, -38.64, -40.317, -40.481, -38.396, -36.517, -34.39, -31.79, -29.515, -27.687, -25.138, -19.178, -9.744 };
		Vector2 T[NS][7];
		Vector2 TT[NS][7];

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

			for (int j = 0, _j = -3; j < 7; j++, _j++)
			{

				if (abs(_j) == 1)  r = 0.6;
				if (abs(_j) == 2)  r = 0.8;
				if (abs(_j) == 3)  r = 1;
				if (_j != 0)
				{
					T[i][j].x = T[i][3].x + (T[i][j].x - T[i][3].x)*r;
					T[i][j].y = T[i][3].y + (T[i][j].y - T[i][3].y)*r;
				}
			}

			ix[i] = 1 + 0.002*(i - 17);
			if ((i == 1) || (i == 2)) ix[i] = ix[i] * 7;
			else
				if ((i = 3) || (i = 4) || (i = 33) || (i = 34) || (i = 35)) ix[i] = ix[i] * 4;
		}
		delete[]iy; iy = new double[7];
		for (int j = 0, _j = -3; j < 7; j++, _j++)
		{
			if (_j = 1 ) iy[j] = 6 / 1.2 * 3;
			if (_j = 2 ) iy[j] = 6 / 3.2 * 1;
			if (_j = 3 ) iy[j] = 6 / 6 * 0.3;
		}
		Voxel blur(VoxelX, VoxelY, 3);

		for (int x = 0; x < VoxelX; x++)for (int y = 0; y < VoxelY; y++) 
		{
			blur.USGet(x, y, 0) = 0;
			blur.USGet(x, y, 1) = 0;
			blur.USGet(x, y, 2) = 0;
			if (x != 0 && y != 0 && x != VoxelX - 1 && y != VoxelY - 1)
			{
				double t = 0;
				for (int _x = -1; _x < 2; _x++)for (int _y = -1; _y < 2; _y++)
				{
					t += (voxel.USGet(x, y, 5) > 0) ? 255 : 0;
				}
				blur.USGet(x, y, 0) = t / 9.0;
			}
		}
		double	xT, xTb, xTbb, G11, G12,
				yT, yTb, yTbb, G21, G22,
				zT, zTb, zTbb, G31, G32,
				fT, fTb, fTbb, G41, G42,
				zyT, zyTb, zyTbb, G51, G52;
		int ori, orib, oribb;
		double KritBB = -1e20;
		for (int E = 0; E < 22; E++)
		{
			ori = (E % 2 == 0) ? 1 : -1;
			double KritB = -1e20;

			if (E < 8)
			{
				G11 = 55;    G12 = 95;
				G21 = 35;    G22 = 70;
				G31 = 0.76;   G32 = 1.4;
				G41 = -25 / 180 * PI; G42 = 25 / 180 * PI;
				G51 = 0.7;   G52 = 1.4;
			}

			if (E > 12)
			{
				ori = oribb;
				G11 = xTbb - 5;    G12 = xTbb + 5;
				G21 = yTbb - 5;    G22 = yTbb + 5;
				G31 = zTbb - 0.05; G31 = zTbb + 0.05;
				G41 = fTbb - 5 / 180 * PI; G42 = fTbb + 5 / 180 * PI;
				G51 = zyTbb - 0.05;   G52 = zyTbb + 0.05;
			}

			for (int I = 0; I < 4000; I++)
			{
				xT = G11 + random * (G12 - G11);
				yT = G21 + random * (G22 - G21);
				zT = G31 + random * (G32 - G31);
				fT = G41 + random * (G42 - G41);
				zyT = G51 + random * (G52 - G51);
				double co = cos(fT), si = sqrt(1 + co * co), fi = zT * zyT * ori, Kr = 0;
				
				if (I % 100 == 0)
				{
					r = 0.96;
					if (xTb < (G11 + G12) / 2) G12 = G11 + (G12 - G11)*r; else G11 = G12 - (G12 - G11)*r;
					if (yTb < (G21 + G22) / 2) G22 = G21 + (G22 - G21)*r; else G21 = G22 - (G22 - G21)*r;
					if (zTb < (G31 + G32) / 2) G32 = G31 + (G32 - G31)*r; else G31 = G32 - (G32 - G31)*r;
					if (fTb < (G41 + G42) / 2) G42 = G41 + (G42 - G41)*r; else G41 = G42 - (G42 - G41)*r;
					if (zyTb < (G51 + G52) / 2) G52 = G51 + (G52 - G51)*r; else G51 = G52 - (G52 - G51)*r;
				}
				for (int i = 0; i < NS; i++)
				{
					for (int j = 0, _j = -3; j < 7; j++, _j++)
					{
						TT[i][j].x = xT + T[i][j].x * zT * co - T[i][j].y * fi * si;
						TT[i][j].y = yT + T[i][j].x * zT * si + T[i][j].y * fi * co;
					}
					double Q1 = ix[4] * (blur.Get(TT[i][4].x, TT[i][4].y, 0) - blur.Get(TT[i][2].x, TT[i][2].y, 0))
						+ ix[5] * (blur.Get(TT[i][5].x, TT[i][5].y, 0) - blur.Get(TT[i][1].x, TT[i][1].y, 0))
						+ ix[6] * (blur.Get(TT[i][6].x, TT[i][6].y, 0) - blur.Get(TT[i][0].x, TT[i][0].y, 0));
					double rr = sqrt(sqrt(abs(Q1))) * ix[i];
					if (Q1 < 0) Kr = Kr - rr; else Kr = Kr + rr;
				}
				if (Kr > KritB)
				{
					KritB = Kr; orib = ori; xTb = xT; yTb = yT; zTb = zT; fTb = fT; zyTb = zyT;
				}
			}
			if (KritB > KritBB)
			{
				KritBB = KritB; oribb = orib; xTbb = xTb; yTbb = yTb; zTbb = zTb; fTbb = fTb; zyTbb = zyTb;
			}

		}

		// Horror
		// Записываю координаты по частям Возможно меньше потребуеться памяти
		int lLen = 5704;
		Vector3 *LegLeo = new Vector3[lLen];
		float *LegLeoK = new float[lLen]{ -56.6, -57.8, -55.5, -56.6, -56.4, -57.2, -57.3, -57.8, -58.1, -58, -58.3, -58.8, -54.1, -54.7, -55.4, -57, -57.1, -58.5, -56.2, -58.8, -57.5, -59.4, -59, -59.8, -58.7, -60, -51.7, -53.1, -52.3, -55.2, -55.1, -55, -54.1, -55.5, -55.5, -55.7, -55.9, -59.2, -59.4, -59.1, -60.4, -60.3, -50.9, -51.1, -53.6, -53.6, -50.6, -51.8, -54.2, -54.5, -56.2, -56.2, -56.9, -60.3, -59.4, -60.7, -60.3, -60.8, -49, -52.8, -52, -53.4, -50.8, -56.4, -53.6, -58.4, -55.3, -59.5, -58.5, -59.6, -60.9, -59.4, -60.4, -60.7, -48.5, -51.3, -51.6, -51.7, -54.1, -50.4, -51.9, -53.8, -57.9, -56.3, -56.5, -58, -57.9, -59.7, -58.9, -60.3, -45.4, -46.3, -49.9, -47.3, -52.2, -48.9, -53.8, -55, -51.8, -56.9, -57.5, -58.6, -57.8, -58.1, -60.2, -60, -45.1, -46, -45.8, -50.5, -47.2, -53.7, -54.9, -55, -51.9, -52.6, -57.5, -55.3, -56.6, -56, -58.5, -58.6, -60, -59.4, -41.5, -44.4, -47.2, -44.9, -46.7, -48.1, -54.3, -55.1, -51.7, -56.9, -55.2, -56.1, -59.1, -59.8, -59.2, -46.7, -45.9, -41.6, -43.7, -44.5, -46.5, -52, -53.8, -51.1, -52.5, -56.3, -58.1, -59, -59.6, -59.4, -45, -40.6, -44.6, -47.4, -48.8, -45.4, -47.4, -52.8, -54.1, -55.2, -53
			, -55.4, -57.9, -57.3, -59.2, -58.7, -39, -43.2, -44.7, -46.1, -43.4, -49.5, -50.8, -52.9, -49.5, -51.2, -51.7, -53.1, -54.2, -55.7, -58.4, -58.4, -58.8, -42.6, -36.7, -42.6, -44.1, -41.6, -42.9, -44.9, -46.5, -52.2, -54.7, -54.5, -55.7, -56, -54.5, -56.2, -58, -58.3, -41.3, -35.3, -37.3, -43.5, -41.7, -42.6, -44.8, -50.3, -48.1, -49.3, -54.2, -55.3, -56, -56.7, -56.1, -57.6, -57.5, -33.8, -39.2, -42.1, -38.1, -40.4, -42, -49, -46.1, -51.5, -53, -54.2, -52.2, -53.9, -54.9, -55.9, -57.9, -32.5, -38.5, -33, -35.3, -36.8, -38.3, -39.8, -42.9, -43.9, -50.4, -48.2, -50, -54.7, -55.1, -53.3, -54.3, -56.6, -57.6, -56.8, -31, -37, -38.7, -35.7, -43, -37.7, -40, -47.2, -49.2, -45.7, -47.4, -53.1, -50.2, -51.9, -55.3, -55, -55.7, -56.5, -57.1, -30.7, -36.5, -37, -38.8, -35.5, -37, -44.5, -39.9, -47.3, -44, -45.9, -51.2, -52.5, -54.4, -51.9, -53.1, -54.2, -55.3, -56.2, -56.7, -29.6, -33.6, -31, -32.4, -33.2, -41.1, -42.7, -38.8, -46, -43.4, -49, -50, -51.9, -53.2, -51.3, -54.8, -56.6, -56.8, -56.7, -27.6, -26.8, -33.9, -29.9, -32.5, -34.8, -42.5, -36.6, -39, -46.1, -48.5, -49.4, -50.7, -47.1, -52.2, -50.7
			, -55.1, -55.6, -55.8, -56.5, -56.6, -56.6, -31.2, -31.8, -28.3, -29, -37, -33.3, -34.4, -36, -37.6, -44.5, -40.9, -48, -44, -49.5, -47.3, -48.6, -50.6, -51.3, -52.6, -53.8, -56.1, -56.3, -25.4, -31.1, -30.7, -26.9, -29.5, -30.9, -39, -34.6, -36.9, -44.1, -44.9, -42, -47.6, -45, -50.2, -48.3, -49.2, -53.2, -52.3, -53, -55.6, -55.2, -55.6, -24, -29.4, -29.5, -24.6, -26.1, -32.9, -35.1, -30.9, -38, -34.7, -36.4, -43.6, -40.3, -42.7, -48.7, -50.9, -50.6, -49.6, -54.1, -52.8, -53.4, -56.2, -56.1, -55.9, -21.3, -26.7, -27.1, -29.1, -25.5, -30.8, -33.8, -29.5, -37.4, -33.4, -40.9, -43, -45.1, -41.3, -43.7, -49.7, -50.4, -48.6, -49.1, -54.1, -52.4, -53.3, -55.1, -55.6, -19.9, -26, -25.4, -20.5, -27.6, -28.8, -30.3, -31.8, -27.4, -35.4, -30.5, -33.2, -35.2, -37.5, -45.2, -41.4, -43, -49.2, -46.1, -48, -49.6, -50.5, -54.7, -52.7, -53.8, -55.1, -55.8, -19, -25.7, -25.7, -26.3, -20.6, -27.8, -23.4, -24.4, -33.2, -28.2, -29.9, -32.2, -39.8, -41.8, -43.9, -45.2, -46.6, -44, -50.5, -46.7, -51.4, -50.1, -51.6, -54.7, -53.7, -55.8, -55.7, -18.4, -23.9, -23.7, -24.7, -26.2, -20.7, -27.7, -28.6, -29.9, -31.3, -26.9, -28.5
			, -37, -37.2, -34, -41.2, -43.3, -45.3, -41.8, -44.8, -45.3, -47.4, -48.1, -50.1, -51.5, -54.2, -54.8, -55.9, -16.2, -15.5, -17, -17.8, -26.1, -20.1, -21.9, -23.9, -25.5, -27.1, -35.7, -29.5, -31.1, -37.8, -40.2, -34.9, -38, -44.8, -46.1, -41.9, -43.9, -46, -51.3, -49.5, -52.5, -53.4, -52.6, -53.2, -55.1, -55.9, -14.8, -20.9, -20.1, -23.1, -17.1, -18.6, -20.2, -20.9, -22.7, -29.6, -30.9, -33.8, -29.8, -32, -40.1, -35.8, -37.4, -39.7, -46.2, -48.5, -44.9, -49.6, -47.5, -49, -53.4, -54, -54.7, -54, -56, -13.4, -18.7, -21.3, -22.1, -17.7, -24.2, -24.7, -25.9, -20.5, -28.9, -23.5, -26.1, -33.9, -29.1, -37.4, -33.7, -40.2, -41.6, -39.1, -40.9, -47, -43.9, -49.5, -46.5, -51.4, -52.1, -49.9, -53.4, -54.1, -53, -55.1, -12.2, -13.1, -19.3, -20, -20.4, -15.3, -22.5, -17.3, -25.5, -20.1, -26.9, -29, -25.1, -26.9, -29.3, -36.8, -38.9, -41, -42.4, -39.2, -42.1, -46.6, -48.2, -49.6, -51.2, -47.7, -50.1, -51.7, -52.2, -55, -55.1, -55.1, -55.8, -11.9, -16.1, -17.1, -18.4, -13.5, -21, -15.2, -16.7, -17.8, -26.3, -21.6, -29.9, -25.6, -33.4, -35.5, -31.7, -33.3, -41.1, -37.8, -45.2, -41.7, -48.4, -48.4, -50.4, -50.5, -49.2
			, -52.8, -54, -52.7, -54.7, -55.6, -9.7, -15.1, -11.3, -17.3, -12.1, -19.1, -21, -16.1, -22.1, -18.2, -20.2, -28.6, -30.7, -26.4, -28.1, -30.4, -32.3, -39.8, -36.4, -37.9, -40, -45.9, -46.6, -48.3, -45.7, -50.4, -48, -49.5, -51.1, -52.5, -53.9, -55.6, -55.6, -14.4, -15.3, -15.7, -11.3, -18.2, -14.1, -22.1, -16.4, -18, -20, -22, -23.6, -25.7, -34.2, -27.5, -29.8, -31.3, -37.9, -40.6, -36.6, -38.1, -40.4, -46.1, -46.7, -44.3, -45.4, -46.1, -47.8, -52.4, -52.6, -52.3, -53.1, -54.4, -55.4, -5.7, -7.6, -12.4, -7, -15.2, -16.9, -11.6, -20, -20.8, -16, -17.4, -25.9, -28.1, -28.8, -30.2, -25.4, -33.1, -35.2, -31.5, -34.1, -35.9, -43.6, -39.2, -40.8, -43, -44.7, -46.1, -47.4, -51.6, -50, -51.5, -54.2, -55.2, -55.3, -55.3, -11.2, -10.7, -5.6, -13.3, -7.9, -9.4, -11, -20, -20.9, -16, -18.6, -25.7, -27.5, -24, -31.8, -28.5, -35.8, -31.7, -33.8, -36, -43.8, -39.1, -45.6, -46.9, -48.1, -44.8, -47.3, -51.7, -51.7, -52.6, -52.7, -53.6, -54.9, -54.2, -2.1, -9.6, -10.6, -12.3, -14.2, -15.1, -17.1, -18, -12.6, -21.1, -16.1, -17.8, -24.7, -21, -23.1, -25, -27.3, -28.8, -30.3, -31.8, -33.2, -40.4, -43.1, -44.1
			, -40.8, -42.9, -43.6, -49.5, -50.7, -48.9, -49.5, -53.8, -53.8, -54.9, -54.9, -54.7, -1.5, -3.1, -10.3, -11.8, -13.5, -7.4, -9, -18.2, -19.8, -14.2, -16.4, -18.2, -20.4, -28.4, -24.4, -32.7, -27.9, -35.4, -37.1, -37.8, -34.8, -36.4, -43.9, -45.7, -42.4, -43.4, -44.3, -45.9, -50.4, -47.9, -49.3, -50.1, -51.9, -53.9, -53.8, -55.3, -0.7, -2.9, -3.7, -12.1, -7.4, -7.7, -10.2, -19.1, -14.2, -16, -17.5, -19.8, -27.8, -23.8, -31.5, -27.6, -34.6, -35.9, -32.7, -41.2, -36.3, -37.9, -44.5, -45.7, -47.7, -49.1, -49.4, -50.9, -49.3, -52.8, -52.2, -51.6, -55, -55.3, -0.5, -5.7, -8.6, -8.7, -10.6, -9.9, -12.1, -13.9, -7.9, -17.7, -12.5, -14.5, -16.1, -24.4, -26.5, -22, -28.7, -30.9, -26.3, -34.6, -30.7, -38.7, -35, -41.4, -42.8, -44.3, -41.9, -42.9, -47.8, -49.2, -46.8, -50.7, -48.6, -51.4, -52.3, -54, -54.2, -55.5, -54.3, -55.3, 2.8, -9.1, -8.6, -10.5, -4.1, -6.2, -14.8, -10.3, -18.6, -13.9, -15.8, -17.9, -19.8, -21.1, -28.9, -25.2, -27.2, -34, -30.8, -32.2, -40.1, -36.6, -42.9, -39.7, -41.4, -42.9, -47.5, -49.3, -47.1, -47.6, -52.2, -50.7, -51.6, -52.5, -53.1, -55.5, 3.4, 3, 1.3, -6.4, -8.7, -9.5, -4.2
			, -5.9, -7.5, -16.5, -11.7, -20.2, -15.7, -17.7, -25.2, -20.5, -22.8, -24.5, -26.4, -28.1, -34.9, -31.6, -39.9, -41.6, -38.3, -44.8, -41.4, -46, -44, -49.6, -49.8, -47.4, -49.9, -52.6, -53.5, -53.5, -53.7, -54.7, 5, 4.8, -4.6, -5.5, 1.2, -0.4, -7.9, -1.9, -9.9, -5.3, -7, -8.7, -9.9, -11.2, -13.9, -21.8, -24.4, -19.2, -21.5, -23, -25.2, -26.8, -34.3, -35.7, -38.4, -33.9, -40.8, -43, -39.9, -44.9, -43.2, -44.1, -49.7, -47.4, -48.1, -52.6, -50.5, -51.5, -53, -53.6, -55.3, 7, -3.1, -5.1, -4.6, -6, -0.5, -8.5, -10.8, -13, -8.1, -9.8, -11.6, -20.3, -21.2, -16.1, -18.4, -20.6, -27.9, -28.9, -24, -26.9, -29.6, -36.5, -38.2, -34, -36.7, -43.7, -40.2, -41.8, -43.4, -48.3, -49.6, -49.9, -50.8, -50.2, -50.4, -52.1, -54.4, -54.2, -55, -55.1, 7.1, 6, -2.2, -4.4, 2.3, 0.9, 0.9, -2.2, -10.2, -11.7, -13.7, -14.6, -16.1, -17.6, -19.8, -14.8, -23.1, -18.8, -26.5, -28.2, -24.2, -26.8, -33.4, -30.1, -31.2, -38.5, -39.6, -41.2, -41.9, -37.3, -44.7, -40.4, -42, -43.6, -44.4, -49, -47.7, -48.7, -48.8, -51.1, -53.9, -54.2, -55.1, -55.6, 8.7, 7.6, -1.2, 6.1, -3.3, -4.1, -5.8, -0.1
			, -1.4, -3.6, -13.1, -7, -9.3, -10.3, -12.8, -20.6, -15.4, -17.4, -18.2, -20.5, -28.2, -30.5, -32.4, -27.3, -35.9, -36.8, -38.1, -34.3, -40.7, -37.3, -39.7, -44.8, -42.9, -45, -45.9, -50.4, -52.2, -52.9, -53.3, -52.1, -53.9, -54.5, -55.2, 9.9, 8.3, 2.2, 7.5, 6, 4.5, -4.9, -6.7, -0.8, -1.4, -10.4, -4.9, -13.2, -7.3, -16.2, -17.7, -19.8, -20.7, -23, -23.8, -25.3, -20.8, -29.1, -25.1, -27.7, -35.5, -37.2, -39.2, -35.2, -43.3, -38.3, -39.9, -42.2, -43.9, -45.4, -49.9, -46.8, -51.5, -52.7, -53.3, -53.9, -54.3, -55.2, 13.3, 9.7, 2.7, 1, -0.6, -3, 3.2, -5.4, 0, -8.3, -10.5, -11.9, -6.8, -8.5, -17.1, -17.8, -19.5, -15.7, -17.1, -24.7, -26.2, -22.5, -24.5, -26.7, -27.5, -34.9, -30.9, -38.2, -34.6, -36.1, -42.8, -40, -45.5, -43.4, -48.8, -49.4, -46.8, -50.8, -52.5, -52.7, -53.4, -53.9, -54.8, -54.7, -55.2, 6.4, 4.2, 4, 9.8, 7.6, 6.5, -3.2, -4.7, 2.3, 0.1, -0.3, -9.1, -3.6, -5.4, -13.5, -7.8, -16.4, -17.7, -19.5, -14.6, -21.7, -18.6, -26, -28.2, -23.2, -24.8, -32.9, -33.6, -30.3, -31.2, -38.9, -40.9, -37.1, -38.6, -41, -42.5, -44.2, -48.3, -45.8, -50.9, -47.8, -52.2
			, -50.3, -54.3, -53.9, -53.8, -55.4, -55.8, 13.3, 8.3, 12.3, 2.9, 1.3, -0.3, 5.3, 3.5, -5.1, 0.1, -9, -2.6, -4.4, -5.9, -7.3, -15, -17.6, -11.2, -12.8, -15.6, -22, -24.1, -24.9, -20.6, -28.7, -31.6, -33, -29.2, -30.5, -32.7, -34.2, -41.7, -38, -40, -41.1, -43, -47.1, -49.6, -47.7, -47.7, -52.8, -50.3, -52.1, -52.9, -53.4, -55, -55.1, 9.5, 14.2, 4.2, 3.5, 1, 6.7, -1.9, 3.8, 2.9, -5.6, -1.1, -2.5, -3.6, -5.5, -12.9, -15.1, -9.9, -18.2, -20.2, -14.7, -15.5, -23.5, -19.9, -20.9, -23.5, -25.7, -25.8, -33.1, -29.2, -31.4, -39.1, -35, -41.4, -43.1, -40.4, -42.8, -48.5, -49.1, -46.6, -48.4, -52.3, -52.6, -50.6, -52.2, -53.5, -53.6, -55, 10.5, 15, 6.4, 4.8, 3.3, 1.1, 8.1, -2, -2.7, -4.2, 1.6, -6.4, -0.9, -2.2, -4.3, -4.8, -7.4, -14.4, -15.5, -16.6, -12.9, -14.5, -21.2, -22, -23.6, -20.9, -21.1, -23.2, -25.2, -27.4, -35.9, -32, -33.7, -40.5, -42.2, -39.2, -46.5, -42.7, -44.5, -46, -50.2, -51.8, -48.8, -50.6, -54, -54.5, -54.2, -55.1, -54.8, 19.3, 16.4, 9.4, 7.2, 13.3, 11.6, 10.4, 9.1, 0.1, 6.6, -2.5, -3.5, 1.5, -6.9, -0.9, -2.2
			, -10.1, -6, -7.5, -15.5, -15.8, -16.7, -18.9, -14.4, -15.2, -22.9, -24.5, -26.6, -27.5, -24.6, -31.8, -29.3, -30.8, -36.2, -33.2, -40.6, -42.8, -39.1, -45.4, -42.2, -43.8, -45.3, -50, -51.3, -52.5, -52.9, -51.1, -52.5, -53.7, -53.9, -55.1, 20, 9.6, 9.5, 7.5, 13.9, 12.3, 3, 1.4, 6.8, 6.4, 5, 3.9, 2, 0.4, -0.9, -8.9, -9.5, -6.3, -13.4, -14, -16.5, -16.7, -18.9, -19.8, -21.3, -17.5, -18.6, -24.6, -26.2, -27.6, -24.7, -32.6, -29.6, -36.7, -38.1, -35.5, -42.1, -44.2, -41.5, -43.1, -44.6, -49.2, -50.7, -51.5, -52.1, -50.3, -53.8, -53.6, -54.8, -54.7, 13.9, 20.5, 11.9, 17.6, 16, 15.3, 13, 11.1, 2.2, 1.4, 6.7, 5.5, -3.6, -3.6, -5.2, -0.1, -7.2, -8.7, -3.9, -5.5, -13.7, -9.1, -9.5, -16.8, -12.7, -19.1, -20.9, -17.2, -24.7, -26.8, -28.1, -29.2, -25.5, -33.3, -29.8, -31.2, -33.4, -40.3, -37.8, -44.4, -42.4, -47.7, -48.9, -50.2, -51.4, -51.5, -52.4, -50.6, -53.8, -53.9, -54.1, -53.1, -54.9, -54.9, 22, 15.4, 15.2, 13, 18.4, 9.4, 7.8, 14.2, 5.2, 11.2, 2.8, 8.6, 7.7, 6.4, 4.6, -4.4, 1.1, -0.5, -1, -3.4, -4.2, -12.2, -6.6, -13.5, -14.5, -10.6
			, -17.4, -14.7, -20.8, -16.9, -24.1, -20.3, -21.4, -24.1, -29.9, -26.3, -28.9, -34.8, -31.4, -38.2, -39.6, -41.3, -38.1, -40.2, -46.5, -43.3, -48.4, -46.4, -51, -52.2, -49.7, -54, -54.7, -53.9, -54.8, -55, -55.2, -55.1, 24.7, 17.8, 22.4, 20.3, 10.8, 16.4, 14.3, 5.4, 4.2, 10.2, 2.8, 8.3, -0.7, -0.8, 4.2, 2.7, 0, -5.8, -8.7, -3.2, -5.6, -7.4, -7.9, -14.3, -16, -17, -18.6, -15, -16.7, -22.2, -24.4, -20.4, -21.7, -23, -29.9, -32.1, -27.9, -34.9, -32.9, -41.1, -37.3, -38.8, -41.1, -42.6, -44.4, -50.4, -50.1, -51.7, -52.3, -50.5, -53.6, -54.5, -54.8, -54.3, -55.3, -54.3, 26.1, 17.4, 23, 19.9, 18, 16.4, 15.5, 7, 13.1, 11.4, 10.5, 9, 0.6, 5.5, -2.2, -3.7, -4.4, -0.5, -2.1, -2.4, -4, -4.8, -10.9, -7.3, -13.9, -14.6, -12.4, -17.8, -17.8, -21.2, -18.2, -24.1, -21.2, -23.2, -24.5, -26, -27.4, -28.7, -30.9, -33.6, -35.2, -41.9, -38.9, -45.6, -47, -47.4, -44.7, -50.6, -51.1, -50.5, -50.3, -52, -52.7, -52.8, -54.7, -54.9, -55.5, 27.4, 18.5, 24.2, 13.5, 12, 9.6, 16.8, 15.1, 13.8, 4.2, 11.2, 9.1, 2.7, 7.7, -0.5, -1.8, 2.8, 2, -5.3, -1.2
			, -2.6, -4.6, -10.6, -11.7, -8.7, -13.9, -15.5, -16.2, -13, -18.7, -15.3, -20, -18.5, -23.7, -26.3, -21.2, -28, -30.8, -34.2, -34.3, -31, -37.2, -38.6, -36.1, -38.3, -40.5, -45.6, -43, -48.8, -49.5, -50, -48.3, -49.3, -53.3, -52.4, -52.9, -52.9, -55.3, -54.1, -55.4, 20.6, 27.1, 23.5, 14.1, 19.1, 18.7, 17.5, 16.1, 14, 12.2, 10.4, 2, 6.3, 5.6, -2.2, -3.6, 1.5, -4.6, -6.5, -2.1, -8.6, -10.1, -11.6, -12.4, -13.8, -12.3, -15.7, -13.3, -18.6, -18.9, -16.3, -22.7, -19.2, -20.7, -22.1, -23.4, -30.1, -32.3, -35.1, -36.5, -31.1, -34, -36.9, -43.6, -44.9, -46.9, -48.3, -49.6, -51.2, -51.2, -49, -50.6, -53.3, -53.9, -54.9, -55.2, -54.6, -55.5, -55.4, 29.3, 28.5, 27, 24.6, 22.9, 13.3, 18.9, 18.1, 8.4, 8.3, 11.6, 11.9, 11.6, 11.1, 9.6, 8.2, 1.4, 5.8, 3.5, -0.8, -2.2, -3, 1.1, -0.4, -1.1, -2.5, -8.4, -8.6, -5.4, -7.2, -9.6, -13.3, -15.5, -13.3, -13.9, -15.3, -16.9, -22, -23.5, -24.3, -26.5, -28.7, -25, -27, -33.8, -29.8, -36.7, -33.4, -41.4, -42.4, -40.7, -45.8, -43.6, -45.2, -45.7, -47.2, -52.5, -49.1, -50.8, -51.8, -54.7, -54.4, -53.3, -55.8, -54.6, -55.8
			, -55.5, 29.8, 29.1, 28.4, 26.3, 25.3, 23.7, 22.1, 13.2, 18.2, 15.5, 15.8, 14.9, 13.2, 13.3, 5.3, 9.7, 7.9, 1.8, 0.2, -1.3, -2.8, -4.2, -4.7, -2.8, -7.9, -5.9, -4.9, -7.5, -10.8, -9.2, -9.9, -14.7, -15.1, -17.4, -18.5, -19.4, -17, -18.5, -19, -20.9, -22, -23.7, -31, -32.5, -29.2, -30.6, -38.3, -34.9, -42.4, -38.5, -44.9, -46.5, -43.6, -49.3, -50.9, -47.6, -52, -53.1, -50.3, -52, -52.8, -54.7, -54.2, -55.9, 32.1, 29.9, 29.5, 26.5, 19.6, 18.2, 24.5, 15.4, 20.4, 19.7, 17.7, 9.8, 8.3, 13.3, 5.6, 5.6, 10.3, 8.2, 2, 7, 0.2, 3.3, -2, -2.7, 1.2, 0.2, -5.4, -6, -4, -8.3, -8.7, -7.3, -8.8, -11.8, -10.2, -11.6, -14.8, -15.6, -13.8, -16.7, -16.7, -16.3, -20.3, -21.3, -22.8, -19.7, -25.1, -27.4, -24.2, -25.2, -32.6, -35.5, -32.1, -39.8, -38.5, -38.6, -40.1, -45.9, -47.9, -44.2, -49.2, -47.8, -47.8, -50.1, -53.7, -54.5, -55.5, -52.9, -55.1, -55, -56.1, -55.4, -56.3, 32.9, 26.7, 30.2, 30.7, 29.5, 22.5, 28, 18.8, 24.5, 23.5, 22.6, 13.9, 19.8, 20.1, 12.5, 17.5, 13.6, 13.7, 12.8, 6.3, 10.4, 5.7, 4.9, 7.1, 1.5, 3.7, 4.1
			, -0.9, -1.7, -2.9, -3.7, -2.1, -2.9, -4.9, -5.6, -5.6, -6.4, -6.3, -7, -10.8, -10, -9.8, -13.6, -15.6, -16.4, -16.2, -15.9, -18.4, -17.3, -22.1, -19.3, -25.1, -26.7, -23.9, -30.1, -27.3, -34.9, -31.5, -38.4, -35.8, -43.4, -44.8, -46.4, -43.9, -46.1, -46.9, -51.9, -53, -49.4, -54.6, -55.2, -53.1, -56, -54.5, -56.2, -56.3, 32.9, 27.4, 32.6, 31.4, 30.4, 22.7, 20.9, 26.6, 25.7, 22.8, 16.7, 15, 14.4, 12.6, 12.6, 10.5, 9.1, 13.2, 12.4, 10.7, 9.4, 7.4, 2.3, 5.1, -0.4, -1.1, -1.7, 0, 0, -1.3, -1.9, -4, -3.8, -3.8, -6.3, -5.6, -6.3, -7.5, -7.1, -7.7, -8.9, -9, -9.8, -11.7, -12.1, -12.6, -13.9, -15.4, -13.3, -14.4, -15.4, -19, -17.1, -18.6, -19.9, -21.3, -26, -26.8, -24, -30.5, -32, -29.5, -36.4, -38.6, -42, -42.8, -38, -44.4, -41.7, -43.2, -49.5, -46.6, -51, -48.1, -53.1, -50.7, -54.5, -54, -55.6, -56.4, -56.2, -56, 35.4, 27.3, 33, 31.1, 24.4, 22.9, 29, 26.9, 25.2, 22.3, 21.4, 14.6, 20.6, 13.9, 17.9, 17.2, 14.6, 9.2, 11.9, 11, 12, 10.2, 7.7, 8.7, 6.2, 6.5, 3.7, 3, 1.5, -0.7, 0.5, -0.3, -1.8, 0.2
			, -2.7, -1.3, -1.1, -4.5, -6.6, -7.9, -8.6, -9.1, -9.6, -10.8, -10, -10.8, -12.2, -12.6, -12.7, -13.2, -13.1, -16.5, -17.3, -18.9, -17.5, -20.6, -21.9, -22.9, -24.5, -25.3, -22.8, -29.9, -25.5, -32.7, -29.4, -36.8, -39.3, -35.2, -36.6, -38.8, -45.2, -47.4, -44.7, -45.1, -46.8, -51, -48.7, -53.2, -51.3, -52, -52.7, -52.2, -56.1, -56.6, -54.4, -56.3, -56.4, 37.9, 35.9, 36.5, 33.1, 33, 25.7, 31.9, 31.1, 29.4, 27.1, 20.8, 19.4, 23.8, 23.6, 20.8, 20.1, 18.2, 18.4, 16.5, 15.7, 14.9, 10.6, 8.5, 10.6, 11, 5.6, 4.7, 3, 5.9, 2.7, 4.1, 2.9, 1.3, 1.2, 0.3, 0.8, 0.8, -10.9, -11.2, -11.5, -12, -11.7, -12.8, -13.4, -15.6, -16.5, -17.5, -19, -20.6, -21.4, -18.1, -19.4, -24, -22.6, -23.2, -30, -32.2, -33.7, -30.3, -31.1, -38.8, -41, -38.2, -44.5, -42.5, -47.2, -48.6, -45.6, -47.2, -52.3, -50.3, -54.6, -54.8, -55.7, -56.1, -54.6, -54.6, -56.5, -56.8, 37.2, 32.1, 37, 36.3, 35.5, 33.9, 25.6, 22.8, 30.3, 29.6, 27.4, 26.6, 24, 17.8, 18.8, 15.7, 18.6, 15.4, 13.3, 10.8, 13, 12.6, 9, 8.2, 10.3, 6, 5.9, 5.7, 4.3, 3.3, 4.4, 2.8, 4.1
			, 3.3, 1.8, -12.1, -13.1, -12.7, -12.5, -14.2, -15.4, -14.1, -14.9, -17.9, -19.8, -17.3, -21.6, -22.7, -20.9, -25.6, -27.7, -23.7, -25.8, -30.9, -28.3, -35.4, -31.8, -32.5, -34.9, -41.3, -37.5, -40.2, -47.3, -48.6, -50.4, -46.4, -51.6, -52.9, -53, -54, -51.9, -51.9, -53.4, -53.5, -57.1, -56.8, -57.1, -57, -57, 39.1, 39.8, 38.2, 37, 31.3, 29.1, 27.7, 32, 30.5, 29.7, 29.5, 27.2, 26.9, 20.8, 25.9, 25.2, 22.3, 21.6, 20.7, 20.8, 18, 13.7, 14.2, 12.5, 15.8, 13.9, 12.2, 10.7, 9.2, 9.7, 7.1, 6.2, 6.4, 6, 6.2, 5.9, 5.8, -13.4, -13.1, -13.9, -14.2, -14.4, -14.4, -18.2, -16.3, -17.3, -17.7, -18.9, -24.2, -21.2, -22, -27.7, -30.2, -26.4, -28.4, -34.8, -30.7, -32.6, -33.3, -41.1, -36, -42.8, -39.7, -40.5, -42, -43.8, -45.7, -46.5, -51.6, -52.7, -50.4, -54.7, -52.8, -56.5, -54.8, -55.8, -57, -56, -57.1, 41.3, 41.1, 39.6, 36.6, 38.2, 37.8, 37.3, 28.3, 33.1, 33.8, 31.9, 23.2, 28.2, 28, 27.2, 20.9, 25.4, 22.7, 21, 18.2, 16.8, 17.7, 14.1, 12.2, 11.8, 11, 10.8, 9.8, 9.7, 8.9, 9.1, 8.5, 8.2, 8.3, 8.6, -14.2, -14.1, -14.8, -15.7, -16.3
			, -17.3, -16.3, -19.7, -17.4, -20.8, -23.1, -24.1, -22, -27.3, -23.8, -29.6, -31.9, -28.4, -34.9, -31.3, -32.7, -40.7, -43.4, -44.2, -45.3, -48.1, -42.8, -49.1, -50.4, -47.3, -52.2, -49.9, -50.2, -54.7, -56, -53.8, -56.5, -57, -55.8, -57.5, -56.5, -57.6, 42.1, 41.3, 39.8, 39.9, 39.2, 36.6, 36.5, 36, 29, 32, 24.9, 30.1, 30.4, 29.5, 26.9, 26.2, 24.7, 19.6, 18.6, 21.3, 19.6, 20.8, 15, 16.8, 16.3, 14.8, 14.5, 12.1, 11.6, 10.2, 11.4, 11, 9.7, 10.6, -14.5, -14.8, -15.2, -15, -15.2, -16.5, -16.7, -18.1, -19.3, -19.3, -19.6, -25.6, -26.6, -29.1, -24.5, -30.9, -33.1, -29.2, -30.6, -32.7, -41.1, -42.4, -44, -39.9, -47.5, -44.3, -45.2, -50.9, -47.4, -49, -53.4, -54.6, -55.6, -51.5, -55.9, -56.7, -57.3, -56, -57.7, -57, -57.8, 43.6, 40.5, 40.7, 41.1, 39.2, 39.4, 37.7, 37.6, 30, 34.9, 35.2, 28.3, 33.3, 32.4, 25.6, 29.7, 27.1, 26.9, 22.3, 24.3, 22.5, 21.9, 17.7, 19.8, 18.2, 18.4, 16.5, 16.6, 14.7, 14.5, 13.7, 13.7, 12.7, 12.5, -15.3, -14.9, -16, -16.4, -15.5, -17.2, -19, -19.1, -21, -19.6, -22.7, -20.1, -20.8, -22, -28.8, -29.8, -32.5, -33.7, -29.3
			, -32.1, -34.2, -41, -42.4, -38.4, -40, -42.1, -48.3, -50.1, -45.7, -51.4, -48.2, -54, -54.6, -55.9, -56.4, -56.7, -55.8, -54.9, -57.3, -57.8, -57.3, -57.7, 38, 38.5, 42.5, 37.6, 41.8, 35.3, 40.7, 40.8, 38.8, 31.1, 31, 30.4, 27.8, 32.5, 28.4, 30.4, 30.7, 30.1, 25.7, 27.2, 26.4, 21.1, 20.9, 20.2, 19.3, 20.2, 19.9, 18.5, 18.3, 16.7, 15.7, 15.9, 15.5, 15.6, 14.5, -15.4, -15.8, -15.8, -16.3, -17.3, -16, -17.4, -20.2, -17.4, -19.1, -20.1, -21.2, -26.8, -23.8, -25.3, -32.2, -33.8, -36, -36.8, -34.3, -42.6, -44.9, -45.8, -48.6, -43.7, -45.8, -51.5, -53.1, -53.4, -54.3, -54.4, -52, -57, -52.9, -53.7, -57.4, -56, -57.8, -57.1, -57.8, 46.9, 45, 39.7, 44.6, 41.8, 39.8, 35.1, 40.1, 38.4, 36.6, 35.7, 34.6, 34.3, 32.7, 32.8, 26.4, 29.2, 29.3, 23.8, 23.1, 25.7, 22.9, 20.8, 20.1, 19.4, 19.4, 18.9, 19.1, 17.7, 17.7, 17.1, 16.4, -15.7, -16.3, -16, -16.3, -19.2, -20.2, -18.2, -19.2, -19.7, -21.2, -26.1, -22.6, -29.3, -31.9, -33.6, -35.5, -37, -33.6, -35.7, -39.3, -39.8, -46.6, -47.5, -49.2, -50.8, -47.5, -52.7, -49.6, -51, -55.2, -53.2, -53, -54, -57.6, -56.4
			, -58, -56.3, -57.8, -57.1, 46.3, 40.5, 45.3, 39.6, 44, 42.8, 43, 34, 39.2, 32.2, 36.9, 37, 34.9, 29.5, 29.1, 33, 27.3, 25.8, 28.7, 28, 27.7, 26.8, 25.1, 25.1, 24.1, 22.5, 22.6, 20.7, 20.7, 20.6, 19, 19.2, -16.1, -17.5, -16, -17.5, -17.6, -18.2, -17.6, -21.2, -21.7, -23, -23.8, -25.5, -27.1, -23.5, -24.8, -26.1, -28.1, -30.2, -37.3, -40.6, -35.2, -36.5, -40, -45.7, -47.7, -48.9, -44.2, -46.6, -47.1, -53.9, -54, -51.2, -56.2, -54.1, -55.1, -54.7, -58, -56.8, -57.9, -58, 47, 41.7, 46.9, 45.7, 39.5, 45.2, 38.8, 37.3, 40.7, 40.8, 33.5, 38, 31.4, 35.2, 33.2, 33.4, 31.4, 32.6, 30.8, 29.3, 25.7, 25.1, 24, 24.2, 24.4, 24.5, 23.3, 22.8, 23.6, 22.3, -16, -16.2, -17, -16.7, -16.8, -20.5, -20, -18.7, -19.9, -24, -26.2, -27.2, -23.5, -25.5, -26.8, -28.9, -30.9, -37.9, -31.6, -35, -34.8, -35.9, -38.1, -39.5, -41.7, -42.4, -49.3, -46.8, -52.5, -48.4, -54.1, -54.3, -51.3, -56.2, -53.3, -56.5, -57.8, -56.3, -58, -57.5, -58.2, 49.4, 44.3, 47.4, 45.7, 44.9, 40.8, 39.4, 37.5, 42.3, 42, 40, 41.3, 35.2, 34.6, 38.4, 33.1, 35.4, 34.1
			, 33.6, 31.7, 28, 29.9, 27.5, 28.1, 28.1, 27.3, 25.4, 25.5, 24.5, 24.3, 24.8, -16.7, -16.2, -16.6, -18.8, -18.7, -20, -20.7, -22.7, -20.2, -25.7, -22.4, -23.6, -24.9, -32.1, -33.6, -35.2, -37.7, -38.3, -39.8, -34.5, -36.7, -38.1, -46.1, -47.9, -48.6, -50.5, -51.6, -47.8, -49.4, -50.8, -50.8, -56.4, -56.9, -54.1, -57.7, -56.4, -58, -57.6, -58.6, -58.5, 45.3, 44.6, 42.3, 45.9, 41.6, 45.7, 43.9, 42.8, 42.2, 40.3, 38.4, 38.5, 33.9, 37.7, 35.7, 34.8, 33.8, 30.9, 31.8, 31.1, 28.8, 30.1, 30.1, 27.6, 27.5, 27, 27.7, -17.4, -16.7, -16.3, -17.1, -19.4, -20.8, -18.8, -22.5, -20.4, -21, -22.5, -28.3, -29.9, -26.3, -33.8, -35.7, -36.9, -33.2, -34.6, -42.5, -43.9, -45.9, -46.7, -48, -49.3, -51.8, -47.6, -49.3, -49.6, -49.7, -55.6, -56.2, -54.4, -58.2, -56, -58.3, -57.5, -58.7, -57.8, 51.9, 52.3, 48.7, 48.3, 41.1, 40.5, 39.5, 43.2, 37.9, 38.7, 41.5, 35.8, 39.8, 36.8, 34, 35.2, 31.9, 31.2, 32, 32, 30.3, 31, 29.1, 28.8, 28.6, -16.5, -16.2, -17.4, -16.8, -17.1, -20, -18.3, -18.9, -23.5, -20.1, -22.3, -27.6, -29.3, -25.7, -32.2, -28.2, -36.3, -31.9, -40.6, -36.1, -37.5
			, -45.8, -41.1, -49.5, -50.1, -52, -47.9, -53.4, -50.2, -55.6, -55.8, -57.1, -53.8, -54.5, -53.8, -58.2, -58.2, -56.7, -58.2, -57.4, -58.6, 47.9, 45.8, 50.7, 49.9, 48.1, 45.3, 45.5, 43.6, 43.7, 41.8, 40.8, 37.4, 35.9, 37.9, 37, 35, 32.6, 32.8, 35, 33.1, 32.9, 31.7, 31.5, -16.2, -16.8, -16.2, -18.3, -18.3, -19.4, -21.8, -22.6, -24.3, -25.2, -22.4, -27.8, -29, -30.3, -32.5, -28.5, -30.5, -38.6, -40.4, -36.9, -44.2, -40.4, -48.8, -48.6, -51.3, -45.6, -46.7, -52.8, -49.8, -55.7, -52.8, -56.7, -56.9, -55.4, -57.7, -56.7, -56.5, -58.3, -58.2, 54.5, 50.1, 52.2, 46.4, 43, 42.7, 42.9, 45.7, 44.5, 40.2, 42.4, 42.9, 40.1, 36.1, 39, 38, 36.1, 37, 35.9, 35.9, 34.1, 33.8, 33.5, -16.4, -16.4, -17.1, -16.9, -19.4, -20, -20, -22.4, -23.4, -25.3, -21.4, -22.6, -28.7, -31.6, -26.5, -32.7, -36.1, -38.5, -41.5, -42, -37.7, -40.4, -41.3, -48.7, -49.7, -45.7, -52.9, -48.8, -54.2, -51.1, -55.7, -56.5, -53.2, -54.4, -57.5, -56.4, -57.9, -58.2, 55.4, 51.4, 53.4, 48.2, 52.3, 51.5, 50.6, 48, 47.8, 45.9, 45.1, 38.8, 41.1, 42.2, 39.8, 39.4, 36.7, 38, 37.9, 36.9, 35.5, 35.1
			, -16.2, -17.1, -16.2, -16.6, -17, -17.8, -21.4, -21.4, -22.2, -24.7, -22.1, -27.3, -29.5, -25.2, -26.6, -35.6, -35.6, -38.7, -39.6, -43, -44.4, -38.4, -45.9, -47.4, -49, -49.9, -51.2, -53, -53.2, -54.5, -55.8, -52.9, -53.8, -54.3, -55.9, -57.6, -57.1, -58.4, -57.6, 55.9, 52.4, 50, 48.5, 51.6, 50.8, 44.3, 43.7, 47, 47.2, 41.7, 45.4, 40.7, 43.3, 41.1, 41.1, 39, 37.7, 38.7, 38.2, 38.6, -16.2, -16.4, -17.8, -16.3, -16.6, -17.5, -17.6, -22.1, -22.9, -20.7, -27.1, -29.4, -24.7, -26, -27.3, -29.4, -31.3, -39.2, -42.5, -36, -44.6, -39.9, -47.6, -42.9, -50.1, -51.6, -53.1, -48.9, -54.1, -55.3, -56.1, -54, -54.7, -56, -57.1, -58.5, -57.9, 57.1, 53.6, 56.1, 54.8, 55.2, 52.7, 51.7, 47.8, 50.2, 48.5, 47.4, 43.1, 45.5, 45.5, 44.5, 44.3, 43, 42.7, 41.9, 41.2, 40.8, 39.7, 39.6, -16.4, -16.4, -18.1, -18, -19.8, -19.8, -20.4, -21.1, -19.8, -25.1, -21.6, -26.7, -23.4, -30.9, -26.7, -34, -37, -31.1, -33, -33.6, -36, -36.8, -44.9, -46.8, -48.8, -50.1, -51.4, -46.6, -48.2, -54.1, -55.4, -51, -53.4, -54.3, -57.4, -57.8, -56.1, -56, -58.1, -58.8, -58.2, 58.7, 54.5, 54.8, 57.7
			, 51.9, 50.5, 49.8, 52.1, 50.4, 50.4, 48.7, 48.7, 48.8, 46.5, 44, 46.5, 43.8, 45.3, 43, 42.9, 42.6, 40.7, -16.3, -16.2, -17.2, -16.2, -16.6, -17.5, -21.6, -20.9, -23.3, -25.2, -26, -22.3, -28.6, -24.9, -26.1, -34.2, -36.6, -30.9, -33.6, -35.8, -43.5, -38.2, -46.9, -47.6, -49.3, -50.9, -45.2, -46.7, -53.7, -49.9, -50.1, -50.9, -53.4, -57.4, -55.2, -58.3, -58.6, -58.8, -58.3, -58.9, 55.1, 58.8, 57.4, 55.7, 54.1, 50.5, 51.5, 47.3, 46.7, 45.2, 45.6, 47, 47.6, 44.8, 44.9, 45.1, -16.2, -17.2, -16.2, -17.5, -18.4, -18.9, -20.9, -21, -22.9, -20.3, -21.1, -22.3, -29.2, -24.3, -32, -34.5, -36.9, -30.3, -39.3, -34.4, -36.5, -46.3, -47, -49.6, -49.1, -50.9, -46.5, -53, -54.2, -49.5, -56.7, -52.4, -53.4, -54.4, -57.2, -58.5, -56.2, -58.5, -57.8, -58.7, -58, -59.1, -59, 60.6, 57.2, 56.3, 58.2, 56.8, 52.6, 51.7, 54.4, 53.5, 51.8, 48.5, 47.9, 50, 48.9, 47.2, 47.6, 47.4, 48, -16.1, -16.5, -16, -16.7, -18.7, -16.3, -19.8, -20.6, -22.9, -23.9, -26.4, -21.9, -23.6, -31.3, -33, -36.7, -36.8, -38.6, -40.3, -42.3, -35.9, -44.7, -45.9, -47.7, -49.1, -44, -51.1, -47.5, -54, -54.6
			, -49.6, -55.9, -52.8, -57.6, -58.3, -59, -55.9, -59, -59.2, 61.2, 60.9, 58.6, 60.8, 60.7, 59.5, 58, 55.8, 54.9, 54.8, 50, 49.5, 49, 51.1, 50.2, 48.7, 49.1, -16.1, -16.1, -16.9, -18.3, -18.6, -19.1, -21.1, -19.2, -19.4, -21, -26.2, -28.6, -29.6, -25.7, -33.7, -34.7, -36.8, -37.7, -39.3, -41.1, -36, -37.4, -46.2, -47.9, -49.2, -44.8, -51.1, -52.3, -54.1, -54.4, -55.8, -55.9, -57.1, -57.5, -57.9, -58.4, -58.6, -58.3, -59.4, -58.6, -59.4, 61.7, 60.5, 61.3, 59.4, 60.8, 58.7, 61, 60.4, 55.4, 58.3, 54.4, 53.2, 53.3, 53.3, 53.4, 51.4, 50.5, 51.7, 50.8, -16, -16.5, -16.2, -17.5, -16.7, -19.6, -22.3, -23.2, -24, -25, -27.6, -29.1, -23.7, -26.1, -33.3, -28.4, -29.1, -38.6, -33.9, -43.4, -38.1, -41, -41.8, -48.6, -50.2, -51.3, -47.3, -48.8, -54.6, -50.9, -52.8, -57, -57.5, -57.9, -58.5, -59.2, -57.2, -59.4, -58.1, -59.6, -59.6, 62.7, 62.9, 61.9, 60.8, 59.9, 60.9, 59.9, 59.3, 57.8, 57, 55.1, 53.4, 52.7, 52.5, 52, 52.3, 52, -16, -15.9, -16.2, -16.5, -18.4, -18.9, -20.1, -20.8, -22.2, -23.8, -25.8, -21.6, -28.9, -24.6, -25.6, -34.2, -35.8, -38.7, -40, -34.7, -36.8
			, -46.2, -47.3, -49.1, -44.6, -51.8, -52.5, -48.5, -54.4, -49.9, -52.9, -54.7, -55.6, -56.9, -57.9, -59.3, -58.3, -59.6, -59.8, 63.2, 62.3, 61, 62.8, 58.6, 59.6, 56.3, 56, 57.8, 53.9, 55.8, 53.4, 54.8, -15.9, -17, -15.9, -16.1, -17.7, -19.9, -20, -21.7, -22.1, -24.8, -20.8, -26.3, -27.9, -30.3, -24.6, -32.9, -35.2, -36.7, -31.3, -33.9, -40.1, -35.7, -36.9, -46.4, -47.5, -48.2, -49.6, -51.4, -52.6, -54.3, -49.8, -55.4, -55.7, -52.3, -53.1, -53.9, -57.6, -58.3, -55.7, -55.5, -59.1, -59.8, -58, -59.2, -59.7, 64, 63.1, 63.6, 61.2, 59, 60.5, 56.3, 58.2, 54.9, 55, 56.2, 55.4, -15.9, -16.9, -15.8, -18.1, -19.1, -20, -20.6, -21.3, -22.8, -23.4, -25.1, -21.8, -28.1, -30.2, -31.4, -26.6, -28, -30, -37.9, -39.4, -34.2, -35.5, -44.4, -46, -40.7, -48.5, -50.2, -51.5, -47.1, -48.6, -54.2, -50.7, -51.2, -56.9, -53.3, -58.9, -55.5, -56.4, -59.2, -58, -59.6, -59.2, -59.8, 64, 64.2, 61.3, 60, 60.6, 58.3, 57.2, 56.4, 55.9, -15.9, -16.1, -15.9, -18.2, -19.2, -19, -21.1, -21.5, -19.6, -19.6, -25.6, -26.6, -28.8, -29.1, -24.9, -32.5, -34.7, -30.6, -30.7, -38.5, -33.8, -36, -37.1, -39, -47.8
			, -42.4, -44.5, -50.4, -51.9, -53, -53.8, -54.8, -56.4, -53.3, -57.5, -54.1, -59.2, -57.1, -57, -59.9, -59.7, -59.2, -59.9, -60.1, 64.4, 63.8, 61.4, 62.1, 61.5, 59.9, 59.1, 57.3, -15.8, -15.9, -16.4, -16.1, -17.4, -19.4, -16.5, -17.3, -22.8, -24.2, -20.4, -27.2, -23.1, -31, -31.4, -34.7, -34.8, -35.7, -38.5, -40.4, -32.2, -34.3, -43.2, -38.4, -47.2, -49, -43.1, -45.2, -51.3, -52.4, -48.7, -55, -55.4, -55.6, -56.6, -53.3, -58.7, -54.9, -59.2, -57.4, -57.5, -57.8, -60, -59.8, 64.3, 64, 63, 61.2, 59.8, 59.7, 58.3, 59.1, -16, -15.7, -17.4, -15.8, -16, -16.8, -19.2, -21.3, -19.1, -19.3, -26, -26.1, -21.8, -28.8, -31, -32.8, -26.9, -35.8, -37.9, -32.3, -33.8, -35.9, -37.5, -46.1, -49.2, -49.3, -44, -46.9, -47.2, -53.9, -50.3, -56.2, -52.3, -53.3, -59, -59.2, -55.7, -56.9, -59.6, -60.1, -58.6, -60, -60.3, -60.4, 62.6, 62.8, 61.7, 61.3, 59.5, 60.3, -16.1, -15.6, -16.9, -15.7, -16.5, -16.8, -17.3, -18.2, -22.9, -19.5, -20.5, -22.7, -23, -23.6, -32.1, -27, -28.3, -30.3, -31, -40.5, -33.1, -42.8, -45.4, -45.5, -46.8, -48.5, -43.3, -45.3, -52.1, -47.5, -54.3, -50.3, -51.8, -51.4, -57.3, -57.9
			, -58.6, -59.2, -57.1, -59.3, -60, -58, -60.1, -60, -59.2, -60.3, 63.8, 63.5, 62.5, 61.5, 61.3, 60.9, -15.6, -16.6, -15.7, -17.5, -17.9, -16, -16.9, -17.9, -21.9, -18.8, -20.3, -26, -28.5, -28.3, -31.3, -25.7, -34.6, -36.8, -30.4, -39, -41.9, -44.1, -44.6, -46.1, -40.4, -43.3, -50.3, -45.1, -52.3, -54.7, -49.6, -51.1, -57.2, -53.6, -58.7, -56.3, -59.7, -60.4, -58.6, -60.4, -59.6, -60.5, -60.5, 62.2, -15.5, -15.7, -16.5, -17.9, -15.8, -16.7, -17.2, -21.9, -22.6, -22.5, -25.6, -21.4, -21.9, -22.6, -24.3, -32.4, -33.8, -28.4, -30.3, -38.5, -39.5, -40.9, -35.5, -36.9, -46.8, -40.5, -49.9, -50.7, -45.5, -47.3, -48.8, -49, -51.3, -52.9, -57.7, -58.6, -58.7, -59.1, -59.8, -60.4, -60.7, -60.6, -59.1, -60.5, -60.8, -61, -16.3, -15.5, -17.2, -15.5, -16, -18.6, -16.3, -22.3, -22.3, -19, -25.6, -27.4, -22, -23.8, -31.9, -25.8, -34.8, -29.7, -32.7, -40.4, -42.5, -45.2, -39.1, -40.6, -48.8, -43.3, -51.8, -46.6, -54, -55.7, -56, -56.6, -52.5, -53.2, -54, -58.8, -59.4, -60.1, -60.5, -58.6, -60.7, -59.8, -60.8, -60.8, -15.7, -15.3, -16.2, -15.7, -18.6, -18.9, -20.6, -21.5, -22.3, -22.9, -26, -27.2, -27.9, -29.5, -30.4
			, -25.9, -26.7, -36.3, -36.5, -33.5, -39.8, -41.2, -43.1, -37.8, -47.4, -41.4, -49.6, -51.4, -52.4, -54, -55, -50.7, -51.5, -57.8, -58.8, -58.9, -59.3, -60, -61, -59.1, -60.9, -59.8, -61.1, -59.5, -61.2, -15.2, -16.3, -15.1, -15.7, -18.9, -19.3, -16.9, -17.6, -22.9, -24.4, -25.3, -21.4, -22.2, -22.7, -31.1, -32, -26.5, -27.9, -37.5, -32.1, -40.3, -32.3, -42.1, -43.4, -46, -47.2, -40.8, -43.6, -43.7, -52.7, -46.8, -55, -55.2, -57.1, -51.6, -57.4, -58.1, -54.4, -59.2, -59.8, -56.9, -60.3, -61, -58.8, -61.1, -60.2, -61, -60.9, -15.2, -16, -16, -15.2, -17.9, -18.8, -15.7, -16.1, -16.9, -17.6, -18.2, -25.4, -26.3, -27.3, -22.1, -24.1, -25.6, -29, -31, -33.1, -35.1, -44.9, -39.3, -39.4, -48.9, -42.3, -51.4, -52.1, -46.9, -48.5, -50.1, -50.4, -58, -58.4, -53.4, -60, -61, -57.2, -60.6, -61.5, -61, -60.6, -61.6, -60.6, -61.7, -15.7, -15.4, -17, -15.1, -15.5, -19.2, -19.5, -16.8, -21.7, -23.8, -19.9, -26, -28.4, -29.9, -30.6, -24.3, -28.4, -31.1, -32.7, -35.3, -37.3, -46.7, -40.9, -50.5, -51.2, -46.2, -54.6, -49, -56.8, -56.4, -52.5, -58.5, -58.9, -56.1, -57, -57.9, -58, -61.1, -60.4, -61.4, -61.3, -15
			, -16.4, -14.9, -15.5, -15.7, -18.8, -16.2, -19.9, -21.5, -22.8, -24.9, -25.9, -27.3, -22.8, -30.8, -25.9, -37.4, -38.4, -39, -41.1, -35.3, -37.4, -46, -47.3, -50, -51.3, -52.2, -47.6, -48.9, -55.7, -50.3, -58.2, -58.5, -59.3, -59.6, -60.5, -60.9, -58, -61.2, -59.5, -14.9, -15.3, -15, -16.9, -17.7, -15.6, -16.1, -16.6, -16.9, -21.8, -19.2, -25.9, -26.6, -28.8, -28.8, -25.2, -26.5, -38.2, -39.8, -41.9, -36.8, -46.1, -47.4, -41, -49.5, -50.3, -51.6, -52.9, -48, -49.6, -57.1, -56.9, -52.8, -58.8, -55.4, -56.2, -56.8, -14.8, -14.6, -14.8, -16.6, -16.7, -15.4, -18.5, -16.4, -21, -18.1, -23.3, -20.7, -21.1, -27.7, -23.3, -25.3, -28.6, -32.7, -34.8, -44.8, -46.2, -47.6, -42, -44.3, -51.9, -52.9, -54.9, -56.7, -51.1, -58.5, -59, -53.5, -53.7, -60.4, -15.1, -14.5, -15.9, -14.5, -15.2, -19.1, -16.2, -20.1, -21.4, -18.2, -18.2, -23.7, -26.2, -27.1, -28.9, -24.1, -26.8, -30, -33.5, -43.5, -37.7, -49, -41.3, -44.2, -45, -53.3, -53.9, -56.2, -50.5, -58, -51.7, -53.7, -14.3, -14.8, -14.5, -16.3, -17.8, -18.9, -19.4, -20.1, -22.7, -17.7, -19.1, -26, -27.3, -24.9, -27.5, -29.4, -31.5, -34.2, -45.3, -46.1, -47.9, -48.1, -49.9
			, -51.7, -46.4, -47.8, -49, -49.3, -52.2, -14.4, -14.2, -14.3, -16.3, -16.2, -18.3, -18.8, -16.4, -22.1, -19, -19.2, -19.2, -26.8, -22, -23.6, -27.2, -30.8, -35, -45.3, -40, -48.5, -42.9, -52.1, -46.1, -48, -55.6, -14.2, -15.2, -14.1, -16.2, -14.5, -18.2, -15.5, -16.7, -20.6, -21.1, -18.2, -22.3, -24.7, -24.9, -20.4, -21.7, -24.2, -26.9, -29.9, -34.4, -45.4, -39.4, -41.1, -42.3, -44.5, -45.9, -55.4, -55, -14.4, -14.3, -15.8, -15.3, -15.1, -15.4, -19.3, -19.7, -16.5, -21.5, -17.3, -23.4, -19.2, -26.7, -26.3, -22.3, -26.7, -27.6, -30.3, -35.2, -46.1, -47.1, -48.5, -51.7, -51.8, -46.2, -53.8, -14.3, -14.6, -14.8, -16.8, -14.7, -17.6, -17.5, -15.4, -18.7, -18.8, -16.4, -16.9, -18.1, -18.7, -23.8, -19.9, -27.5, -24.3, -27.7, -31.8, -36.6, -47.1, -49.2, -49.9, -44.3, -53.8, -14.6, -15.2, -14.7, -16.6, -14.7, -15.4, -18, -16.4, -20.3, -21.6, -23.4, -24.3, -25.1, -27.1, -21.9, -24.8, -27.8, -30.8, -35.3, -47.7, -42.2, -50.7, -52.6, -14.7, -15.3, -14.4, -16.4, -15.2, -15.8, -18.4, -20.1, -17, -22.9, -23.6, -24.1, -22, -24.9, -26, -30.6, -36, -37.5, -40.4, -42.3, -14.7, -14.5, -15.6, -15.8, -14.6, -15.3, -18.3, -16.1, -16.1
			, -17.7, -23.7, -18.7, -26.1, -21.4, -24, -28.1, -31.3, -35.4, -38.3, -15.2, -14.6, -15.8, -14.3, -15.1, -15.4, -18.9, -16.4, -20, -17.1, -18.3, -24.3, -25.9, -21.7, -26.7, -27.6, -31.4, -35.5, -37.3, -39.2, -15, -14.2, -16.3, -17.1, -15.1, -19.4, -15.6, -16.3, -22.2, -19, -24.1, -26, -21, -22.2, -25.5, -28.8, -32.9, -35.9, -38.5, -14.1, -14.2, -15.2, -17.2, -14.5, -15.4, -19.5, -20.3, -20.8, -17.9, -23.4, -24.6, -22, -24, -25.6, -29.1, -32.3, -35.7, -14.1, -13.9, -15.5, -15.6, -14.4, -15, -15.4, -20.7, -16.1, -22.1, -23.3, -18.5, -20.9, -23.8, -26.2, -29, -30.3, -32.4, -35.1, -14, -14, -14.4, -15.5, -15.8, -17.8, -18.9, -19.9, -20.6, -21.4, -17.4, -18.3, -19.9, -21.7, -25.7, -27.8, -30.4, -34.3, -13.6, -14.3, -13.7, -13.9, -14.3, -18, -19.1, -19.2, -16.3, -21.1, -22.2, -23.4, -20.7, -23.2, -27, -30.5, -33.2, -13.9, -14, -14.9, -16.1, -13.9, -14.7, -18.2, -18.6, -15.5, -16.2, -23.2, -23.7, -19.3, -21.9, -24.3, -27.2, -31.2, -13.9, -13.7, -14.8, -13.7, -16.5, -14.8, -17.8, -15.7, -20, -22.4, -22.1, -18.6, -20.2, -23.4, -24.8, -27.3, -30, -13.7, -13.5, -13.6, -14.5, -15.7, -17.1, -17.6, -18.9, -15.2, -16.3
			, -21.4, -23.2, -19.1, -21.5, -24.7, -28.7, -13.4, -14.5, -13.3, -16.1, -16.8, -17.5, -14.1, -15.4, -15.5, -20.8, -22.3, -18.9, -20.8, -23.2, -23.9, -29.1, -13.3, -14, -13.2, -15.4, -17.1, -13.9, -18.8, -19.4, -20.6, -22.1, -22.5, -19.1, -21.5, -24.8, -13.4, -13.1, -14.5, -13.1, -13.5, -13.8, -14.8, -18.6, -16.1, -16.6, -17.1, -19.7, -21.5, -24.2, -13.1, -14.5, -13, -15.6, -16.5, -16.4, -18.5, -14.9, -19.5, -21.2, -22.5, -19.1, -20.3, -22.3, -12.9, -13.9, -12.8, -13.5, -16.3, -16.6, -18.4, -18.8, -19.8, -20.3, -16.6, -19.2, -22.4, -13, -12.7, -13.5, -12.8, -15.2, -15.7, -13.9, -18.1, -20, -20.4, -20.7, -17.6, -19.8, -12.6, -13.5, -12.6, -14.7, -12.7, -13.3, -17, -17.3, -15, -20.6, -16.2, -18.1, -12.4, -13.6, -12.8, -15.1, -15.4, -13, -13.6, -14.3, -14.5, -20, -20.8, -17.5, -19.4, -12.8, -12.2, -13.9, -14.6, -12.6, -16.3, -14.1, -17.7, -19.7, -20.4, -18.8, -12.1, -13.6, -13.5, -12, -12.4, -16.4, -17.6, -17.9, -19.2, -15.3, -12.1, -12.8, -11.8, -14, -14.8, -12.5, -16.7, -17.6, -15.3, -19.3, -15.9, -12, -11.7, -12.4, -12, -15, -11.9, -13, -13.4, -18.5, -14.7, -12, -11.8, -11.4, -13.6, -14.7, -15.5, -12.8, -13.9
			, -18.4, -19, -11.7, -11.2, -12.8, -11.1, -11.5, -15.5, -15.4, -11.8, -10.9, -12.6, -11.6, -14.5, -14.8, -11.2, -11.3, -12.6, -11, -11.8, -11, -46.3, -21.6, -28.2, -20.5, -20.7, -49.1, -36.7, -44.7, -45.7, -26.8, -44.3, -40.4, -30.4, -35.4, -27.5, -28.1, -43.6, -43, -23.4, -24.3, -24.5, -25.6, -30.7, -21.5, -22, -22.8, -50.4, -48.2, -48.7, -49.5, -49.5, -44.3, -47.9, -43.2, -42.9, -8.4, 9.4, -45.4, -57.8, -57.3, -57.4, -57.7, -54.9, -57.2, -57.1, -54.4, -52.9, -54.4, -55.1, -56, -51.7, -52.1, -52.7, -52.4, -52.4, -49.8, -52, -53.3, -52.8, -52.7, -48.3, -49.7, -52.2, -52.1, -46.9, -49.1, -48.3, -50.5, -47.6, -45.6, -49.5, -46.3, -44.3, -42.9, -42.5, -45.7, -41.9, -42.5, -43.8, -45.3, -39.6, -38.6, -43.5, -37.1, -41.8, -34.9, -41, -34.8, -32.4, -32.5, -31, -36.6, -30.6, -34.3, -28.2, -33.5, -26.5, -25, -31.1, -29.4, -30.4, -26.6, -28.5, -23.6, -21.3, -25.7, -25.9, -19.2, -19.4, -24.9, -25.9, -18.5, -24.1, -24.1, -17.9, -24.1, -16.5, -21.9, -21.4, -12.2, -20.7, -12, -18.2, -8.8, -16.9, -16.7, -16.6, -6.5, -7.7, -13.9, -11.8, -5.1, -10.9, -3, -3.8, -0.5, -8, -7.3, -0.6, -6, 0.5, -4.7, -5.1, -4.7
			, -3.9, 5.1, -1.4, 6.5, -1, 7, 1.4, 8.2, 1.1, 10.1, 3, 11.7, 4.4, 13.5, 6.8, 13.3, 14.3, 8.5, 9.4, 18.5, 20.3, 12.4, 13.4, 21.8, 15, 23.2, 23.6, 16.7, 26.7, 18.7, 27.2, 26.4, 21, 30.5, 31.1, 31.9, 31.8, 34.6, 26.1, 36.3, 36.8, 29.9, 30.1, 38.8, 38, 39.9, 32.4, 41.9, 34.1, 41.4, 35.6, 36.8, 38.6, 44.9, 46.8, 40.2, 41.1, 42.2, 48, 43.9, 51.7, 46.4, 54.1, 48.8, 55.6, 49.7, 56.3, 57.4, 58.3, 54.1, 59.7, 56.2, 57.5, 61.1, 61.4, 61.4, 61.3, 62.5, 61.5, 63.2, 63.2, 63.3, -57.6, -55.9, -53.9, -45.8, -43.2, -31.7, -27.4, -26.7, -25.8, -12.2, -7.7, -4, 7.2, 17.5, 31.4, 44.5, -31.6, -32.5, -33.4, -34.4, -31.3, -32.2, -33.1, -34, -34.9, -35.8, -28.4, -29.3, -30.2, -31.1, -32, -33, -33.9, -34.8, -35.7, -36.6, -37.5, -27, -28, -29, -29.9, -30.9, -31.9, -32.9, -34, -35, -36, -37, -38, -39.1, -40.1, -25.6, -26.7, -27.8, -28.9, -30, -31.1, -32.3, -33.4, -34.6, -35.7, -36.9, -38, -39.2, -24.7, -25.6, -26.5, -27.5, -28.5, -29.5, -30.5, -31.5, -32.5, -33.5, -34.5, -35.5, -36.6, -37.6, -38.6, -39.6, -40.6, -41.6, -42.6
			, -24.2, -25.1, -26, -26.9, -27.8, -28.7, -29.6, -30.5, -31.5, -32.4, -33.4, -34.3, -35.3, -36.2, -37.2, -38.1, -39, -40, -40.9, -41.8, -42.7, -43.5, -44.4, -23.5, -24.4, -25.4, -26.4, -27.4, -28.4, -29.5, -30.5, -31.6, -32.6, -33.7, -34.7, -35.8, -36.9, -37.9, -39, -40, -41.1, -42.1, -43.1, -44.1, -45.1, -21.6, -22.5, -23.4, -24.3, -25.3, -26.2, -27.2, -28.2, -29.2, -30.3, -31.3, -32.3, -33.4, -34.4, -35.5, -36.6, -37.6, -38.7, -39.8, -40.8, -41.9, -42.9, -44, -45, -46, -21.5, -22.5, -23.5, -24.5, -25.6, -26.7, -27.8, -29, -30.1, -31.3, -32.4, -33.6, -34.8, -36, -37.2, -38.4, -39.6, -40.8, -42, -43.2, -44.4, -45.6, -46.8, -47.9 };
		for (int i = 0; i < lLen; i++) LegLeo[i].x = LegLeoK[i] * zTbb;
		delete[]LegLeoK;
		LegLeoK = new float[lLen]{ 2.7, 4.4, -2.9, -0.3, 3.7, 11, -0.5, 3.2, 9, 4.1, 1.1, 1.8, -4.6, 3.1, 5, 12.2, 12, 7.8, -3.2, 6.7, -1.8, 6.4, 9.3, 4.3, -0.3, 4.2, -7, -0.5, 0.6, 7.5, 14.6, 6.1, -5.8, -4.3, -4.4, -4.5, -4.7, 9.2, 0.4, -0.1, 4.9, 2.9, -3, -1.1, 9, 15.8, -4.1, -8.4, -6.1, -5.8, -4.5, -5, -4.3, 6.7, -0.9, 4.3, 1.8, 3.4, -1.7, 11, 16.7, 15.9, -9.2, 12.9, -7.3, 11.2, -5.9, 8.7, -2.8, 8.5, 4.3, -1.3, 2.3, 4.1, -3.6, 12.8, 18.1, 16.8, 15.3, -10, -8.7, -7.1, 11.2, -5.3, -5.5, -4.1, -4, -0.5, -2.4, 3.9, -10.8, -6.1, 13.4, -10.8, 16.8, -10.7, 15.9, 14.6, -9.1, 12.9, 11.8, 10.2, -4.1, -3.3, 2.1, 1, -11.1, -0.2, -11.3, 18.5, -11.7, 16.2, 14.6, 14.6, -9.2, -8.6, 11.8, -6.4, -4.9, -5.8, -2.6, -2.7, 1.9, 0, -11.7, -6.8, 20.4, -12, -11.9, -11, 15.5, 14.3, -9.1, 12.1, -5.7, -5, 7.7, 2.8, 0.8, 20.8, 20.8, -7.8, -12.6, -12.6, -11.9, 17.1, 15.5, -9.4, -8, 12.3, 9.5, 7.6, 6.6, 3.6, 21.1, -8, 21, 19.8, 19.3, -12.3, -11.4, 16.4, 15, 13.3, -7.2
			, -5.6, 8.8, -2.9, 1.6, 1.6, -12.5, 16.5, 20.7, 20.3, -13.1, 18.7, 17.6, 16.4, -10.2, -9.1, -8.1, -6.6, -6, -4.6, 7.5, 6.4, 2.4, 21.6, -8.4, 21, 20.6, -13.6, -13.3, -12.4, -11.5, 16.3, 14.3, 14.5, 12.2, 11.9, -5.4, -3, -0.4, 4.2, 22, -8.6, -13.4, 20.8, -13.9, -13.4, -12.2, 17.2, -10.3, -9.6, 14, 13.1, 11.5, 10.5, -3, -0.5, 0.3, -13.4, 21.5, 20.8, -14.3, -13.9, -13.3, 17.7, -11.2, 16, 14.9, 13.4, -6.6, -5.2, -3.9, -2.3, 2.1, -13.3, 17.2, -10.1, -14.4, -14.4, -14.3, -13.8, -12.9, -12.3, 16.6, -9.7, -8.3, 12.8, 12.6, -5.6, -4, 9.1, 2, 0.1, -13.7, 17.6, 21.2, -14.5, 20.6, -14.2, -13.5, 18.3, 17.2, -11, -10, 13.8, -7.7, -6.3, 11.3, -3.2, -1.6, 0, 5.7, -14.1, 21.4, 21.2, 21, -14.8, -14.3, 19.9, -13.3, 18.1, -11.8, -10.7, 15.4, 14, 12.4, -6, -4.9, -3.4, -1.7, -0.1, 6.7, -10.1, 17.4, -14.5, -15, -15.2, 20.5, 20.1, -13.3, 18.6, -11.8, 17.1, 16.3, 14.4, 13.2, -6.2, 11, 7.7, 5.6, 1.6, -14.3, -10.3, 21.4, -15.3, -15.2, -14.8, 20.2, -13.8, -13.1, 18.6, 17.1, 16.3, 15.4, -9.3, 13.8, -6.5
			, 10.3, 8.9, 8.6, 6.5, 1.5, 2.3, 17.8, 21.3, -15.3, -15.4, 21.2, -14.7, -14.3, -13.7, -13.2, 19.2, -12.4, 17, -11.1, 16, -9, -8, -6.6, -6, -4.5, -2.9, 6.6, 3.8, -10.8, 21.5, 21.5, -15.3, -15.5, -15.1, 20.8, -13.9, -13.2, 19.1, 18.6, -11.8, 17, -10.2, 15.1, -8.2, -7.2, 11.9, -4.5, -3.8, 7.2, -0.6, 0.2, -14.9, 19, 19.4, -15, -15.5, 22, 21.6, -14.9, 21.1, -13.6, -13, 19.1, -12.2, -11.2, 16.3, 14.7, 14.6, -6.8, 10.7, -4.1, -3.1, 5, 1.1, 5.8, -10.8, 12.2, 21.9, 19.7, -15.8, 22.1, 21.9, -15.2, 21.3, -13.8, 20, 19.3, 18.2, -11.6, -10.6, 15, 14.4, -7.6, -6.9, 10.3, -3.8, -2.9, -0.4, 0.9, -15.4, 13.8, 18.7, -15.8, 19.7, 22.3, 22.5, 22.5, -15.5, 21.9, -14.5, -13.5, -13.1, -12.6, 18.1, -11.5, -10.8, 15.8, -9, -7.7, -6.5, -5.6, 8.4, -3.3, -1.8, -0.1, 5.5, -15.9, 16.7, 17.7, 19.2, -16.1, 22.6, -16.4, -15.8, 22.1, -15.1, -14.3, -13.6, 20.5, 19.4, 18.4, 17.7, 17.2, -10.1, 14.2, -8.5, 12.6, -5.6, -4.2, 7.7, -1.9, 5.5, 2.5, -15.8, 21, 21.9, 19.5, 20.2, -16.4, 22.9, 19.9, 23, 22.6, -15, -14.4
			, 21.2, 21.2, -12.9, 19.6, 18.6, 17.3, -11.1, -9.8, -9.2, -7.9, -7.2, -5.7, -4.3, 8.4, 7.4, 5.4, -16.3, -12.7, -17.1, -16.9, 23, -16.5, -16.3, -15.6, -15.1, -14.6, 21.8, -13.6, -13.2, 20.8, 20.2, -12.6, -12, 17.8, 17.2, -10.9, -9.9, -8.8, 12.5, -5.9, 10.8, 9.6, -2.9, -2, 1.2, 4.9, -16.9, 21.6, 21.1, 21.8, -17.1, -16.8, -16.5, -16.4, -15.8, 20, 23.1, 22.3, -13.3, -12.8, 20, -12.3, -12.1, -11.6, 16.6, 15.2, -9.5, 13.7, -7.4, -5.9, 9.5, 8.3, 7.1, -0.7, 3.8, -17.6, 20.9, 22, 22.7, -17.1, 23.2, 20.7, 20.4, -16.2, 23.4, -15.2, -14.1, 22.1, -13.2, 20.8, -12.4, 19.8, 19.4, -11.7, -11.2, 16, -9.8, 13.6, -8.2, 11.7, 11.1, -5.3, 9.2, 8, -1.6, 5.6, -17.3, -13.7, 17.9, 20.4, 22.1, -17.6, 20.6, -17.1, 21, -16.2, 24, 23.4, -14.1, -13.5, -12.7, 21.2, 20.2, 19.6, 19.1, -11.5, -10.8, 16.3, 14.8, 13.3, 11.9, -6.9, -4.7, -3.5, -2.4, 5.8, 1.7, 5.5, 4.3, -13.5, 19.5, 20, 20.6, -18.3, 23.6, -17.5, -17.3, -16.7, 24.1, -15.1, 23.3, -13.5, 22.1, 21.5, -12.6, -12.1, 19.6, -11.9, 17.3, -10.8, 14.9, 14.3, 12.8, 12.7, -5.4
			, 9.6, 7.7, -1.9, 0.6, 2.2, -14, 19.2, -18.5, 21.8, -18.5, 23.1, 20.6, -17.5, 20.7, -16.1, -15.2, 23.7, 22.9, -12.9, -12.7, -12.4, -12, 20, -11.9, -11.9, -11.5, 16.5, 15.8, 14.4, -8.6, 12.3, -6.4, -5, -3.5, -2, -0.5, 2.1, 3.8, 19.7, 20.6, 21.7, -18.9, 23.2, -17.8, 21.1, -16.7, -15.9, -14.9, -14.2, -13.6, -12.9, 21.9, -12.4, -12.4, -12.1, 20.6, 20, -12, -12, -11.3, 16.4, 15.8, -9.4, -8.7, -8, -6.6, 10, 9.7, -2.3, -1.3, 0.3, 1.9, -19.2, -14.7, 19, -19.4, 22, 23.4, -18.7, 24.3, 24.5, -16.7, -15.6, 24.1, 23.5, 23.4, 22.9, -12.7, 22.4, 22, -11.9, -11.9, -12.1, 18.4, -11.8, -11.3, -10.2, -9.1, -8, -6.6, 10.8, -4.4, -3.1, 6.8, 4.9, 4.7, 4.5, 16.8, 19.4, -19.6, 20.6, -19.3, -19.2, -18.5, 24.3, 24.7, -16, -14.8, 24.1, 23.7, -12.8, 22.7, -12, 21.9, -11.7, -11.9, -12.2, 18.3, -12, 16.3, 15.4, 14.1, -8.9, -6.7, 10.6, 10.6, 9.2, -1.8, -0.7, 0.7, -0.3, -19.1, 17.8, 20.2, 20.6, 22.9, 23.6, 24.1, 24.6, -17, 24.7, -15.3, -14.7, 24.3, -13.5, -12.7, -12.2, -11.8, -11.8, -11.6, -11.7, -12, 19.9, 18.3, 17.9
			, -11.5, -10.4, -9.8, 12.7, 11.2, -5.5, -4.8, 7.9, 7.7, 5.4, 0.5, 5.2, -19.2, -19.9, 20.1, 21.5, 23.3, -19.4, -18.7, 24.7, 24.7, -15.8, -14.8, -14.2, -13.4, 23.5, -11.9, 22.7, -11.6, 22.4, 21.7, 21.3, -12.2, -12.5, 17.8, 16.3, -10.7, -9.9, -9, -7.8, 11.6, -6.4, -5, -4.3, -2.9, -0.5, -0.6, 4, -16.2, -20.3, -20.4, 23.1, -19.4, -18.8, -17.3, 24.7, -15.4, -14.7, -14.1, -13.2, 23.6, -11.8, 23.2, -11.5, 22.6, 22.5, -11.9, 19.5, -12.7, -12.5, 17.4, 16, 13.8, 12.9, 12.7, 10.7, -5.1, 8.7, -2.6, -3, 4.3, 2.8, -16.4, 19.4, 20.2, 22.1, 21.7, 21.7, 24, 24.3, -18.2, 24.7, -15.7, -14.9, -14.2, 24.1, 23.9, -12, 23.5, 23.4, -11.3, 22.8, -11.6, 20.8, -12.7, 19.6, 18.5, 17.1, -10.8, -10.1, 13.6, 12.5, -7.4, 11, -6.2, -3.3, -2.5, 6.5, -0.4, 2.9, -0.2, 3.5, -16.4, 22.2, 22.9, 23.3, -19.9, -18.7, 24.5, -16.1, 24.7, -14.7, -14, -13.2, -12.4, -12, 23.8, -11.3, -11.2, 23, -11.8, -12.2, 19.9, -12.8, 18.3, -11.9, -11, -10.2, 13.8, 12.4, -7.2, -6.9, 9.4, -3.9, -3.4, -2.8, -2, 2, -19.6, -16.6, -21, 22.5, 22.9, 23.1, -19.4
			, -18.3, -17.4, 24.6, -14.9, 24.6, -13.7, -12.9, 24, -11.7, -11.2, -11.2, -10.9, -11, 23.1, -12.3, 20.2, 19, -12.3, 16.6, -11, 15, -9.5, 12, 11.9, -7.1, -5.1, 8.8, 7.6, -1.4, -1.4, 5.2, -20.1, -20.7, 20.2, 21.8, -21.4, -21.1, 21.5, -20.5, 24.4, -18.2, -17.3, -16.1, -15.4, -14.7, -13.8, 24.4, 24.1, -11.7, -11.1, -11, -10.7, -10.7, 23.2, 22.7, 21, -12.8, 19.7, 17.8, -11.7, 16, -10, -9.5, 11.8, -7.2, -6.6, 8.7, -4.5, -3.8, -2.3, -1.6, 3, -20.8, 19.8, 22.4, 22.7, 23.1, -20.8, 24.5, 24.8, 24.6, -16, -14.8, -14.2, 24.3, 24.2, -12.6, -11.6, -11.2, 24.3, 24.3, -10.5, -10.6, -11.3, 22.4, 21.3, -12.8, -12.7, 16.7, -11.5, -10.8, -10, 13.4, 11.9, 11.4, 10.5, -4.8, -4.7, -3.4, 5.4, 0.2, 1.2, 1.7, -17.2, -21.3, 22.3, 22.7, -21.6, -21.1, -21.1, -19.8, 24.6, 24.6, 24.3, 24.3, 24.2, 24.2, 24.1, -12.7, 24.1, -11.1, 24.3, 24.4, -10.2, -10.5, 23.7, -11.6, -11.9, 21.2, 20.7, 19.2, 18.6, -12.3, 16, -11.2, -10.7, -9.9, -9.3, 12.3, -7, -6.4, -6.3, -4, 6.2, 5.8, 4.2, 2.5, -17.5, -21.5, 21.6, -21.6, 23.2, 23.4, 23.8, -20.7
			, -19.1, -17.3, 24.3, -15.1, -13.9, -13.6, -13, 23.8, -12, -11, -10.7, -10.3, 24.4, 24.4, 24, -10.1, 22.7, 22.4, 21.3, -12.7, 19.2, -12.3, -11.5, 15.7, -10.2, -9.1, -8.5, 11, 9.5, 8.5, 7.8, -3.7, -1.2, 0.4, 3.2, -17.8, -17.7, 20.4, -21.7, -21.7, -21.9, 23.9, 24.6, -19, -18.1, 24.4, -15.5, 24.1, -13.9, 23.7, 23.5, 23.8, 23.7, 24, 24, 24.2, -9.8, 24.6, -9.6, -10.3, 23, 22, 20.3, -12.5, 17.1, -11.9, -11.4, -10.5, -9.7, -8.9, 11.6, -8, 10.2, 8.8, 7.5, 6.3, 5.2, 3.5, -21.4, -21.7, 21.3, 22.7, 23.2, 23.8, -20.9, 24.6, -18.3, 24.5, 24.1, 23.8, -13.7, -13.1, 23.5, 23.4, 23.7, -10.6, -10, 24.1, 24.3, -9.2, -9.4, -9.9, -10.2, 23.2, -11.9, 20.8, -12.3, -12.1, 17.2, -11.3, 15.4, -10, 12.9, 12.2, -8.2, 11.2, 8.8, 8.6, 7.3, 6.1, 4.6, 1.1, 3.7, 16.7, 21.6, 21.8, -21.9, -21.5, -21.7, 24, 24.3, -19.5, -18.1, -17.2, 24.1, -14.6, -13.5, 23.4, -12.7, 23.3, 23.1, 23.5, -10.4, 23.6, -9.6, 24.2, 24.6, -9, -9.2, 24.2, 23.6, -11.9, -12.1, 19.8, 18.4, -11.8, -11.5, -10.9, -10.5, -9.7, 13.7, -8.9, 11.1, -7.8, 9.5
			, -5.7, 5.5, -1.8, -1.9, 3.1, 2.5, -17.7, 16, -21.9, 22.9, 23.3, 23.8, -20.6, -19.3, 24.2, -16.5, 24, -14.4, -13.9, -12.6, -12.2, 22.9, 23, -11.2, -10.7, -9.8, 23.4, 23.9, 24.1, -8.4, 24.6, 24, 23.6, -11.6, -11.8, -12, -12.1, 17.6, -11.6, -11.3, -11.1, -10.3, 14.4, 12, -7.9, -8, 8.8, -5.9, -4.5, -3.8, -3, 1.6, 3.3, 17.3, -21.8, 23, 23.1, 24.1, -20.5, 24.1, -19.3, -17.7, 23.8, -14.4, -13.6, -13, -12.6, 22.8, 22.8, -11, 22.6, 23, -9.1, -8.8, 23.6, -8.5, -8, -8.7, -9.3, -9.5, 23.3, -11.2, -11.8, 19.6, -11.8, 17.7, 16.8, -11.3, -10.5, 13.4, 12.8, -8.8, -7.6, 9.5, 9, -6.2, -4.7, -3.1, -3.3, 3.2, 16.5, -17.5, 22.6, 23.1, 23.5, 24.1, -20.3, 23.8, 23.8, 23.7, -16.2, 23.4, -14.2, -12.9, -12.3, -11.7, -11.4, 22.1, 22.2, 22.2, -9.4, -8.9, 22.5, 23, 23.6, -8, -7.6, -8.2, -9.2, -10.4, 21.4, -11.8, -11.8, 18.2, 17.2, -11.5, 14.9, -10.7, -10, -9.1, 11.6, 10.1, -7.8, -6.4, 6.2, 5.3, -1.5, 3.3, 0.1, -21.4, -21.8, 21.7, 22.8, -21.6, -21.1, -20.6, -20, 24, -17.9, 23.5, 23.4, -14.6, 23.1, -12.6, -12
			, 22.3, -11.2, -10.7, 22.1, 21.9, 21.6, 21.9, -7.7, -7.5, 23.5, 23.8, 24.3, 24, -9.2, 23, -11.1, -11.4, 21, -11.6, 18, 16.9, -11.7, 16, -11.1, -10.4, -9.6, 12, 10.6, 9.2, 8.5, -6.8, -5.2, -3.6, -3.8, 2.6, -17.5, 21.6, 22.3, 23.3, -21.2, -20.7, 23.8, 23.9, -17.9, -16.7, -15.7, -15, -13.7, -12.4, -11.8, 22.1, 21.8, -10.5, 21.6, 21.3, 21.7, 21.1, 21.8, 21.7, 22.8, -7, -6.8, 23.8, 23.9, 23.6, -9.3, 22.3, -11.1, 20.5, 19.6, -11.6, 17.3, 16.5, -11.5, -10.9, -10, 12.5, 11.1, 10.6, 9.4, -7.8, 6.9, -4.8, -0.2, -1.3, 18.1, -21.2, 21.1, -21.6, -21.1, -21.2, -20.1, -18.9, 23.6, 23.5, -16.2, -14.8, 22.8, 22.3, 22.1, -11.6, 21.6, 21.3, -9.9, -9.3, 21.3, -8.7, -7.7, 20.9, -6.7, 21.5, 22.8, -6.1, 23.7, 23.9, 23.4, 22.9, -9.7, 21.6, -10.9, -11.2, -11.5, 18.2, -11.8, 16.4, -11.2, 14.3, 12.7, 11.7, 10.7, 10.4, 9.2, -8, 6.8, 6.6, -4.1, -5.9, -0.4, -0.6, -17.1, 20.1, 20.1, 22.1, -21.5, 23.6, 23.4, -20, 23.6, -18.8, 23.3, -16.7, -15.2, -14.2, -13.2, 22.2, -11.4, -10.9, -10.1, -9.2, -9, 20.7, -7.9, 20.1, 20, -6.2
			, 20.7, -5.9, 22.5, -5.7, 23.3, -6.5, -7.3, -9, 22.4, -10.2, -10.7, 20.8, -11.3, 19.1, 18.4, 17.7, -12, -11.9, 15, -11.4, 13.1, -10.1, 11, 9.4, -9, 6.8, 5.4, -5.4, 4.1, -1.6, 3, 0.1, -17.2, 19.6, -21, -21.4, 23.5, -20.5, -19.9, 23.4, 23, -16.5, 22.8, -14.2, 22.2, 21.9, -11.6, -10.5, -10, 20.5, 20.6, -8.3, -8, -7.4, -6.8, 19.7, 20.2, 20.4, 21, -5.2, -5.5, 22.9, 23.1, -6.7, -7.6, -8.5, 21.9, 21.5, -10.3, 20.5, -11.7, 17.8, -12.1, -12.3, -12, -11.8, -11.3, 11.8, 12.4, 10.1, 9.2, -8.6, 7.3, 5.1, 4.1, -4.6, 1.5, -4.8, -16.9, 20.4, -20.8, -21.3, -20.8, -20.4, -18.9, 23.2, -16.8, -16.2, -14.7, -13.5, 21.6, -11.4, 21.5, 21.2, 21, -9.2, -8.6, -7.7, -7, -6.6, 18.8, -5.6, 19, 19.1, -5.2, 20.2, 20.1, 22.3, -5.5, 22.3, -7.2, -8.5, -9.2, -9.5, -9.9, -10.7, -11.3, -11.7, -11.9, 17.4, -12.2, 15.8, 14.5, 14.1, -11.6, 11.4, 10.9, -9.1, -9.6, -8.1, -7.4, -7.5, -3, -4.1, 1.5, -16.7, 21.6, -15.8, 23.6, 23.5, 23.4, -18.7, -17.3, -16.1, 22.5, -13.8, -13.2, 21.3, -11, 21, 20.5, -8.7, -8.3, 19.3, -7
			, -6.5, -6.1, 18.6, 18.1, -4.5, 18.4, 18.8, 19, -3.3, 20.8, -3.8, 21.2, -5.5, 22, 22, -7.5, 21.4, 21, 20.5, 20.2, -11.3, 19.4, 18.8, -12.3, -12.4, -12.6, 15.5, -12.3, 13.2, 12.9, 12, -11.1, -10.4, 8, -7.9, -7.6, -7.7, -1.4, -6.2, 0.3, 21.8, -20.3, -20.8, 23.8, -19.9, -18.8, -17.7, -16.8, -15.8, -14.4, -13, 20.5, -9.5, -8.9, 20, 19.5, -7.2, 18.4, 18.7, -5.6, 18.2, 17.9, 17.9, 17.6, 18.3, -3.4, 18.5, -2.4, 20.5, 20.7, -4.3, 21.2, -6.2, -7.3, -7.8, -8.5, 20.7, 20.3, 19.6, 19.2, -11.3, -12.1, -12.7, 16.7, 16, 14.7, 13.6, 12.7, 10.6, 10.6, -10.7, -10, 7.7, 6.6, 4.7, 3.3, -5.3, 0.4, -2.7, -15.9, -16.2, -20.3, -20.4, -20, 23.3, -18.3, -16.7, 22.6, 22, -12.8, -11.8, -11, -10.6, -9.7, -10, 20.3, -8.8, -7.6, 18.7, 19, 17.7, -5.7, -5.2, -4.7, -3.9, 16.9, 17.2, -2.9, -3.1, -2.4, 17.1, 18.5, -2.4, -2.7, -3.2, -4.7, 20.6, 20.8, 20.9, 20.8, 20.6, -9, -10.1, 20, -11.2, 19.1, -12.2, 17.6, 17.2, -13.1, 15.8, -12.9, -12.6, -12.6, -12.2, 9, -11.5, -10.5, -9.7, 5.2, -6.5, -8.3, -2.7, -6.7, -2
			, -4.1, -16, -15.8, -20.3, -14.1, -19.9, -19.4, -18.7, 23, -16.7, -14.6, -13.7, -13, -12.5, -10.9, 20.3, -8.8, -8.4, 19.1, 18.6, 18.1, 17.3, 16.7, 16, -3.7, 16, -2.1, -1.1, -1.7, 16.2, -1.3, -1.4, 17.6, 17.3, 18.5, 19.2, 19.9, -4.4, -5.4, -5.8, -7.2, -7.7, -8.4, 20, 19.9, -11, -11.5, 19.1, -12.7, 17.1, -13.4, 16.2, 15.1, -12.9, 12.8, 11.3, -12.5, 9.9, 8.5, -11.4, -10, -9.3, -6.8, -7.8, -2.3, -20.3, -15.9, -14.9, -13.8, 24.1, 23.8, -18.9, 23.5, -17.2, -15.5, -15, 21.7, 21.1, -10.7, 20.1, 19.1, -7.4, -7.3, 18.8, -5.6, 17.4, -5.2, 16.3, 15.9, -3.1, -2.5, 14.6, 14.4, -1.6, 14.6, 14.3, 0.3, 0.1, 15.3, 1, -0.3, 17.2, 17.4, -1.2, 17.5, 17.5, -3.7, 19.4, 19.3, 19.7, -6.3, 19.9, 19.8, -8.6, -9.2, 19.7, 19.4, -12.3, 18.3, -13.4, -13.5, -13.7, 15.5, 13.9, -13.5, 12.9, -12.7, -12.8, -11.9, 7, 5.7, 2.7, -9.5, 3.5, -6.1, 0.9, -5.3, -0.7, -15.2, 25.3, -15, -14.1, -20.4, 24.1, -19.9, 23.7, -18.1, -17.4, -16.7, 22.7, -14.6, -13.2, 21.3, -11.6, -9.7, -9.6, -8.9, 19.2, -7, 18, 17.9, -5.1, 17.2, -4.2, -3.3
			, 14.9, 14.6, 13.2, 13, -0.8, -0.1, -0.1, 0.6, 2.5, 2.2, 3.2, 2.9, 14.6, 2, 2.9, 15.5, 16.8, 17.2, 16.5, -2.6, 17.7, -3.8, 19, -5.5, 19.4, 19.4, -8.6, 19.7, -10.6, 19.5, -12.4, 18.6, -13.4, 16.9, 16.5, 15.2, -13.7, -13.3, -13.1, 10.1, 8.3, -12.5, 5.6, 4.3, -9.9, -4.6, -8.4, -2.8, -3.9, -14.3, 25.2, -13.6, -13.1, -20.1, 24.4, 24, -18.5, -17, -15.8, 22.6, 21.9, 21.7, 21.2, 20.3, 20.4, 19.6, -7.8, -7.3, -6, -3.9, -4.2, 16.2, -3.1, 14.1, 13.7, 12.5, -0.8, -0.5, 0.7, 1.5, 9.7, 2.9, 3.9, 12, 5.3, 9.9, 11.5, 5.7, 4.5, 12, 12, 12.7, 14.1, 13.9, 13.6, 15, 15.7, 0.5, -0.3, -1.3, 17.3, -3.6, -5, -5.9, -6.9, 19, 19.2, -8.7, 19.5, 19.5, -11.8, 19.1, 18.5, 17.8, 17.3, -14.2, 16.6, -14.4, -14.2, 12.9, -13.7, 11.4, -13.3, 8.7, -12.1, 6.3, -9.2, 3.9, 1.1, -5, -6.1, -12.7, 25, -12.7, -12.2, 24.6, 24.5, -18.3, -17.6, -15.3, -13.2, -12.4, 21.6, -10.9, 20.3, -9.7, -8.1, -7, 17.7, -5.8, -5.1, -4.1, -4.4, -3.3, -2.2, -2.3, -1.1, -1.7, -1, 12.3, 10.6, 1.2, 9.7, 8.3, 3
			, 8.1, 4.5, 5.5, 8.5, 7.8, 7.3, 8, 7.8, 6.6, 12.2, 9.4, 4.3, 12.6, 12.4, 1.7, 1.4, 1.5, 15.9, 16.4, 17.3, -3.7, 17.5, 18.2, 18, 18.9, 18.8, -8, 19.3, -9.9, 19.3, -12.1, 19, 18.4, -13.8, -14.3, -14.6, 16.2, 14.6, -14.3, -14.4, -13.9, 11.2, -13.6, 8.5, -12.6, -11.6, -11.3, -12, 2.8, -2.3, -9.7, -5.2, 0.4, -18.7, -19.4, -19.2, -11.7, -11, 24.7, -19.2, -18.5, -17.1, -16.7, 23.1, 22.4, -12.9, -12, -10, -9.3, -8.8, -7.9, -6.9, -6.5, -6, 17.2, 17.1, -3.2, -1.8, 14.4, 14.2, 13.7, -0.3, 11.9, 1.1, 1.7, 9.1, 3.2, 7.8, 5, 6, 8, 6.7, 5.8, 11.1, 5.7, 3.1, 2.3, 14.3, 14.9, 15.4, 16.7, 17.5, 17.6, -3.6, -4.9, 18.1, -7.9, -8.2, 19, 19.3, 19.1, -12.7, -12.8, 18.5, 18, -14.7, 16.6, -14.7, 15, 13.8, -14.7, -14.4, 9.6, -13.3, 6.3, 5.8, 4, 2.5, -9.6, -10, -5.5, -4.7, -11.4, 25.8, -18.7, -19.1, -18.9, -9.9, 24.3, 24.1, -16.9, -14.9, -14.3, -13.2, -11.1, 21.4, 21, 19.6, -6.9, 18.9, 17.7, 17, -2.9, -2.6, 15.4, 15.1, -1, 13.6, 13.4, 12.3, 11.1, 10.7, 3.1, 9.5, 3.9
			, 5.6, 6.2, 6.4, 11, 9.8, 6.1, 11.5, 13.1, 1.7, 1.2, 15.2, 16.4, -2.6, 16.8, 17.5, -6.5, 18.4, 18.7, -8.9, -10.1, 19.1, -12, 19.4, -13.3, -13.8, -14.3, 18, -14.9, -15.2, 15, 14.2, 12.4, -15.2, 10.7, 9.4, 9.3, 7.8, -12.7, -13.3, -11.7, -11.8, -1.9, 1.9, -3.9, 0.9, -4, -17.8, -10, -10.6, -18.7, 25.5, 25.2, 24.7, -17.5, -15.8, -14.4, -12.6, -12.2, -11.3, 21.3, -8.9, -8.1, -7.9, -7.2, -6.7, -6.5, -5.3, 17.8, 16.8, 16.3, -2.3, -1.9, -0.9, 0, 13.2, 0.9, 11.6, 11.4, 10.8, 9.3, 4.4, 5.3, 6.2, 8.7, 6.8, 3.3, 10.3, 2.3, 2.4, 14.9, -0.6, -2.5, -2.8, -4.2, 17.7, -6.8, -7.3, 18.4, 18.9, -11.1, -12.4, 19.3, -13.2, -14.1, -14.2, 18.2, -14.8, 17.5, -15.2, -15.5, -15.8, -15.7, -15.7, -15.6, 11.3, 10, -13.9, 6.5, -12.6, 3, -10.3, -8.8, -4.1, -8.9, -4.2, -15.7, -9.5, -17.1, -10.1, -17.5, -17.8, -17.4, 24.5, -16.3, -14.9, -13.9, 22.8, -11, -10.9, -9.5, 20.2, -6.3, -6, -5.1, 18.3, 17.5, -2.8, 16.7, 15.4, 15, 13.7, 13.7, 12.3, 12.3, 11.1, 4.2, 9.9, 8.8, 5.9, 6.9, 9.1, 8.2, 2.1, 11.6, 12.4
			, 13.6, -0.5, 15.1, -2, 15.6, 17, 17.3, -7.3, 18.3, -9.5, 18.7, 18.8, -12.6, 19.5, -13.8, -14.4, 18.5, 17.5, 17.3, 16.4, 14.6, -16.1, 14, 12.6, -15.9, 10.9, -14.9, -15.1, 7.1, 4.6, -12.1, 3.7, 1.6, -9.4, -1.5, -8.2, -1.7, -8.7, -15.7, -8.5, -16.4, -16.6, -17.3, -15.6, -15, 24.3, -12.9, 22.8, -10.6, -8.9, -8.2, -7.8, -7, -5.5, 19.2, 18, -3.4, -2.3, -1.3, 15.8, -0.3, 0, 1, 1.4, 11.9, 11.7, 11.4, 4.8, 9.5, 6.2, 7.4, 5.4, 3.8, 9.6, 2.7, 2.6, -0.7, -0.1, -3.2, -4.6, -4.7, -5, 17.9, 18.1, 18.6, -10.8, 18.9, 19.1, -12.9, -13.9, -14.8, 18.5, 18, 17.5, -16.2, 15.4, -16.1, -16.5, 12.3, -16.3, -15.9, 9.3, 7.9, 5.8, -14.8, 5.5, 3.2, 1.3, -9.1, -2.8, -7.5, -1, -8.2, -8.2, -8, -15.3, -16.6, -6.9, -4.5, -16, 24.6, -14.1, -13.2, 23, -10.9, -10.2, 21.3, -7.2, -5.9, -5.1, 18.9, -3.8, -2.5, -1.8, 16.6, -0.5, 0, 0.9, 1.4, 2.3, 2.8, 3.6, 10.2, 5.3, 6, 7.9, 8.4, 5.5, 10.1, 10.8, 2.3, 11.4, 14, 13.8, 15.1, -4.9, 16.2, -6.3, -7.1, -8.5, 18.4, 18.6, 19.1, 19.5, -13.6
			, -14.9, -15.6, 18.6, 18.3, -16.5, -16.6, -16.5, 14.9, 13.3, -17, 11.9, -16.6, 9.1, 7.9, 5.9, 4.5, 3.2, -10.3, -12.2, -6.8, -1.1, -7.1, -0.9, 25.2, 25.3, -6.8, 25, -14.9, 25, -5.7, -14.7, -4.3, 24.8, 23.9, 23.4, 22.7, -9, 22.2, -7, -5.1, -4.5, 20.3, -4, -2.5, 17.6, 17.4, 16.1, 15.8, 0.5, 1.3, 1.9, 2.7, 3.2, 3.9, 9.8, 8.7, 6.4, 7.4, 5.2, 2.1, 7.9, 0.9, 11.4, 1.8, -1.4, 14.2, -1.7, -4.8, -6.3, -7.8, 17.9, -10.6, -11.8, 18.9, 19.2, 19.7, 19.6, -16, 18.3, 16.9, 16.6, 15, -17.3, -17, 11.6, 10.5, 10, 8.7, 8.5, -15.2, 3, -14.7, -13.9, 1.2, -10.3, -2.3, -8.7, -3.3, -12.7, -5.4, 26.5, -13.5, -5.5, -4, 24.8, -14.7, -13.4, -11.4, -10.4, -9.7, -8.8, -7.4, -6.5, 20.8, -3.7, -2.7, 18.5, 18.1, -0.8, 16.8, 15.1, 15, 13.7, 13.7, 12.6, 4.8, 10.2, 5.3, 9.1, 7.9, 3.8, 8.6, 4.6, 1.4, 13.5, 14.2, -3.1, -4.9, -6.1, -7.9, 17.5, -9.6, 18.3, 19, 19.3, 19.5, 19.6, -16.1, -16.8, -17, -17.2, 16.1, 15.6, 14.2, 13.3, -17.1, 11.4, -16.6, -16.1, 7, -14.4, -14.9, -14.3, 0.5, -9.9
			, -2.5, -11, -5.5, -9.4, -12.7, 24.2, -4.5, 24.4, -12.9, -3.4, -13.2, 24.3, -12.4, 23.8, -9.2, -7.4, -6.8, 21.9, 21.6, -4.6, 20.1, 19, -1.1, -0.4, -0.1, 0.5, 1.6, 1.7, 2.3, 13.5, 3.6, 12, 5.1, 6, 9.5, 8.6, 2.7, 10.9, 5.3, 10.9, 10.7, 11.5, -1.9, 14.8, 15.3, 16, 16.4, 17.2, 17.8, -11, -11.9, -12.9, -14, -15.2, 19.8, 19, -17, -17.3, -17.7, 17, 15.5, 14.7, -18.1, -17.7, -18.1, 9.5, 9.3, -16.4, 5.5, -14.2, -12.7, -13.7, -1.8, -10.3, -5.7, -2, -4.6, 23.8, -3.3, -3.6, 24.5, -12, 24.4, 24.2, -12, -11.1, 22.9, -9, 22.3, -5.1, -3.7, -2.7, -2.3, -1.1, 0.3, 0.9, 17.4, 16.2, 14.9, 14.5, 4.2, 4.7, 11.7, 10.5, 10.7, 8.4, 5.3, 3, 8.8, 0.3, -0.1, 14.4, 13.6, -4.7, -6.7, 16.2, 17.4, 17.7, -11.3, -12.7, -13.7, -15, -15.5, 19.8, -16.2, -16.9, -17.1, -17.4, -17.8, -18.1, -18.2, -18.4, 14.6, -18.1, 11.7, -18.1, 9.3, 8.9, -16.6, 5.5, -15.4, 5.2, 0.8, -11.3, -1.2, -8.8, -2.2, -10.7, 24.4, -11.6, -2.7, -1.8, 24.1, 24.1, 24.1, -10.8, -10.5, -9.5, -7.6, 22.5, 22.1, -5.3, 21.5, -3.1, -1.7
			, -0.9, 0.5, 18, 2.6, 16.8, 4.2, 4.4, 4.9, 5.4, 12.2, 6.1, 7.9, 10, 7.8, 4.8, 0.9, 12.3, 11.8, 13.4, 14.1, 15.5, -7.3, 17.2, -10.3, -11.6, -12.8, 19.5, 19.6, 20.2, 20.3, 20.1, 19.5, -17.4, -17.7, -18.3, 16.5, 15.9, 15.6, 13.9, 12.8, -18.6, -18.3, -17.2, -17.8, 5.2, 3.1, -14.8, 1.7, -11.5, -0.4, -9, -3.9, -5.5, 23.9, 23.7, 23.7, -0.8, 23.8, -9.9, -9.8, -8.8, -8.1, -6.2, -4.4, -3.4, 20.7, -0.9, -1.1, 0.3, 1.2, 18.7, 3.2, 4, 16.8, 5.6, 6.5, 13.7, 12.7, 8.6, 10.7, 9.3, 7.4, 1.6, -1.5, 12.5, 13.9, -5.7, 15.3, -7.6, -8.7, -10.5, 18.1, 19, -14, 20.1, 20.7, 20.4, -17.2, -17.8, 18.7, 18.1, 16.5, 16.9, 16, 14.9, 12.7, -18.7, -18.3, -18.5, -18.6, 7.1, 6, -14.9, -1.5, -12.7, -1.5, -10.2, -3.9, -9.5, -8.9, -8.1, -2.9, -9.8, 23.9, 23.6, 23.4, -7.8, 22.9, 22.3, -4.6, 21, -2.3, -0.2, 19.4, 2.2, 18.9, 17.6, 5.2, 6.1, 16.1, 7.7, 8.2, 10, 11, 6.4, 2.5, 9, -1.1, -1.9, 12.9, -5, -6.2, 15.7, -7.8, -10.3, 17.9, 18.7, -13.5, 19.7, -15.5, 20.6, -17.1, 19.6, -18.5, -18.9
			, 17.2, -19.6, 14.8, 14.3, 12.4, -19.4, 10.5, -18.4, 7.4, 6.9, 4.1, -16.2, -15.3, -16.3, 1.2, -0.8, -12.3, -7.7, -10.6, -5, 23.9, 23.7, -8.2, -8.1, -7.6, -7.3, -6.4, -5.2, -4.1, -3.2, -1.9, 20.5, 19.8, 1, 2.6, 4, 18, 17.7, 6.8, 8.2, 9.1, 11.7, 13.6, 2.4, 7.2, 2.2, 10.6, 10.4, 12, 14.5, 15, 16.1, 16.8, -10.6, 17.9, 18.6, 19.1, 19.8, -15.9, -16.3, 20.2, 19.6, -19, 18.1, -19.8, 15.4, 15.9, 13.2, -20.1, -20, 11.5, -19, 7.5, -16.9, 5.5, 4.2, -14.9, 1.6, -12.3, -12.5, -6.9, -2.2, -6.6, 24.3, -8, 23.6, 23.2, 23.1, 22.3, -5, -3.9, 21, -1.7, 0.2, 0.5, 19.5, 3.1, 4.5, 17.7, 6.3, 7.9, 8.9, 15.2, 11.2, 13.1, 5, 1.1, 7.6, -1.8, 12, 12.7, 12.5, 14.8, 15.3, 16.6, -10.3, -11.7, 18.5, 19.6, -14.9, 20.1, 21, 20.3, 19.5, 19.1, -19.6, -19.8, -20.3, 15.8, 14.9, -20.5, 11.7, -20.1, 9.8, -18.5, 7.4, 5.2, -17.5, -15.9, 0.8, -12.6, -1.3, -7.2, -6.4, 24.4, -7.1, 23.6, -6.7, -5.9, -5.3, -4.9, -4.1, -3, -2, 20.1, 1.7, 2.9, 5.1, 6, 17.9, 8.4, 9.4, 11, 13.5, 15.4
			, 2.9, 7.5, 1.7, -1.3, -2.5, -4.7, 13.8, 13.8, 14.6, 16.3, -10.9, 17.9, 19, -14.1, -15.2, 21, 20.9, 20.2, 20, 19.1, 18.3, -20.2, 17.8, 16.9, 15.5, 14.9, 13.6, 11.6, 11.1, 9.5, 6.2, -17.8, -16.9, -16, -13.6, -0.5, -10.9, -4.4, -10.2, -5, 24.8, 23.9, 23.6, -5.9, -4.3, 22, 21.7, -2.5, -1.5, 20.3, 0.8, 18.9, 3.1, 5.4, 7.3, 8.7, 17, 11.4, 12.3, 13.3, 3.7, 4.5, 9.1, 2.5, -1.6, -4.6, -4.8, 14.5, 15.1, -9.7, 17.8, 18.9, -14.1, -15.1, -16.2, -17.3, -18.4, 20.5, 19.6, -20, 18.3, -20.9, 16.7, -21.1, 14.8, 13.4, 11.4, -20.6, 9.4, 7, 4.8, -16.4, -15.3, -13.5, -11.8, -5.7, -9.5, -3.3, 25.1, -3.6, -5, -4.1, -4.8, -5, 22, -1.8, -1.2, -0.6, 19.6, 1.7, 2.2, 4.3, 5.2, 6.8, 7.7, 9.3, 11.1, 11.9, 13.5, 14.5, 0.5, 5.2, 9.8, 9.7, 12.2, 12, 12.7, 13.5, -8.8, 16.7, -11.6, 17.5, -13.7, 19.9, -16, 20.8, 21.4, -18.4, -19.1, -19.5, -20.1, -20.5, 18.3, 17.2, 15.8, 14.8, 13.5, -21.7, -21.3, 9.4, 6.6, -19.3, -17.2, -16.5, 1.4, -0.6, -13.8, -14.1, -9.7, -3.5, -9.7, -1.9, 25.2, 25.3, -1.5
			, 23.5, 22.8, 22.5, -2.2, -0.9, -0.8, 0.6, 1.5, 3, 3.8, 18.2, 5.7, 16.6, 8.2, 9.6, 11.5, 13.3, 13.8, 3.3, 2.2, 7.8, 1.1, -2.4, -5, 14, 13.3, 15.5, 16.8, 17.2, -12.6, 18.9, -14.8, -16, 21, 21.9, -18.7, -19.9, -20.5, 18.9, -21.1, 17.2, 16.8, 15.7, 14.1, -22, -22, 9.3, -20.3, -19.8, -19.3, -17.4, 2.1, -16, -2.1, -9.1, -4, -10, -6.3, 23.2, -0.3, -0.8, -2.2, -1.6, 21.5, 0.3, 19.8, 18.9, 18, 17, 6.7, 7.8, 14.8, 11, 13.9, 2.1, 7.7, 3.9, 8.4, 10.1, 10.9, 13.3, 13.4, 15.4, -10.3, -11.6, -13, 19.2, -14.7, 20.5, 21.4, 21.6, -18.7, 20.9, -20.4, -21.2, 17.7, 17, 15.5, 16, 13.6, -22.2, 10.3, 8.8, -20.3, 4.2, -18.3, -17.4, -17, 2.7, -2.3, -14.7, -1.6, -12, -3.5, -12.2, -7.5, -7.7, 3, 22.5, 23, -0.2, -0.8, 23, 21.8, 0.4, 0.8, 2.1, 18.8, 17.6, 4.5, 6.1, 15.8, 8.7, 11.6, 12.6, 1.9, 5.5, 2.7, -3.2, 10.8, -2.3, 12.3, 13, 15.2, 16.1, 17.7, -13.1, -14.7, 20.4, 21.2, 21.6, 21.8, 21.8, 20.2, 19.7, -21.2, 18.8, 18.2, 16.9, 15.6, -22.5, 12.6, -21.6, 9.1, 8.2
			, -20.5, 6, -18.6, 1.7, -0.3, -3.7, -15.7, -3.7, -6.7, 5.1, 8.1, 22, 6.9, 3.9, 1.5, 1, 1.4, 2, 2.9, 19.2, 18, 16.8, 5.7, 7.3, 14.7, 10.9, 3.7, 3.6, 7.2, 10, 10.5, 11.3, 13.6, -9.2, -9.8, -11.8, 17.8, 19.1, 19.6, -16.4, 21.4, 21.6, 22.1, 22, 21.3, 20, -21.6, -21.9, 18, 16.5, 15, -22.6, 12.4, 11, 9, 8.4, 6.1, 5.8, 3.6, 3.3, 1.2, -0.2, -1.9, -11.6, -6, -11, -6.1, 12, 20.5, 19.7, 21.1, 7.8, 21.6, 7.6, 3.6, 22.7, 2.9, 21.3, 19.9, 4.3, 5.2, 6.1, 8.5, 12.2, 12.4, 10.3, -0.5, 6.1, -2.7, 8.7, -4.9, 11.9, 14.8, 15.6, 16.3, 17, 18.6, 19.6, -15.1, -16.7, 21.6, -18.5, -19, 21.7, -21.2, 19.7, -22.7, -23, -23.3, 15.3, 13.6, 12.2, -21.8, -21.4, 8.2, -20.2, -19, 4.2, 3.3, 2.1, -0.6, -3.2, -14.4, -4.3, -12.8, -6.3, -6.7, 14.1, 16.9, 18.6, 20.1, 21, 6.8, 4.8, 4.3, 4.3, 4.5, 4.7, 18.8, 17.6, 17.1, 15.4, 10.6, 12.5, 3.3, -1.6, 5, -3.8, 10.3, 11, 12.6, 13.3, 14.8, 16.2, 17.5, -13.5, 19.5, -16, -16.8, 21.8, 22.5, 21.8, 20.6, -21.8, -22.6
			, 17.7, 16.2, 14.7, -22.3, 11.6, 11.3, -21.7, 8.9, -21.2, -19.1, -17.7, -17, -15.5, -13.9, -3.5, -13.2, -8.5, -8.7, 16.8, 18.5, 20, 11.7, 21.3, 5.6, 20.6, 20.5, 6.8, 16.9, 8.4, 14.7, 11, 2.1, 7.7, 2.9, -3, 9.2, 12.2, 12.4, 14.3, 14.6, 16.8, -12.9, 18.2, 19, 20.7, -16.3, 21.7, 22.4, 22.1, -21.1, -21.7, 20.8, -22.5, -22.9, 17.5, 15.9, 15.6, 14.3, 12.5, 11.1, 9, -21.4, 7.5, 7.2, -20.3, -19.3, -18.8, 3.7, 0.6, -17, -17.4, -0.8, -6.3, -14.2, -9.8, -5.9, 13.4, 11.8, 11.4, 19.9, 20.4, 7.6, 19.2, 7.8, 16, 15, 10.3, 14, 1.9, 7.5, -0.2, 9.8, 11.2, 12.4, 13.1, 13.7, 15.4, 15.9, 17.3, -13.9, 19.2, 20.7, 21.4, -18.3, -19.2, -20.5, 21.9, 21.4, -22.2, -22.8, 18.5, 17.3, -23.1, 15.4, 13.7, 12.4, -22.6, -22.1, 9.6, -21.5, -21.1, 5.3, -19.8, 0.2, -17.5, -16.8, -1, -14.3, -3.1, -10.8, -5.2, 15.7, 13.7, 19.9, 20.4, 7.8, 18.7, 8.6, 10.4, 14.1, -1.3, 4.4, -2.3, 9.8, 11.2, 11.3, 13.6, 14.4, -11.4, -11.4, 17.9, 18.7, 19.8, 20, -17.3, 21.5, 22.8, -21.1, -21.3, 22.4, -22.3, -23, -23.2, -23.1, 16.1
			, -23.2, -22.9, 13.5, 12.1, 11.7, 10.4, 9.1, 6.7, -20.1, 5.2, -19.3, -0.1, -16.2, -16.4, -7.3, -3.5, -11, -6.5, -7.5, 13.6, 10.6, 19.7, 8.9, 8.9, 9.2, 9.8, 13.3, 1.5, -0.5, 6.1, -3.4, 8.7, 11.9, -5.9, -8.1, 15.7, 17.2, -13.1, 19.2, -15.9, 21.4, 21.7, 22.7, 22.8, 22.7, 22.5, 21, -22, -22.8, 18.9, -23.1, 16.6, 15, -23.6, -23.3, 13, 12, -22.6, 9.5, 8.8, 8.6, 7.3, -20.3, 2.1, -18.7, 0.6, -15.5, -15.7, -14.9, -8.5, -11.5, 14.5, 11.3, 18, 19.1, 18.3, 16.8, 15.5, 12.6, 4.5, -0.7, 8.6, -1.9, -3.9, -7, 11.8, 14.2, -10.7, -11.7, 18.5, 18.5, -14.8, 20.4, 21.6, 22.5, -19.1, 22.9, 22.2, -22.3, -22.7, -22.9, -23, 17.5, 15, 14.9, -23.8, -23.1, -23.4, 10.9, -22.4, 8.2, -21, -20.3, 2.6, 1.4, -18.2, -16.6, -1.7, -5.8, -14.3, -9.7, -4, -7.1, 17.8, 17.6, 17.3, 10.3, 15.3, 12.9, 4.9, 0, 7.4, -3.2, -6.3, -7.2, -8.4, -10.1, 16.3, -12.2, -13.8, -16.1, -16.6, -17.4, 22.3, -19.3, -20.3, -21.8, -22.2, 20.4, -22.6, 19.2, 18, 17.9, 16.7, 15.6, -24.2, -23.8, 12.9, -23.7, 10.7, -22.6, -21.5, -22, 6.6, 5.5
			, 4.1, 2.6, -16.8, 2, -1, -15.2, -1.2, -10.9, -12.8, -7.3, 12, 15.8, 16.5, 16.1, 12.9, 15, 0, 6.5, -3.6, 9.1, 9.8, -5.3, -7.6, -9.7, 15.4, -11.4, -13.5, 18.9, 20.2, 20.1, 22.1, -18.9, 22.8, 22.5, -22.1, 20.4, 19.6, 18.9, 18.7, 17.3, -24.3, -24.3, 14.5, -24, 13, 10.7, -23.2, -22.2, 7.1, -20.4, 4.1, -17.9, 1.6, -2.4, -14.6, -2.6, -12, -6.4, -9.4, 13.3, -1.2, 3.5, 7.1, 9.8, -4.5, -7.3, -8.5, 15.3, 16, 16, 18.5, -15.1, -15.9, -16.6, -18, 22.4, 23, -21.1, -22, 20.6, 20.6, 20.1, -23.3, -23.8, 16.8, -24.5, 14.8, 14.4, -24.4, -24.1, -23.7, -23.6, -22.5, -21.1, 7.1, 4.9, 4.8, 3.6, 1.5, -0.6, -6.6, -2.9, -14.1, -9.5, -4.8, -5, 6.2, 1.9, 8.4, -2.4, -5.8, 11.1, -6.9, 15.8, 15.8, -12.2, 18.8, 20, -16.2, -18.2, 22.5, -19.7, 22.4, -21.2, -22, 20.3, 19.6, 18.1, -24.6, -24.6, 15.8, -24.9, 13.5, -24.4, 12, 10.2, 9.5, 9.1, -22, -21.3, -20.6, 5.5, 3.4, 1.2, -1.8, -15.1, -3.9, -12.5, -5.9, -9.9, 4.1, -1.7, 5.8, -4.9, 11, 11.8, 13.9, 15, 15.8, 16.4, 18.9, 19.9, 20.6, 21.4, 21.7
			, -20.2, -20.5, 21.6, 21.7, -22.5, 20.6, 20.1, 19.4, -24.3, 16.9, -25, 15.3, 14.2, 13.5, 12.1, 10.6, -23.3, -22.8, 7.6, 5.4, 5.2, 4.1, 1, -3.1, -14.4, -3.3, -13.4, -7.2, -13.8, -7.4, 3, 6.5, -1.6, -4.9, 11.8, 12.4, -9.5, -10.7, 16.2, 17.8, 18.5, -15.7, -17, -17.8, 22, 21.9, -20.4, -20.9, 21.3, -22.3, 20.4, -22.4, 20, 19.4, 17.8, 17, -25.3, -25.2, -25.4, 13.4, -24.9, 11.4, 11.3, 8.8, -23, 8.5, 7.4, -21.1, 4.9, 2.9, -17.8, 1.5, -1.4, -15.4, -2.7, -13, -6.4, -11.3, 3, 5.6, 5.5, -3.1, 10.2, 11.6, -6.3, -7.4, -9.7, -10.8, -12.1, 18.4, 19.7, 20.2, -17.5, -19.1, -19.5, -21, -22.2, -23, -24, 18.6, -24.9, -25.3, 16.3, -25.5, 14.4, 14.1, -25.2, -24.5, -24.1, -24, 7.9, 7.1, -22, 2.6, 0.3, -18.4, 1.2, -5.7, -1.8, -12.4, -5.8, -12.5, -6, 4.6, 3.5, 8.2, -3.4, -5.4, 12.4, 12.9, -9.9, 15.1, 17.4, -14.6, 19.6, 20.8, 21.3, 21.3, -18.8, -20.9, -22.5, -23.1, -24.2, -25, 17.7, -25.7, 15.6, 15.1, -25.4, 12.1, -24.7, 9.9, 10.4, -22.9, 6.9, 6.6, -19.6, -18.6, -17.5, -17.8, -2, -13.5, -7.9, -9.9, 1.6
			, 7.1, -1.1, -5.5, -6.6, 12, -8.9, 13.6, 15, 16.5, 18.8, 19.3, 20, -17.8, 21.7, -19, 21.7, 21.2, 21.2, 20.6, -24.5, -25.3, 18.6, 17.7, 16.2, 15.2, 14.6, -25.3, -24.8, 11.5, -24.4, 7.7, 7.5, 6.2, 5.2, 3.7, 1.9, -17.8, -0.2, -15.4, -2.5, 4.1, -3.6, 8.5, 10.2, -7.9, -9, -10.1, -11.3, 15.7, -15, 18.7, 19, 20.3, 20.2, -18.7, -20.3, 21.3, 21.2, 20.6, -25.4, 18.5, 17.7, -26.4, 16.8, 16, 15.5, 14.1, -25.4, -24.9, 10.1, 10.7, -22.9, 7.3, -20.7, -20.1, -19.7, 1.3, -2.8, 2, 7.7, 8.3, -7.7, 11.6, -10.6, 14.7, -13.8, 16.9, -16, -16.7, 19.7, -17.9, -19.5, -21.8, -24.1, -24.9, 19.1, 18.5, 18, -26.4, -26.6, 15.3, 14.5, 13, 11.1, -24.4, 8.2, 7.4, -22.7, -23, 4.6, 3.9, -2.9, 7.4, -4, -7.1, 12.4, -10.4, 13.9, 15.5, -14.1, -14.1, 16.6, 18.5, 19.6, 21.1, -18.7, -20.9, -23.1, -24.6, 19.5, -26, 17.3, -26.7, -27.1, -26.8, 14.5, 14.3, 12, -25.1, 9.6, -24.5, -23, -2, 3.6, -5.2, 8, 10.6, 12.2, 12.9, 13.7, 15.9, -13.2, -14.6, 18.5, 20.1, -19.4, -21.2, -22.9, -24, -25.3, 19.2, 18.8, 18.3, 18.1, 17.2
			, 15.9, -26.7, -26.2, -26, -25.9, -24.2, 1.7, -3.2, 1.5, 8, 7.8, 11.3, 12, -10.9, 14.9, -14.4, -14.7, -14.8, 19.3, -17.6, -18.7, -21.4, -24.1, -25.6, 19.4, -27, 17.8, -27.4, 15.9, -27, -26.5, 13.3, -2.3, 5.3, -1.5, 7.8, -4.6, 11, -8.8, -11, 13.2, 13.8, -13.6, 14.4, 17.3, 17.5, -16.5, -17.5, -19.8, -21.8, -23.7, -25.7, 19.4, -27, -27.5, -27.5, -27.4, -27.1, 13.6, 14.3, 2.3, 1.3, 6.8, 4.8, -6.8, -7.9, 11.5, 11.3, -10.5, 13.6, -12.7, 15.9, -15.5, 19.4, 18.8, -18.3, -21.6, -21.9, -24.2, -26.3, 19.3, 18.7, 18.3, 16.9, 16.8, -27.4, 15.4, 0.1, -3.5, 2.9, 8.2, -3.2, 8.9, 8.8, -7.4, 10.5, 10.3, -10.7, -12.2, -14, -14.9, 16.5, -16.4, 20.3, -20, -22.7, -25.1, -27.3, 18.9, 18.2, 18, -27.9, 15.5, -1.1, 3.2, -3.2, 7.1, -4.7, -7.4, 9.3, -10.7, 12.5, 14, 16, 17.1, 17.9, 19.9, -18.6, -20.5, -22.9, -24.6, -27, 19.2, -28.3, 17.7, 16.6, 0.5, 4.1, -1.5, 6.7, -7.6, -9.7, 10, 12.4, -13.3, 15.4, 16.1, 16.9, -18.7, -20.7, -22.2, -24.7, -27.4, -27.9, -28.4, -28.7, 1.3, -2.7, 4.9, 5.7, -5.7, -8.8, 9.8, -11.2, -11.2
			, -14.4, 16.4, -16, 19.1, -18.5, -20.8, -23.2, -25.7, -27.6, -28.2, 3.9, 2, 5.5, -2.9, -7.9, -9.1, 10.5, -12.3, 12, -13.7, -15.8, 17.5, 18.9, -19.1, -22.4, -23.1, -25.9, -27.6, -28.2, -28.5, 3.8, -1.1, 6.4, 7.9, -9.1, 11.2, -10.6, -12.7, 14.8, -16.3, 17.3, 19.3, -18.4, -19.8, -22.4, -24.8, -27, -28.1, -28.7, -0.2, -5.1, 4.3, 7.8, -7.3, -10.6, 11.5, 12.6, 13.2, -15.3, 16.3, 18.1, -19.6, -21.3, -22.9, -25, -26.8, -28, -0.3, -1.4, 4.9, 4.9, -6.7, -9.7, -10.7, 13.1, -13, 14.6, 16.3, -16.9, -19.6, -20.9, -23.3, -25, -25.7, -27.2, -28.3, -0.5, -4.6, 2.1, 4.7, 5.6, 8.8, 10.4, 12, 13, 13.7, -15.7, -16.8, -18.3, -20.4, -23.2, -24.3, -26.4, -28.1, -2.6, 2.1, -3.7, -6.2, -8.9, 9.3, 11, 11.2, -14.1, 13.5, 15.1, 16.7, -19.7, -21.5, -23.9, -26.2, -28, 0.1, 0, 3.5, 6.2, -7.4, -10, 9.6, 10.2, -12.5, -15, 16.6, 17.4, -18.8, -20.4, -22.4, -24.9, -27.7, -0.1, -2.1, 3.5, -6.1, 6.9, -10.3, 9.2, -13.6, 12.4, 15.6, 15.7, -17.5, -20.1, -22.1, -22.5, -25.1, -26.7, -0.3, -5.1, -2.1, -9.4, 5.8, 8.3, 9.1, 10.6, -12.8, -15.1
			, 14.7, 17.2, -19, -20.8, -23.9, -26, -3.3, 3.2, -3.5, 6.6, 7.5, 9, -9.7, -13.7, -14.1, 13.7, 16.2, -19.1, -20.8, -22.5, -23.8, -26.4, -4.5, 2, -5.6, 5.5, 8.5, -10.2, 11.3, 12, 13.7, 15.7, 16.9, -19.3, -22.1, -24.6, -0.8, -2.9, 3.7, -5.8, -9.1, -11.2, -13.1, 11.1, -15.5, -16.6, -17.8, -20.5, -22.6, -24.6, -1, 3.6, -3, 6.2, 7.8, 7.7, 10.9, -14.2, 12.5, 15, 16.9, -19.7, -21.6, -22.9, -3.1, 2.4, -5.9, -10.1, 7.6, 8.4, 10.8, 11.6, 13.2, 14, -17.3, -20.5, -23.2, -2.3, -4.3, 1.3, -8.2, 5.7, 6.5, -13.2, 10.6, 13.7, 13.9, 14.6, -18.8, -21.6, -4.3, 1.4, -6.4, 4.8, -8.5, -11.5, 9, 9.7, -15, 14.5, -17.7, -20.2, -3.6, 2.9, -8.4, 5.5, 6.2, -11, -13, -14.1, -15.3, 13.5, 15.2, -19.8, -21.9, 0.1, -3.8, 3.6, 5.2, -9.9, 7.6, -14.3, 10.2, 13.4, 15.1, -20.5, -4, 2.6, 2.4, -7, -10.1, 8.4, 10.1, 10.8, 12.4, -16.9, -6, 0.6, -7.1, 4.1, 5.7, -11.3, 9, 10.7, -16.9, 13.1, -18.1, -3.2, -5.3, 0.4, -9.2, 6, -9.3, -13.9, -14.8, 12.1, -17.2, -1.5, -8.3, -4.4, 3.7, 5.4, 7.1, -13.8, -15.9
			, 11.9, 12.9, -2.6, -4.8, 1.9, -6.5, -9.6, 7, 6.8, -1.8, -6.7, 1.7, -10.6, 5.2, 5.9, -3.8, -3, 1.6, -7.9, -10.9, -4.9, 19.4, 16.2, 21.1, 15.4, 15.6, 18.9, 22, 19.7, 19.7, 19.9, 19.9, 20.9, 21.5, 22.2, 20.2, 20.7, 20.2, 20.5, 17.5, 18, 18.5, 19.1, 21.6, 16.1, 16.6, 17.2, 0.8, 1.4, 1.2, 5, 1.4, -0.8, -0.5, -0.8, -0.7, 19.1, 19.1, -0.9, 5.4, 2.7, 4.3, 9.3, -2.9, 11.3, 11.3, 2.6, 1.7, 4.2, 5.6, 13.2, -6.9, -2, 0.3, 1.9, 7.1, -8.2, 0.8, 9.6, 10.6, 16.9, -9.1, 1.1, 11.2, 18, -9.7, -2.2, -1.4, 12.9, -3.4, -6.1, 14.1, -9.9, -6.2, -11.3, -6.6, 15.8, -7, -0.5, 16.9, 21.4, -7.8, -8.1, 21.8, -8.3, 17, -8.4, 22.1, -8.5, -13.1, -9.3, -13.5, 22.1, -9.4, 17, -14, 17.4, -9.9, -14.4, 22.3, 18.5, 22.1, 10.9, 18.9, -10.7, -15.1, 8.2, 14.4, -15.6, -11.5, 16.4, 19.3, -15.8, 17.3, 17.2, -12.1, 17.2, -12.8, 22.7, 22.4, -17.3, 21.6, -13.3, 21.6, -18.1, 21.4, 21.3, 21.4, -18.6, -14.4, 20.7, 20.8, -15, 19.3, -15.4, -18.8, -19, 17.6, 18.2, -15.8, 18.8, -16.5, 17.5, 19.8, 18.6
			, 19.6, -20, 16.3, -20.4, 17.9, -20.6, 17.8, -17.1, 16.7, -21.1, 16.8, -21.3, 16.8, -21.4, 17.8, -17.4, -18, 17.8, 17.8, -21.6, -21.3, 17.3, 19.6, -17, 20.2, -17.4, -20.6, 20.8, -20.1, 21.9, -16.7, -20.1, 22.8, -20.1, -16.1, -20.1, -15.3, -20.2, 25.7, -19.8, -19.3, 26.4, 26.4, -18.2, -11.2, -17.5, 26.7, -15.5, 26.4, -9.4, 25.8, 26.2, 26.6, -6.5, -5, 25.6, 24.8, 24.7, -3.9, 25.2, -9.4, 25.3, -7.8, 25.4, -6.3, 25.7, -5.2, -3.4, -2.7, 25.6, -0.8, 24, 23.6, 2.4, 6, 8.5, 3.5, 12.2, 9.4, 14.1, 16.2, 14.3, 4.4, -0.9, -4.4, -10.3, -11.2, -13.4, -14.4, 12.4, 14.9, -17.3, -18.3, -15.5, -17.2, -21.8, -16, -7.3, 21.5, 21.8, 21.9, 22.1, 21.7, 21.9, 22, 22.1, 22.1, 22.1, 20.6, 20.8, 21, 21.2, 21.3, 21.4, 21.5, 21.5, 21.5, 21.4, 21.3, 19.5, 19.8, 20.1, 20.4, 20.6, 20.8, 21, 21.1, 21.1, 21.1, 21.1, 21, 20.9, 20.8, 18.6, 19, 19.4, 19.8, 20.1, 20.4, 20.6, 20.8, 21, 21, 21.1, 21.1, 21, 17.9, 18.3, 18.8, 19.1, 19.5, 19.8, 20, 20.2, 20.4, 20.5, 20.6, 20.6, 20.6, 20.6, 20.5, 20.4, 20.2, 20, 19.8
			, 18, 18.5, 18.9, 19.3, 19.7, 20, 20.3, 20.6, 20.8, 20.9, 21.1, 21.2, 21.2, 21.2, 21.2, 21.2, 21.1, 20.9, 20.8, 20.6, 20.3, 20, 19.7, 17.4, 18, 18.4, 18.9, 19.3, 19.7, 20, 20.2, 20.5, 20.7, 20.8, 20.9, 21, 21, 21, 20.9, 20.8, 20.7, 20.5, 20.2, 20, 19.6, 15.8, 16.4, 17, 17.5, 18, 18.4, 18.8, 19.2, 19.5, 19.8, 20.1, 20.3, 20.5, 20.6, 20.7, 20.7, 20.7, 20.7, 20.6, 20.5, 20.3, 20.1, 19.9, 19.6, 19.3, 16, 16.7, 17.3, 17.8, 18.3, 18.8, 19.2, 19.6, 19.9, 20.2, 20.4, 20.6, 20.8, 20.9, 21, 21, 20.9, 20.8, 20.7, 20.5, 20.3, 20, 19.7, 19.3 };
		for (int i = 0; i < lLen; i++) LegLeo[i].y = LegLeoK[i] * zTbb * zyTbb * oribb;
		delete[]LegLeoK;
		LegLeoK = new float[lLen]{ 0.3, 0.8, 0.6, 1.1, 0.6, 1.3, 1.8, 2, 1.4, 2, 2.6, 3.6, 0.8, 0.2, 0.6, 0.9, 1.6, 2.2, 2.5, 3.1, 3.5, 3.9, 4.4, 4.5, 4.7, 4.9, 0.3, 0.4, 0.1, 1.2, 0.1, 1.2, 2.2, 3, 3.3, 4.2, 4.9, 4.7, 5.6, 5.9, 6.3, 6.6, 0.2, 0.1, 0.3, 0.2, 1.3, 3, 3.7, 3.8, 5.2, 6.5, 7, 7.2, 7.5, 7.9, 7.9, 9.2, 0.2, 1, 0.6, 2, 2.7, 4, 5.7, 6, 7, 7.4, 8.3, 8.5, 9.2, 9.7, 10.4, 10.4, 0.4, 0.6, 0.1, 1.6, 2.7, 4.4, 4.6, 5.8, 8, 7.8, 9.1, 9.2, 10.3, 10.2, 10.9, 11.3, 0.8, 0.8, 0.8, 1.7, 2, 4.4, 4.9, 5.9, 6.8, 7.9, 8.5, 10, 10.9, 11.7, 11.5, 12.5, 1.1, 0.3, 1.7, 2.7, 4.3, 4.7, 6, 7.3, 7.4, 8.6, 10, 10.3, 10.5, 11.3, 11.9, 13.2, 12.7, 13.5, 0.4, 1.2, 0.7, 2.6, 5.1, 5.2, 6.6, 8.7, 9.1, 10.7, 11.8, 12.5, 13.3, 13.5, 14.4, 0.2, 0.7, 1.1, 3.7, 4.4, 6.1, 6.7, 7.4, 9.7, 11, 11.4, 13.3, 14.4, 14.1, 15.2, 0.3, 0.7, 0.7, 3.3, 4.7, 6.4, 8, 8.6, 9.9, 11.4, 12.5
			, 13.2, 14.5, 15.7, 15.5, 16.3, 0.6, 1.2, 2, 3.3, 5.5, 6.7, 8, 8.8, 9.8, 11.2, 12.7, 13.7, 14.6, 15.8, 15.6, 16.6, 17.1, 0, 0.4, 1.3, 2.7, 4.4, 6.8, 8.4, 9.7, 10.7, 12, 12.4, 13.5, 14.6, 16.1, 17.2, 17.4, 17.9, 0, 0.4, 1.7, 3.3, 5.7, 7.8, 9.5, 10, 12.2, 13.2, 14.1, 14.1, 16, 16.8, 17.4, 18.1, 19.1, 1, 0.7, 3.3, 5.1, 7.1, 8.4, 10, 11.8, 12.6, 14.1, 15.4, 16.6, 17.9, 18.6, 19.2, 19.5, 1, 0.6, 1.1, 3.7, 5, 6.5, 7.7, 10.4, 11.2, 12.7, 14.3, 15.6, 16.2, 17.2, 18.6, 19.3, 19.4, 20.2, 21, 1, 0.9, 2.7, 4.7, 5.4, 7.1, 9.1, 10.7, 12.7, 13.8, 15.2, 16.7, 17.4, 18.7, 19.2, 20.1, 21.1, 21.9, 21.6, 1.3, 0.7, 2.4, 4, 6.3, 7.7, 8.1, 10.2, 12.1, 13.7, 15.2, 16, 17.4, 19.4, 19.9, 20.8, 21.6, 22.3, 22.9, 22.3, 0.4, 0, 2.4, 3.7, 5.7, 7.4, 8.5, 10.4, 12.1, 14.3, 14.4, 16.1, 18.1, 19.4, 20.6, 21.2, 22.1, 22.8, 23.6, 0.2, 0.4, 2, 3.9, 6.2, 7.1, 8.7, 9.8, 11.8, 13.4, 14.5, 16.7, 16.8, 18.5, 19.2, 21.2
			, 22.8, 23.4, 23.1, 23.4, 24.3, 25.3, 0, 1.3, 3.7, 4.4, 6, 8.2, 9.1, 10.4, 11.8, 12.3, 14.6, 15.5, 17.2, 18.1, 19.9, 21, 21.3, 23, 24, 24.9, 25.4, 26.1, 0.4, 0.7, 1.5, 3.7, 5.3, 7.3, 8.2, 10.4, 12.3, 12.7, 14.7, 16.2, 17.4, 19.2, 20.1, 21.9, 22.6, 23.3, 24.9, 25.4, 26.2, 27, 27.2, 1, 0.6, 2, 1.9, 3, 4.7, 6.7, 8.4, 9.4, 11.7, 13.1, 14.8, 16.5, 18.5, 19, 20.8, 21.7, 23.8, 25.5, 25.3, 26.3, 27.1, 27.5, 27.9, 0.4, 0.6, 0.7, 2.5, 3.7, 4, 6.7, 8.4, 10.1, 11.8, 13.4, 15.4, 17.4, 18.5, 20.5, 22.1, 22.8, 23.3, 24.8, 26.8, 27.3, 28, 28, 29.2, 1, 0.9, 0.8, 1.7, 2.6, 3.3, 4.7, 6, 7.8, 9.4, 10.4, 12.7, 14.5, 16.5, 18.8, 19.9, 21.2, 21.6, 23.7, 25.2, 25.3, 27.1, 28.8, 28.8, 29.6, 29.5, 29.9, 1.6, 1.1, 1.3, 2.5, 3, 3.6, 5.5, 6.4, 7.4, 9.7, 11.1, 13.1, 14.8, 16.8, 18.8, 20.1, 21.5, 23.2, 24.1, 25.4, 26.5, 28, 29.2, 30.1, 30.7, 31.5, 30.9, 1, 0.3, 1.3, 2.5, 2.6, 4.3, 4.7, 6, 6.7, 8, 9.7, 11.1
			, 13.4, 13.6, 15.8, 17.4, 19.5, 21.5, 22.6, 24, 25.5, 26, 27.7, 28, 29.7, 30.8, 31.4, 32.8, 0.3, 0.3, 2.3, 3, 3.4, 5, 6.6, 8.4, 9.7, 11.1, 12.1, 13.1, 14.5, 15.4, 16.4, 17.8, 20.5, 22.1, 22.2, 23.9, 25.6, 27.3, 27.6, 30, 30.2, 31.2, 31.8, 32.7, 32.8, 34.2, 0.3, 0, 0.6, 2, 3.7, 5, 6.4, 7, 8.5, 9.4, 10.1, 12.8, 14.5, 16.5, 18.8, 19.8, 21.2, 23.2, 24.8, 25.9, 27.6, 28, 29.7, 30.8, 31.9, 32.4, 33, 34.5, 34.4, 1.6, 0.6, 2.8, 3.3, 4.5, 4.1, 6, 7.1, 7.8, 9.4, 10.4, 12.2, 14.1, 15.1, 17.4, 19.2, 20.1, 21.5, 23.9, 25, 26.8, 27.9, 28.3, 29.9, 31.5, 32.2, 32.7, 33.3, 34.5, 34.9, 35.7, 0, 0.7, 0.9, 2.1, 3.2, 4.6, 5.3, 6.3, 6.7, 8, 8.7, 10.7, 12.9, 14.4, 16.5, 18.1, 20.1, 22.1, 23.5, 25.2, 26.7, 27.3, 28.2, 30.8, 31.3, 32.2, 34, 34.1, 35.2, 36.8, 36.3, 36.7, 37.1, 0.9, 0.2, 1.8, 2.8, 4.3, 4.7, 5.7, 7, 7.9, 9.4, 11, 12.8, 14.4, 16.1, 18.1, 19.8, 21.2, 23.5, 25.2, 27.5, 28.6, 29.5, 30.9, 32.9, 32.7, 33.4
			, 35.5, 36.9, 37.2, 37.6, 38.5, 0.5, 1.3, 2.3, 2.4, 4.3, 4.3, 5.5, 7.7, 7.3, 9.4, 11.1, 12.8, 14.8, 16.3, 17.8, 19.8, 21.5, 23.5, 25.2, 26.6, 27.4, 29.5, 30.1, 31, 33.1, 34.2, 34.8, 36, 37.2, 38.3, 39.3, 39.2, 40.2, 0.5, 1.3, 2.6, 4.8, 4.7, 6.8, 7.4, 9.1, 10.4, 12, 13.8, 15.1, 16.9, 18.1, 18.5, 20.5, 21.9, 22.8, 24.2, 26.6, 27.9, 29.9, 30.9, 31.5, 33.2, 33.9, 34.6, 35.9, 37.6, 37.2, 38.1, 38.9, 39.6, 40.2, 0.8, 0.9, 1.2, 2.3, 3.2, 4.6, 6.3, 7.3, 8, 9.4, 11.1, 12.8, 14.8, 15.4, 16.8, 17.8, 19.5, 21.5, 23.2, 24.8, 27.2, 29.5, 30.2, 31.5, 33.3, 34.7, 34.8, 36.7, 37.1, 38.8, 40, 40.9, 40.9, 41, 42.2, 0.6, 1, 2.3, 2, 4.3, 5.6, 7, 7.4, 9.4, 11.1, 12.9, 13.8, 15.4, 17.8, 19.5, 20.5, 22.4, 24.5, 26.6, 28.6, 29.7, 31.3, 32.9, 34.2, 35.6, 36, 36.7, 38.1, 38.5, 40.1, 40.8, 41.5, 42.5, 43.2, 0.3, 0.2, 2, 3.5, 4.8, 5.5, 6.1, 7.9, 9.4, 10.7, 12.3, 13.7, 14.1, 16.4, 18.2, 19.8, 21.9, 23.2, 24.5, 25.9, 27.2, 28.9, 31.6, 31.8
			, 34, 34.8, 36.3, 38.3, 39.6, 39.8, 40.9, 41.6, 41.7, 43, 43.8, 44.1, 0.9, 2, 2.9, 4.1, 5.3, 6.3, 7.7, 9.4, 10.8, 11.9, 13.7, 15.3, 17.1, 18.8, 20.5, 21.5, 23.6, 25.2, 26.9, 27.5, 29.9, 31.4, 33.6, 34.2, 35.3, 37.3, 38.1, 39.3, 39.2, 40.7, 41.9, 42.5, 44, 44.2, 45.3, 45, 1, 3.6, 4.3, 5.3, 6.3, 7.7, 9.7, 11.4, 13.1, 14.5, 15.8, 17.8, 19.5, 21.2, 22.8, 24.5, 25.3, 26.9, 29.3, 30.9, 32.6, 34, 34.3, 35.7, 38.9, 40.3, 39.9, 42.3, 42.4, 44.3, 44.2, 45, 47, 46.3, 1.1, 0.1, 2.6, 3.7, 4.2, 4.8, 6.4, 7.9, 9, 11.4, 12.8, 14.4, 15.8, 17.4, 19.5, 20.7, 21.5, 23.5, 24.5, 26.9, 28.6, 30.9, 32.6, 33.6, 34.9, 36, 37.4, 39.4, 40.3, 41.6, 41.7, 42.6, 43.9, 44.9, 45.5, 47, 46.9, 47.6, 48.2, 47.9, 0.3, 4, 4.7, 6.3, 7, 8.8, 10.1, 12.1, 13.4, 15.1, 16.7, 18.5, 20.1, 21.2, 22.8, 24.8, 26.6, 27.5, 29.9, 31.3, 33.6, 35.3, 36.2, 38, 39.4, 39.4, 40.7, 43, 42.8, 43.9, 44.9, 45.5, 47.5, 48.2, 48.6, 48.9, 0.2, 0.4, 3.6, 4, 6, 6.7, 8.3
			, 9.7, 10.9, 12.7, 14.4, 16.1, 17.8, 19.5, 20.7, 21.8, 23.9, 25, 27.1, 28.6, 29.6, 32, 33.9, 36.3, 38, 38.3, 40.6, 40.8, 42.8, 43.4, 43.8, 45.5, 47.4, 47.8, 48.7, 48.9, 50.2, 50.2, 0.4, 1.6, 2.6, 3.3, 4.9, 6.2, 6.7, 7.6, 8, 10.3, 11.7, 13.1, 14, 15.1, 17.4, 18.8, 21.2, 21.9, 23.9, 25.2, 27.2, 28.6, 30.2, 31.6, 34.3, 35.3, 35.8, 39, 40.6, 41, 42.1, 44.1, 44.7, 45.5, 47.2, 49, 49.1, 49.9, 50.8, 51.5, 52.3, 0.9, 2.6, 4, 4.7, 6, 7.6, 8, 10, 12.1, 13.7, 15.1, 16.2, 18.7, 19.5, 20.5, 22.4, 23.3, 25.5, 26.4, 27.2, 29.9, 31.4, 33.6, 34.5, 36.7, 38.1, 41, 42.1, 43.4, 44.8, 45.7, 47, 46.5, 48.3, 48.9, 50.3, 51.6, 52.4, 51.8, 52.3, 53.5, 0.3, 2.7, 3.9, 4.7, 6.5, 7.6, 7.6, 9, 10.7, 12.1, 14, 14.8, 16.1, 17.4, 19.5, 20.5, 22.4, 23.9, 25.5, 27, 28.6, 30, 31.3, 33, 34, 35.6, 36.6, 38.3, 40.3, 40.8, 42, 43.4, 44.8, 46.1, 46.8, 47.7, 48.2, 50.2, 50.3, 51, 53, 52.6, 54.4, 53.9, 0.5, 2.9, 3.1, 4.2, 6, 6.7, 8.1, 9.7
			, 10.6, 12.4, 14.6, 15.1, 17, 17.8, 19.6, 21.5, 22.2, 23.9, 24.5, 26.6, 28.2, 30.2, 30.9, 32.6, 34.3, 36.3, 37.6, 39.4, 40.3, 42.1, 44.1, 44, 46.8, 47.8, 49.3, 50.4, 51, 53.1, 53.5, 54.1, 54.5, 55.6, 55.9, 0.3, 2.9, 2.7, 4.2, 5.6, 6.9, 8.5, 10, 10.7, 11.7, 13.4, 14.3, 16, 16.4, 18.8, 20.1, 22, 22.8, 24.8, 25.5, 26.9, 27.9, 30.3, 31.8, 33.6, 36.3, 37.2, 39.2, 41.4, 43, 44.1, 45.4, 47.5, 48.8, 48.8, 51.1, 51.3, 52, 53.4, 54.8, 55.5, 56, 57.1, 0.2, 3.4, 3.3, 4.6, 6, 8, 9.2, 10, 11.7, 12.7, 14.7, 16.1, 17.1, 18.6, 20.8, 21.5, 22.9, 24.7, 25.9, 27.5, 28.9, 30.6, 31.4, 33.3, 35.3, 37, 38.7, 40.3, 42.1, 43.4, 45, 46.8, 47.7, 49.3, 51.1, 51.7, 51.9, 52.9, 55.1, 54.8, 56.1, 56.8, 57.8, 57.2, 58.3, 0.5, 3.1, 3.3, 4.6, 5.9, 7.6, 9.4, 10.7, 11, 11.8, 13, 14.7, 15.6, 17.1, 18.8, 19.1, 21.5, 22.7, 24.2, 24.9, 26.2, 27.4, 30, 31.2, 31.3, 34, 35, 37, 39.4, 40.1, 42.3, 44.4, 45.4, 46.8, 48.8, 50.2, 51.5, 51.7, 52.9, 53.2, 54.5, 55.6
			, 56.5, 58.4, 57.9, 59, 59.8, 59.2, 0.8, 0.8, 3.6, 5.3, 6.7, 8, 9.5, 11.1, 12.3, 13.6, 14.9, 15.6, 16.6, 18.6, 19.8, 21.5, 23.1, 23.2, 24.5, 26.1, 27.7, 29.6, 30.2, 31.3, 33.6, 36.3, 37.6, 39.1, 40.7, 42.7, 44.1, 46.4, 47.5, 49.3, 50.2, 51.8, 51.7, 54.4, 55.5, 55.7, 57.8, 57.8, 58.8, 59.5, 60.1, 59.9, 60.7, 0.2, 3.1, 5.3, 6, 8, 9.6, 10.6, 11.1, 12.6, 14.1, 15.7, 16.9, 17.7, 18.7, 20.8, 22.8, 23.2, 25.4, 26.2, 27.2, 27.9, 30.2, 30.6, 32.5, 34.1, 36, 37.4, 39, 40.7, 42.7, 43.7, 46, 46.7, 49.1, 50.8, 52.9, 54.4, 55.1, 56.1, 56.3, 58.5, 58, 59.3, 60.1, 60.6, 61.6, 61.7, 0.4, 2.9, 4.6, 6, 7.3, 8, 9.6, 12, 12.7, 14.1, 13.8, 16.1, 16, 17.8, 18.6, 19.9, 21.4, 23.5, 24.4, 25.5, 26.2, 27.5, 29.5, 30.2, 31.6, 32.6, 34, 36, 38, 39.9, 43, 43.5, 46.1, 47.7, 48.7, 51, 52.4, 54.1, 55.6, 55.6, 56.7, 59.1, 59.1, 60.1, 61.7, 61.2, 61.8, 63.1, 62.8, 0.9, 3.6, 3.3, 5.1, 6.3, 7.8, 8.7, 9.9, 11.4, 11.8, 13.7, 14.7, 15.5, 17, 17.8, 18.8
			, 20.8, 21.4, 22.9, 25.2, 26.1, 26.9, 28.9, 29.3, 29.9, 32.2, 33.6, 34.3, 36.3, 38.7, 40.3, 41.9, 43.4, 44, 46.8, 49.1, 50, 52.2, 52.5, 54.9, 56.3, 56.3, 58.5, 59.8, 61.2, 60.6, 62.3, 62.7, 63.2, 64.4, 64.5, 0.9, 3.2, 4.3, 6, 6.9, 8.3, 10, 11.4, 11.8, 13, 14.1, 15.1, 16.2, 17.8, 18.9, 20.5, 21.6, 22.7, 24.4, 25.7, 26.7, 28.2, 28.9, 30.9, 30.9, 32, 33.8, 34.9, 36.3, 37.6, 40, 42.4, 44.8, 46.4, 47.7, 50.1, 51.8, 53.8, 55.6, 56.9, 56.9, 58, 59.3, 61.2, 62, 63, 63.7, 64.6, 64.7, 65.6, 0.1, 2.2, 2.9, 4.9, 6.3, 6.9, 8.8, 10.3, 12, 12.7, 13.4, 14.8, 16.1, 17.4, 18.6, 18.9, 20.8, 22.1, 22.5, 23.9, 25.5, 25.9, 27.2, 29.5, 29.9, 31.5, 32.9, 34, 36.3, 37, 38.5, 40.2, 42.1, 44.4, 46.1, 47.5, 49.5, 51, 53.5, 55.1, 57.6, 58.5, 59.8, 61.2, 61.2, 62.5, 62.8, 64.2, 65.2, 65.1, 65.6, 66.4, 66.1, 67, 0.2, 0.6, 0.8, 3.6, 5.4, 6.7, 7.8, 9, 9.4, 10.4, 12.7, 12.4, 14.1, 15.1, 16.2, 18.1, 18.8, 20, 21.2, 23.2, 23.9, 25.6, 25.9, 27.9, 28.8, 29.3
			, 31.3, 32.3, 34.1, 34.4, 37, 38, 39.2, 40.6, 42.3, 44.1, 45.7, 47.1, 48.8, 50.4, 51.8, 52.7, 55.1, 56.9, 57.3, 59.6, 60.5, 61, 63.2, 64.5, 65, 66.5, 66.2, 67.1, 67.7, 67.5, 68.3, 68.5, 0.2, 1, 2.9, 4.9, 6.7, 8.3, 9.1, 11.1, 12.7, 12.7, 14, 14.6, 16.1, 17.4, 17.6, 19.1, 20.8, 22.1, 23.5, 24.1, 25.5, 26.6, 27.6, 29.1, 30.3, 32.2, 33.6, 33.6, 34.7, 35.9, 38, 39.4, 40.7, 42.1, 43.7, 45.7, 46.8, 48.4, 51.5, 54.5, 55.6, 56.9, 58.9, 60.3, 61.4, 62.5, 63.4, 65.2, 65.9, 65.7, 67.1, 68.6, 68, 68.5, 68.5, 69.8, 0.2, 1.3, 3.6, 6, 7, 8.4, 10.1, 11.3, 11.9, 12.7, 13.9, 15.1, 17.4, 17.6, 18.7, 20.1, 20.8, 21.8, 23.2, 24.5, 25.9, 26.6, 28.1, 28.6, 30.8, 31.5, 32, 34.2, 34.3, 36.9, 37.5, 39.6, 41.4, 43.4, 44.8, 46.1, 47.5, 48.8, 50.8, 52.2, 54.9, 56.5, 58.2, 59.7, 60.3, 61.8, 63.4, 65.2, 65.8, 67.1, 68.2, 68.7, 69.5, 70.1, 70, 70.8, 71.2, 0.2, 1.3, 2.9, 6.6, 8, 9.7, 10, 11.4, 12.3, 14, 14.4, 15.2, 16.7, 17.1, 19.1, 20.5, 21.2, 21.8, 23.9, 24.5
			, 25.8, 27, 28.4, 30, 30.2, 32.2, 33.5, 34.2, 34.7, 35, 36.6, 37.3, 38.7, 40.1, 41.7, 42.8, 44.5, 47.1, 49.1, 50.4, 52.2, 53.1, 54.5, 56.9, 58.9, 60.9, 61.1, 63.2, 64.6, 65.2, 65.9, 67.7, 68.2, 69.1, 69.8, 70.8, 71.5, 71.4, 72.4, 72.6, 0.6, 2.2, 5.6, 7.3, 8.4, 9.6, 10.5, 11.7, 12.6, 13.9, 15.1, 18.7, 18.9, 19.9, 21.4, 22.8, 23, 24.5, 25.5, 26.4, 27.5, 28.8, 30.2, 32.1, 32.3, 34, 35, 36, 37.4, 37.6, 38.3, 41, 40.8, 43.4, 44.8, 46.1, 47.7, 49.7, 52.4, 53.8, 53.5, 56.2, 58.9, 60.5, 61.9, 63.9, 64.1, 66.6, 67.2, 67.3, 68.4, 69.8, 70.8, 71, 72.6, 72, 72.7, 72.6, 73.5, 0.9, 1.5, 3.6, 5.3, 7.2, 9.3, 10, 11, 12.7, 14, 15.1, 15.6, 16.1, 16.4, 17.6, 17.8, 19.5, 19.9, 21.7, 22.7, 22.8, 24.5, 24.8, 25.8, 26.6, 27.7, 29, 28.8, 29.7, 30.6, 32.7, 34.1, 34.9, 36, 36.8, 38.3, 39.6, 41.6, 43, 43.7, 45.7, 47.7, 48.8, 50.8, 52.4, 53.5, 55.1, 56.9, 58.6, 60.4, 63.7, 63.9, 66.3, 67.7, 68.1, 69.5, 69.8, 71.1, 72.5, 73.1, 72.9, 73.8, 74.4, 74.6, 75.1, 74.7
			, 75.8, 1.6, 2.3, 3.6, 4.1, 6.3, 7.6, 8.9, 10.7, 11, 12.9, 13.4, 14.1, 14.8, 15.7, 17.8, 18.2, 19.3, 21.2, 22.3, 23.5, 24.8, 26.1, 26.8, 27.9, 29.3, 30.5, 30.7, 31.9, 32.2, 33.4, 34, 35.6, 37.1, 38.8, 39.2, 39.9, 42.1, 42.4, 44, 44.8, 46.4, 48.8, 51.1, 52.4, 54.2, 55.6, 57.9, 59.6, 60.6, 62.9, 63.5, 65.9, 66.4, 67.6, 69.3, 71.1, 70.8, 72.4, 73.4, 74.2, 75, 75.4, 76.4, 76.1, 1.4, 1.6, 2.9, 4.3, 6, 7.3, 8.1, 8.7, 10.4, 11.4, 12.3, 14, 15.4, 15.8, 18, 19.4, 19.1, 19.8, 21.4, 21.9, 23.6, 23.9, 25.4, 26.1, 26.7, 27.5, 28.8, 29.6, 30, 31.5, 32.6, 32.7, 34, 35.3, 35.7, 36.6, 36.9, 37.6, 38.7, 39.4, 39.8, 41.4, 41.8, 43.7, 45, 46.1, 47, 49.1, 49.7, 51.5, 53.8, 56.5, 58.2, 60.5, 63, 64.3, 65.7, 66.5, 68.6, 69.4, 70, 72.4, 72.6, 74.5, 74.3, 75.9, 76, 76.4, 76.6, 76.9, 78, 77.4, 77.5, 0.6, 0.6, 2.2, 2.9, 4.7, 4.6, 6.2, 8, 8.7, 9.2, 9.9, 11.3, 11.7, 12.4, 14, 14.3, 16.3, 16.4, 17.1, 18.7, 19.4, 20.7, 21.4, 22.1, 23.4, 24.4, 25.1
			, 26.3, 26.9, 28.3, 28.8, 29.3, 30, 30.9, 31.6, 32, 32.7, 32.9, 33.6, 34.8, 36.1, 36.5, 37.8, 38.9, 39.6, 40.7, 42.1, 42.6, 44.8, 45.7, 46.8, 48.3, 49.7, 51.5, 52.8, 54.9, 57.2, 58.9, 60.5, 63, 65.3, 66.6, 67.6, 70.4, 71.1, 71.8, 74, 75.3, 75.2, 76.1, 77.9, 78.2, 78.4, 79.3, 78.7, 79.7, 0.9, 1.3, 2.1, 3.4, 4.4, 5.6, 7.3, 8, 9.2, 10.3, 11.3, 12.4, 13.2, 14.1, 15.3, 16, 17.4, 17.8, 18.5, 19.9, 21.9, 22.6, 24.1, 24.7, 26.7, 27.4, 28, 28.6, 28.6, 29.6, 30.1, 30.8, 31.2, 31.4, 31.7, 32.6, 32.9, 33.4, 34.1, 34.6, 34.8, 34.8, 35.5, 36.7, 37.4, 38.6, 38.9, 40.4, 41.1, 42.8, 43.9, 44.3, 44.6, 46.1, 47.5, 48.8, 50.4, 51.1, 52.9, 54.5, 55.8, 58.2, 59.9, 61.9, 63.9, 64.7, 66.3, 67.1, 69.7, 71.1, 72.7, 73.5, 74.2, 75.4, 76.6, 76.7, 77.8, 78.9, 79.5, 80.7, 80.2, 81.2, 0.9, 2.6, 2.8, 3.6, 5.3, 6.6, 7.6, 8.4, 10, 11.8, 12.4, 13.3, 13.3, 15.3, 15.1, 16.2, 17.9, 20, 19.7, 20.3, 20.8, 21.2, 23.1, 23.5, 24.6, 25.3, 26.5, 27.2, 26.9, 28.6, 29.1, 28.9, 29.9, 29.6
			, 30.4, 30.3, 30.1, 31.5, 33.5, 35, 35.9, 36.7, 37.4, 37.3, 37.6, 38.6, 38.9, 39.8, 40.6, 41.5, 41.5, 42.3, 43, 44.3, 46.1, 47, 47.3, 49, 49, 51.1, 52.9, 55.1, 55.6, 57.6, 59.5, 61, 62.6, 65, 66.3, 68.4, 69.1, 70.5, 72.4, 73.5, 74.7, 75.2, 77.2, 77.9, 79, 78.6, 79.2, 80.2, 80.2, 81.2, 81.8, 81.6, 82.1, 0.9, 1.5, 2.2, 2.9, 4.1, 5.3, 6.1, 6.9, 8.3, 8.8, 10, 11.3, 11.5, 11.9, 13.7, 14.6, 15.6, 16, 17.1, 17.8, 18.5, 20, 20.7, 21.9, 22.6, 24.4, 24.8, 25.8, 26.2, 26.7, 27.5, 28.2, 28.4, 29.2, 29.2, 29.5, 29.4, 39.3, 40.2, 40.5, 40.2, 40.9, 42.2, 42.8, 42.9, 43.6, 45.5, 45.7, 47, 47.7, 49.2, 49.8, 51.2, 52.6, 53.9, 56.5, 58.5, 59.9, 61.6, 62.3, 64.6, 66.6, 69, 70, 71.7, 72.7, 73.9, 75.8, 77.1, 77.9, 78.6, 80.7, 80.4, 82.1, 81.5, 82, 83.2, 83.1, 83.6, 1.6, 0.6, 2.9, 3.6, 4.2, 4.2, 6.6, 8, 7.8, 9, 9.7, 10.5, 12.2, 12.9, 13.3, 15.6, 16.4, 16.7, 18.6, 20.1, 21, 21.2, 22.4, 22.9, 23.4, 24.7, 24.9, 25.5, 26.7, 27.1, 27.7, 27.8, 28
			, 28.4, 29, 42.1, 42, 42.2, 43.1, 43.8, 44.4, 45, 46.8, 46.6, 47.7, 49.5, 50.3, 50.7, 52.1, 53.7, 54.5, 56.2, 57.1, 58.5, 60.9, 62.6, 64.3, 65, 66.8, 67.8, 69.7, 72.2, 74, 75.4, 76.1, 77.8, 77.4, 78.7, 80.1, 81.2, 81.3, 82.5, 82.9, 83.8, 83.8, 84.7, 84.6, 84.5, 84.9, 0.9, 0.3, 1.4, 2.9, 2.6, 4.6, 6, 7.2, 8.3, 9.2, 9.9, 10.7, 11.2, 12.7, 12.6, 13.1, 14.4, 15.1, 15.8, 15.9, 17.5, 18.4, 19.3, 20.4, 20.2, 21.2, 22.6, 23.7, 23.6, 24.6, 25.5, 25.9, 26, 26.6, 27.2, 27.4, 27.4, 44.2, 44.3, 45.5, 45.1, 46.2, 46.3, 47.7, 49.5, 49.5, 50.6, 52, 53.6, 53.7, 55.4, 56.9, 59.2, 60.3, 62.2, 63.2, 64.5, 66.3, 67, 68, 68.4, 69.6, 71.7, 73.7, 75.1, 76.8, 78.5, 79.2, 79.7, 80.6, 81.2, 82.2, 83.4, 84.2, 84.7, 85.4, 85.8, 86.6, 86.3, 0.8, 0.3, 1.6, 2.7, 2.9, 3.9, 4.9, 6.6, 7.3, 7.6, 8.7, 10.3, 10.9, 11, 11.9, 14, 13.9, 15.1, 16.5, 16.7, 17.9, 19.2, 19.4, 21.4, 21.8, 22.8, 22.8, 24, 24.1, 24.9, 25.9, 25.5, 25.9, 26.3, 26.2, 46.2, 46.3, 47.3, 47.1, 47.7
			, 48.3, 49.6, 50.5, 52, 52.4, 53.2, 55.1, 55.7, 57.8, 58.9, 59.9, 61.8, 63.6, 64.6, 66.3, 67.7, 70, 71.4, 73.4, 73.6, 76.1, 77.1, 77.9, 79.1, 81.2, 81.6, 83, 83.7, 84.5, 84.8, 85.3, 85.6, 86.2, 86.7, 87, 87.7, 87.8, 0, 1, 2.1, 3.5, 4.2, 5.7, 6.6, 6.9, 7.3, 9, 10, 10.4, 11, 11.7, 12.8, 13.3, 14.5, 15.3, 16.9, 17.1, 18.4, 18.5, 20.1, 20.8, 21.2, 22.4, 22.6, 23.5, 23.7, 24.4, 24.9, 24.8, 25.7, 25.4, 48.2, 48.7, 48.4, 49.1, 50, 50.8, 52.2, 52.8, 54.1, 54.4, 55.7, 56.5, 58.5, 60.3, 60.9, 62.3, 64.2, 64.3, 67, 69, 71.1, 72.5, 73.7, 75.8, 76.9, 78.5, 80.3, 81.5, 82.6, 83.8, 84.1, 85.4, 85.5, 86.1, 87, 88.1, 87.7, 88.1, 88.6, 88.8, 89.3, 0.3, 1.4, 1.6, 3.6, 4.2, 3.4, 5, 5.6, 7, 7.5, 8, 9.3, 9.4, 10, 12, 12.1, 13.5, 14, 15.3, 15.8, 17.1, 17.6, 18.6, 19.4, 20.4, 20.8, 21.8, 22.1, 23.1, 23.5, 23.5, 24, 24.5, 24.6, 49.5, 49.8, 50.1, 50.3, 51.1, 52.2, 52.6, 53.6, 55, 55.6, 56.4, 57.5, 58.3, 59.6, 61.6, 62.6, 64.9, 65.9, 67
			, 69.7, 71.7, 72.7, 74.1, 75.7, 77.1, 77.8, 78.9, 80.6, 82.4, 82.4, 84.6, 84.9, 85.5, 88.2, 88.9, 88.3, 89.3, 90, 90, 89.7, 91.5, 90.9, 0.3, 0, 0.9, 1.9, 2.9, 3.9, 3.6, 4.9, 4.8, 6.7, 8, 8.6, 10.1, 10.4, 10.6, 11.9, 12.6, 13.1, 13.3, 14.4, 15.4, 16.7, 16.9, 18.3, 18.8, 19.7, 20.2, 21.1, 21.4, 22.3, 22.9, 22.6, 23.1, 23.3, 23.8, 51.5, 52.2, 51.8, 52.9, 52.4, 53.9, 54.3, 55.8, 55.7, 57.6, 57.7, 59.2, 61.2, 61.7, 64.3, 65.9, 67.3, 69.2, 70, 73.1, 75.4, 76.5, 77.5, 80.1, 81.9, 82.6, 83.7, 86.2, 85.8, 87.5, 87.6, 88.2, 89.4, 89.9, 90.4, 91.6, 90.8, 91.2, 91.5, 91.7, 0.1, 0.9, 0.1, 2.6, 3.4, 4.3, 5.1, 5.6, 7, 8.3, 8.9, 9.6, 9.9, 10.9, 11.2, 12.6, 13.7, 14.1, 15.3, 16, 16.5, 17, 18.8, 19.2, 20, 20, 20.7, 21.6, 21.9, 22.3, 22.4, 22.9, 53.3, 53, 54, 55.4, 55.2, 55.8, 57.5, 59, 59.6, 60, 61.9, 62.8, 64.6, 66.3, 68.4, 70, 71.4, 73.7, 75.8, 77.8, 78.7, 79.5, 81.4, 82.3, 84.9, 85.3, 86.9, 87.5, 89.1, 89.9, 90, 91.2, 92, 91.8, 92.3
			, 92.6, 93.3, 92.9, 94.1, 0.9, 0.6, 1.4, 2.6, 4.3, 3.4, 4.7, 6.9, 7.3, 8.2, 9.2, 9.7, 10.4, 10.9, 11.3, 11.8, 13.5, 14.6, 15.1, 15.6, 15.8, 16.5, 17.8, 17.9, 18.7, 18.9, 19.9, 20.3, 21.1, 21.2, 21.7, 21.7, 55, 55, 55.6, 55.1, 56.4, 56.8, 58.3, 57.8, 58.8, 60.5, 61.2, 62.5, 63.9, 65, 66.3, 67.7, 69.7, 71.7, 72.9, 74.7, 76.3, 77.8, 79.8, 80.7, 81.7, 84.2, 84.9, 86.1, 87.6, 89.6, 89.4, 90.3, 92.3, 92.1, 92.8, 93.8, 93.7, 94.4, 94.2, 94.8, 0.3, 0.8, 1.5, 2.1, 3.9, 4.3, 4.6, 5.9, 6.8, 7.1, 9, 9, 10.3, 11.1, 12.2, 12.6, 13.5, 13.5, 14.8, 15.5, 16.2, 16.9, 17.9, 18, 19.1, 19.2, 19.4, 19.9, 19.5, 20.6, 55.8, 57.5, 56.7, 58.3, 58.1, 58.5, 59.3, 61, 62.3, 62.6, 64.4, 65.2, 66.3, 68.4, 69.7, 71.7, 72.4, 73.4, 74.4, 76.4, 77.5, 78.5, 80.5, 81.9, 83.9, 84.6, 85.2, 87.3, 87.9, 90, 90.9, 91.3, 91.6, 92.3, 94, 93.8, 95.6, 94.9, 96.1, 95.6, 96, 0.3, 0.2, 2.3, 2.3, 3, 4, 5.3, 6.1, 6.7, 7, 8, 8.3, 9.3, 10, 10.1, 11.2, 11.7, 12.7
			, 13.2, 14.4, 15.3, 15.8, 16.1, 17.1, 17.2, 17.9, 18.9, 18.4, 19.5, 19.7, 19.2, 57.2, 58.2, 58.5, 58.4, 59.7, 60.6, 61.1, 62.3, 63.3, 65.2, 66.4, 67.7, 69, 70.7, 72.1, 73.4, 75.2, 75.9, 77.7, 78.5, 80.5, 81.9, 83.2, 85.6, 86.2, 88.3, 88.3, 90.7, 91.9, 92.1, 93.4, 95, 94.5, 94.9, 95.9, 96.2, 96.6, 96.9, 98.3, 98, 1.2, 1.9, 3.9, 2.8, 4.6, 5.3, 6.3, 7.2, 7.8, 9, 10.3, 10.6, 12, 11.8, 12.4, 13.2, 13.8, 14.2, 15.3, 15.8, 16, 16.6, 16.9, 17.5, 17.7, 18.4, 18, 58.9, 59.7, 60.3, 60.5, 60.8, 62.5, 63.7, 63.8, 64.5, 65.2, 67.7, 68.6, 70, 71.7, 73.4, 75, 76.1, 78.5, 79.8, 81.5, 82.9, 83.7, 85.6, 86.9, 88.3, 89.6, 90.8, 92, 93.2, 93.7, 94.7, 95.3, 96.1, 97.6, 97.7, 97.6, 98.3, 99.6, 99.5, 0.3, 0.9, 0.9, 3.6, 5.5, 6.3, 6.7, 7.4, 8.1, 8.6, 9.2, 11.1, 10.7, 12.4, 13.3, 13.6, 14, 14.9, 15.7, 15.8, 16, 16.6, 17.5, 17.6, 17.6, 59.8, 60.7, 60.7, 62.4, 62.8, 63.2, 64.4, 65, 65.9, 66.4, 67.6, 69.3, 70.6, 71.1, 73.2, 74.9, 76.8, 78.5, 80.8, 82.5, 83.9
			, 85.2, 87.3, 89.6, 90.3, 92.3, 93.4, 93.9, 94.1, 96.3, 96, 97, 98, 97.6, 98.3, 98.4, 99.3, 100.1, 99.8, 100.8, 100.9, 1.1, 3.2, 3.6, 4.4, 5.4, 6.8, 7.2, 8.4, 8.7, 9.7, 10.4, 11.3, 12.4, 12.3, 13.2, 14.2, 14.5, 14.6, 14.7, 15.8, 16, 16.5, 16.4, 61.3, 61.1, 62.2, 61.9, 63.1, 63.7, 64.5, 65.3, 66.6, 67.3, 68.9, 70.7, 71.7, 72.8, 74.7, 76.5, 77.2, 80.2, 81.9, 84.6, 85.6, 87.8, 88.9, 89.9, 91.6, 92.7, 93.6, 94.4, 95.8, 97.7, 97.5, 97.7, 99.3, 99.5, 100.4, 100.4, 101.5, 101, 101.4, 0.6, 0.6, 2.3, 3.9, 6, 6.2, 7.3, 7.7, 8.6, 9.9, 10.1, 10.6, 11.6, 12.6, 12.5, 13.2, 13.8, 13.9, 14.6, 14.7, 15.5, 15.8, 15.9, 62.5, 63.3, 63.8, 65.1, 64, 64.7, 65.6, 66.9, 68.4, 68.6, 70.4, 71.7, 72.7, 74.3, 75.8, 76.1, 78.5, 80.2, 82.9, 83.6, 86.6, 88, 90, 90.7, 92.3, 94.1, 95.7, 96.8, 97.2, 97.5, 98.9, 99.4, 100.5, 101.3, 101.5, 102.6, 102.2, 102.5, 0.2, 0.6, 2.3, 2.9, 4.4, 5, 5.8, 6.8, 7.3, 8.4, 9.1, 11.3, 11.5, 11.4, 12.7, 12.9, 13.6, 13.9, 14, 14.7, 15.3, 15.1
			, 63.9, 63.8, 65.1, 65.5, 66.3, 67.5, 67.9, 68, 68.6, 70.6, 71.1, 72.7, 74.6, 75.7, 77.1, 79.2, 79.8, 81.5, 83.5, 85.6, 86.9, 88.6, 89.6, 90.8, 91.7, 93.7, 95, 97.1, 96.7, 98.7, 100.4, 100.2, 100.9, 102, 102.3, 103.1, 103, 103.1, 104.1, 1, 0.2, 3.1, 3.2, 5, 6, 7.3, 7.9, 8.2, 8.5, 9.9, 10, 11.6, 11.2, 12.2, 12.6, 13.5, 13.8, 13.9, 14.3, 14.1, 64.8, 65.5, 65.4, 66.3, 67.2, 68.4, 68.2, 68.7, 70.3, 71.3, 72.9, 75, 76.5, 77.8, 79.2, 81.2, 83.1, 84.1, 86.3, 87.7, 89.6, 91.3, 92.1, 94, 94.9, 96, 98.4, 99.5, 99.7, 101.1, 102.1, 102.2, 103.2, 103.6, 104.3, 104.7, 105.3, 0.8, 0.1, 1.7, 2.8, 3.4, 5, 5.4, 6.6, 7.3, 8.2, 8.9, 9.9, 10.3, 10.4, 11, 11.3, 12, 12.1, 12.6, 13, 13.3, 14, 14, 66.7, 67.1, 66.8, 67.7, 68, 69.2, 69.7, 70.2, 72.4, 73.4, 74.4, 74.7, 76.5, 78.1, 79.8, 80.8, 82.2, 83.5, 85.8, 86.6, 88.3, 89.7, 90.8, 93, 95.1, 96.4, 97.7, 98.8, 100.1, 99.9, 102.4, 102.4, 104.3, 104.9, 105.1, 105.8, 105, 106.2, 106.2, 107.1, 106.5, 0.2, 0.1, 0, 2.2
			, 3.9, 5.2, 5.9, 6.4, 7.7, 7.7, 9, 9.3, 9.8, 10.4, 10.6, 10.8, 11.5, 11.4, 12.2, 12.4, 12.6, 13.5, 67.4, 68.2, 68.2, 68.8, 69.8, 69.8, 70.6, 71.3, 73.1, 74.7, 75.4, 76.5, 77.4, 79.2, 80.5, 82.2, 84.2, 85.2, 87.9, 90, 91, 92.3, 94.4, 95.1, 96, 98.4, 98.8, 100.1, 101.8, 101.5, 102.9, 103.6, 105.6, 105.3, 107, 106.5, 107.7, 108.5, 107.8, 107.9, 1.4, 2.1, 3.5, 4.4, 5.7, 6.4, 7.7, 8.6, 9.2, 10.3, 10.6, 10.8, 10.9, 11.6, 11.6, 11.5, 68.7, 68.8, 69.7, 70.2, 70.3, 71.1, 71.3, 72.6, 74, 75.5, 76.5, 77.8, 79.2, 79.8, 81.5, 83.7, 85.3, 85.9, 87, 90, 92, 93.7, 95.7, 97.1, 97.8, 99.8, 101.2, 101.3, 103.5, 103.7, 105, 106, 105.6, 107.7, 107.2, 107.7, 107.7, 109, 108.7, 108.7, 110.2, 109.2, 110.4, 1.6, 1.7, 1.2, 3, 4.4, 4.6, 6.3, 6.4, 7.1, 8.4, 8.9, 9.6, 9.9, 10.5, 10.6, 11.1, 10.9, 10.8, 69.9, 70.9, 71, 71.2, 71.7, 72, 72.9, 73.5, 75.3, 76.1, 78.1, 78.7, 80.5, 82.2, 83.5, 85.6, 86.9, 88, 90.3, 91, 92.7, 94.1, 95.7, 96.5, 99.1, 100.2, 101.4, 102.4, 104.5, 105.2
			, 105.1, 106.7, 107.6, 108, 109.9, 109.8, 110, 109.8, 110.5, 0.8, 0.2, 0.5, 1.3, 2.1, 3.3, 4.5, 6.1, 6.9, 7.4, 8.4, 9.1, 9.8, 9.8, 10.4, 10.5, 10.8, 70.9, 71.5, 72.3, 71.9, 73.2, 73.7, 75.1, 75.7, 77.2, 77.7, 79.2, 80.4, 82, 83.9, 84.3, 86.3, 88.1, 89, 90.1, 92.4, 94, 95.4, 96.8, 99.1, 100.5, 102.2, 102.3, 103.7, 105.9, 105.3, 107.9, 107.7, 108.2, 109.8, 110.5, 111.2, 110.7, 111.6, 111.8, 111.8, 111.9, 0.4, 0.3, 0.1, 1.2, 1.9, 2.9, 3, 3.2, 4.6, 5.2, 6.1, 7.6, 8.9, 9.2, 9.4, 10.3, 10.5, 10.4, 10.6, 72.7, 73.1, 74.4, 73.8, 75.2, 75.2, 76, 76.7, 77.4, 79.4, 81.5, 82.8, 83.1, 84.3, 86.3, 87.9, 88.6, 91, 93.3, 95.8, 97.4, 98.8, 100.8, 101.2, 102, 103.6, 105.6, 106.9, 106.6, 107.8, 109, 110.4, 109.9, 110.7, 112.6, 112.3, 112.7, 112.8, 113.4, 113.3, 114.6, 0.5, 1.1, 1.7, 2.5, 3.2, 4.4, 4.4, 5.1, 6.4, 7.1, 8.1, 8.2, 9, 9.2, 9.9, 10.4, 10.3, 73.3, 75.2, 74.4, 74.8, 74.9, 76, 76.5, 77.4, 78.5, 79.7, 81.3, 81.5, 83.2, 85.2, 85.5, 88.3, 89.7, 92.4, 93.7, 95.4, 97.4
			, 99.8, 101.1, 102.1, 103.9, 105.9, 106.5, 107.9, 108.5, 109.1, 110.4, 111.7, 112.4, 113.4, 114.1, 114.1, 115.5, 114.8, 115.5, 2.1, 2.4, 3.6, 3.2, 4.9, 5.7, 6.7, 6.8, 8.4, 9.2, 10.1, 10, 10.5, 74.6, 74.8, 75.8, 76.2, 76.4, 76.7, 78, 78.1, 79.6, 81.1, 82.6, 82.8, 84.2, 86.1, 86.6, 88.3, 90.3, 91.7, 93.3, 94.6, 95.1, 97.2, 98.8, 100, 102.5, 103.2, 104.3, 106.6, 107.9, 108.6, 110.3, 110.1, 111.2, 112.3, 111.7, 112.4, 112.9, 113.5, 113.8, 114.9, 115.6, 115.9, 116.3, 115.9, 116.1, 3.1, 3.4, 3.8, 4.6, 6.5, 7.1, 7.9, 9.1, 9.7, 10, 10.4, 10.3, 76, 76, 77.1, 76.7, 77.3, 78.1, 79, 79.9, 80.1, 81.8, 83.2, 83.7, 85.6, 86.5, 88.3, 90, 91.3, 93.3, 94.1, 95.5, 97.4, 98.8, 100.5, 101.5, 103.5, 104.3, 106.6, 107.9, 108.6, 110, 110.3, 112.4, 112.8, 114, 114.5, 115.3, 115.4, 116, 116.2, 116.7, 116.8, 117.2, 117.7, 3.6, 4.1, 4.5, 6.1, 7.3, 8.4, 10, 10.4, 10.3, 77.6, 77.7, 78, 78, 78.7, 79.8, 80.1, 81.3, 82.5, 82.7, 83.5, 85.6, 87.4, 87.6, 89, 90.5, 92.3, 94.1, 95.4, 95.8, 97.8, 99.8, 101.5, 103.2, 105.2
			, 106.1, 106.9, 107.8, 108.9, 110.6, 111.6, 112.5, 113.4, 115.8, 115.8, 116.4, 116.9, 117.4, 118.4, 118.6, 119.1, 118.7, 119.7, 119.9, 5.1, 5.4, 6.1, 7.7, 8.5, 9.7, 10.5, 10.7, 78.3, 78.7, 79.2, 80.6, 79.9, 80.1, 81.4, 82.4, 83.8, 84.9, 86, 87.2, 88.3, 90.3, 90.7, 92.5, 92.4, 94.4, 95.7, 97.8, 98.1, 100.1, 101.9, 103.8, 104.6, 106.5, 108.3, 109, 109.7, 111.3, 112.8, 114, 113.4, 114.8, 115.8, 117, 117.3, 117.3, 118, 118.9, 119.5, 120.4, 120.2, 120.9, 5.2, 7.1, 6, 7.2, 8.5, 9.6, 10.3, 11, 79.3, 80.2, 80.2, 81.3, 81.8, 82.1, 82.4, 83, 84.4, 86, 86.2, 87.6, 88.7, 89.7, 91.6, 93, 94, 95.8, 97.8, 99.4, 100.7, 102.8, 104, 105.9, 107.9, 109.3, 110.3, 111.7, 113.2, 114, 115.6, 116.6, 116.7, 117.3, 118.7, 119.1, 119.5, 120, 120.1, 121.1, 121.6, 121.6, 121.8, 122.7, 7, 7.2, 8.9, 10, 10.5, 10.9, 80.8, 81.9, 82, 82.6, 83.8, 83.6, 84.3, 86, 86.2, 87.1, 88.7, 89.8, 91.3, 92, 93.7, 95.4, 96.7, 98.8, 99.4, 100.5, 101.5, 103.9, 105.2, 106.6, 107.9, 108.8, 110.9, 111.9, 113.4, 114.8, 115, 117.1, 117.1, 118, 119, 119
			, 120.7, 121.4, 121.2, 121.2, 121.4, 122.1, 122.6, 123, 123.8, 123.9, 8.4, 8.1, 8.8, 9.9, 10.5, 10.5, 82.1, 82.1, 84, 83.1, 83.9, 84.5, 85.7, 85.7, 86.7, 88, 88.4, 89.9, 91.3, 91.8, 94.2, 95.4, 97.1, 99.1, 100.1, 101.5, 103.5, 105.4, 106.2, 107.3, 109.6, 111, 112.7, 113.5, 114.6, 116.1, 117.8, 117.8, 120.1, 119.9, 120.8, 121.9, 123.3, 123.2, 123.5, 124.5, 124, 124.2, 125.1, 10.1, 83.5, 84, 84.3, 84.4, 85.2, 86.7, 87.3, 87.1, 87.9, 88.4, 89.6, 90.9, 92.7, 93.3, 94.1, 96.4, 97.7, 99.4, 100.2, 101.3, 103.2, 104.6, 106.2, 107.6, 109.1, 111, 112.3, 113.7, 115.5, 117.1, 118.5, 118.6, 120.5, 120.5, 121, 121.7, 122.5, 123.2, 123.6, 124.2, 125.4, 126, 125.7, 125.7, 125.6, 126.7, 84.2, 85.4, 84.8, 86, 87.1, 86.8, 87.5, 88.2, 89.5, 90.7, 91.2, 93.4, 94.1, 96.1, 97, 98.1, 99.9, 101.8, 103.5, 105.2, 107.3, 108.8, 111, 112.3, 113, 114.9, 116.8, 117.8, 119, 120.8, 120.3, 121.8, 122.3, 123.3, 123.9, 124, 124.4, 125, 125.7, 126, 126.3, 126.7, 127, 127.7, 85.6, 86.3, 86.6, 87.9, 86.8, 88.1, 88.2, 88.9, 89.7, 91.1, 92.3, 94.2, 95.1, 96.4, 97.2
			, 99.4, 100.3, 102.5, 102.7, 105.5, 105.9, 107.3, 109.2, 111, 113.4, 114.4, 115.1, 117.4, 117.7, 120.1, 120.2, 122.6, 123.2, 124, 124.4, 125.4, 126.1, 126.3, 127.3, 127.7, 128.9, 128.1, 128.4, 129.2, 129.2, 87.4, 88, 88.1, 88.2, 88.2, 89.8, 90.9, 91.7, 92.2, 92.3, 93, 94.7, 95.6, 97.4, 97.8, 99.8, 100.2, 101.5, 103.7, 105.5, 106.4, 106.9, 109.3, 110.7, 112, 113.2, 115, 116.4, 117.8, 118.8, 120.5, 121.2, 121.6, 123.5, 124.6, 124.7, 125.2, 126.4, 127.3, 126.9, 127.4, 128.8, 128.6, 128.7, 130.1, 129.7, 129.6, 130.3, 87.5, 87.8, 88.6, 89.1, 88.9, 89.7, 90.6, 91.1, 92.2, 92.4, 93.7, 94.4, 96.2, 97.1, 98.1, 100.1, 101.4, 104.9, 106.9, 108.9, 111, 112.1, 113.7, 115, 116.1, 117.8, 118.7, 119.6, 121.8, 123.2, 124.6, 124.8, 126.9, 126.3, 127.4, 128.4, 129.5, 130.2, 130.4, 130.8, 130.6, 131.2, 131.1, 131.8, 132, 88.9, 90, 89.5, 90.8, 91.3, 90.9, 92.3, 93.3, 94, 95.5, 95.6, 97.2, 99, 100.5, 101.2, 101.5, 105.5, 108.3, 109.8, 112.3, 114.4, 115.7, 117.7, 120.2, 120.8, 122.5, 123.2, 124.9, 125.6, 126.1, 127.9, 127.6, 128.6, 129.4, 130.1, 130.8, 132.1, 131.9, 132.4, 132.5, 133.1, 90.3
			, 90.3, 91.5, 91.5, 92.4, 93, 93.9, 93.9, 95.1, 96, 97.5, 98.4, 99.6, 100.2, 102.5, 104.1, 107.3, 109.1, 110, 112, 113.7, 115.7, 116.8, 118.1, 120.8, 122.2, 122.6, 124.1, 125.7, 125.6, 127.4, 128.4, 129.2, 130.9, 130.5, 132.3, 131.8, 132.1, 132.4, 133.1, 91.7, 92, 92.3, 92.7, 93.6, 94.4, 94.5, 95.2, 96.1, 96.5, 98.8, 99.8, 100.3, 101.4, 102.1, 103.5, 106.2, 109.3, 112, 114.1, 116.4, 118.1, 119.5, 120.4, 121.5, 122, 123.5, 124.7, 126.6, 128, 128.3, 129.2, 129.4, 130.4, 131.4, 132.1, 133.1, 92.3, 94, 93.4, 92.9, 94.2, 95.5, 95.3, 96.8, 97.1, 98.8, 98.9, 101.5, 100.9, 102.5, 104.2, 106.2, 109.6, 113.7, 115.7, 118.1, 119.5, 120.9, 122.5, 124.3, 124.6, 126.3, 128.3, 129, 129.3, 130.8, 131.7, 131.6, 132.7, 133.4, 93.7, 94.5, 94.8, 95.3, 95.8, 95.7, 97, 97.6, 98.2, 100.2, 100.1, 100.5, 102.5, 103.2, 104.6, 106.2, 108.8, 112.3, 115.7, 118.1, 119.8, 122.2, 123.2, 125.9, 126.6, 127.4, 128.2, 129.7, 131.3, 131.7, 132.4, 132.9, 94.9, 95.5, 96.7, 96.3, 96.6, 97.2, 98.4, 98.9, 99.8, 100.9, 102, 103.6, 104.5, 107.1, 109.8, 113, 115, 117.7, 119.8, 121.2, 122.4, 123.3, 125.6
			, 126.5, 128.1, 129.6, 131.4, 131.6, 132.7, 95.6, 96.2, 96.8, 96.3, 97.5, 97.7, 98.8, 99.7, 100.6, 102.2, 102.4, 103.4, 104.2, 105.4, 108.3, 111.2, 115.7, 119.8, 122.2, 124.5, 124.3, 127.3, 129, 130.1, 130.9, 132.1, 97.2, 96.9, 98.2, 97.8, 98.6, 99.2, 99.7, 100.9, 100.7, 101.1, 102.6, 103.3, 103.8, 105.2, 106.2, 107.6, 110.3, 113, 116, 120.5, 123.6, 125.2, 126.9, 127.9, 130, 131.2, 132.4, 133, 97.9, 98.6, 98.6, 99.6, 100, 100.4, 101.1, 102.2, 103.1, 103.6, 104.2, 105, 106.3, 106.6, 107.6, 109.5, 112.8, 113.7, 117.7, 122.5, 124.3, 126.1, 127.5, 129.7, 131.1, 132.6, 133, 99.5, 100.2, 99.9, 100.3, 101.6, 101, 101, 102.8, 101.9, 103.2, 104.2, 104.9, 106.3, 106.9, 106.8, 108.3, 108.9, 111.8, 116.4, 120.5, 125.2, 127.7, 129.7, 130.4, 131.6, 133.1, 101.1, 101, 101.7, 101.7, 103, 103.1, 103.9, 104.3, 105.4, 105.2, 106.5, 107.2, 108.1, 109.3, 111.7, 114.1, 117.8, 120.7, 125.2, 128.5, 130.8, 132.4, 133.1, 102.4, 103.5, 103.5, 104, 104.7, 105.2, 105.4, 106.5, 107.6, 108.5, 109.1, 109.5, 111.8, 114.8, 117.2, 121.8, 125.9, 128.6, 131.3, 133.1, 103.3, 105, 103.9, 104.9, 105.6, 105.9, 106.6, 107.5, 107.8
			, 109.7, 109.3, 110.9, 111.7, 113.7, 116.4, 120.5, 123.8, 127.9, 130.6, 104.6, 105.6, 106.2, 106.1, 106.4, 107.2, 107.5, 108.4, 109, 110.2, 111.7, 111.9, 113.3, 114.8, 119.1, 121, 125.2, 128, 129.7, 132.7, 105.8, 107, 106.4, 107.4, 109, 108.6, 109.7, 110.7, 110.6, 112.4, 113.2, 113.4, 115.8, 117.1, 120.5, 123.9, 127.9, 130.4, 133.4, 107.4, 107.9, 108.3, 108.3, 109.4, 110.7, 109.9, 110.8, 112, 113.7, 113.9, 114.7, 116.9, 119.3, 121.9, 124.9, 128.6, 130.6, 108, 108.9, 108.5, 109.7, 110.4, 111.5, 111.4, 111.9, 113, 113.7, 114.2, 115.8, 118.4, 121.2, 123.7, 126.6, 127.8, 130, 132.7, 109.2, 111, 110.3, 110.9, 111.1, 111.2, 111.9, 112.6, 113.5, 114.3, 115.8, 116, 117.2, 120.5, 124.5, 126.6, 129.3, 131.9, 110.5, 110.7, 111.7, 112.4, 113, 112.6, 113.3, 114.6, 115.8, 116, 116.7, 117.3, 119.5, 123.2, 126, 130.6, 133.4, 111.5, 112.1, 113, 112.6, 113.8, 113.8, 114, 115.2, 116, 117.1, 117.4, 118.5, 120.5, 121.9, 125.6, 128.6, 132.7, 112.8, 113.6, 113.4, 114.3, 114, 115.2, 116.1, 116.5, 117.6, 118.1, 119, 120, 122.3, 124.7, 126.1, 130, 132.7, 113.9, 114.7, 115.2, 115.8, 115.9, 116.4, 117.2, 117.9, 118.3, 119.7
			, 119.8, 120.6, 122.2, 125.2, 128.6, 131.5, 115.3, 115.2, 116.4, 116.2, 116.6, 117.6, 118, 119.9, 120.1, 120.6, 121.6, 123.9, 125.9, 127.5, 129.2, 133.1, 116.6, 117.1, 117.9, 118.1, 118, 119.2, 119.7, 120.8, 121.2, 121.5, 122.9, 124.1, 128, 131.3, 117.3, 118.5, 118.4, 119, 119.9, 120.6, 120.7, 121.5, 123.2, 123.9, 124.6, 127.3, 129.3, 132, 118.9, 118.9, 119.7, 119.5, 120, 121.2, 121.4, 123.2, 123.4, 123.8, 124.2, 126.6, 129.3, 131.3, 120.2, 121, 121.3, 121.3, 121.6, 122.6, 123.1, 124.1, 124.7, 125, 126.5, 129.3, 132.7, 120.9, 121.7, 122, 122.3, 122.5, 123.3, 124.6, 124.8, 124.9, 126, 126.6, 128.4, 131.3, 122.1, 122, 122.9, 122.9, 123.8, 124.2, 124.2, 125.5, 126, 127.5, 128.7, 130.6, 123.7, 123.7, 125.3, 124.1, 125.5, 126, 126.8, 127, 128, 128.6, 129.1, 131.4, 133.5, 124.1, 125.4, 125, 126, 126.7, 127.3, 128.7, 128.2, 129.6, 129.9, 132.7, 126.5, 125.7, 126.7, 126.9, 127.8, 128.6, 128.3, 129.6, 130, 131.5, 126.8, 127.1, 128, 128.1, 128.7, 129.3, 130, 130.6, 131.4, 131.8, 133.4, 127.6, 128.7, 128.7, 129, 128.9, 129.6, 131.4, 131.9, 132.4, 133.4, 129.1, 129.5, 129.7, 130.5, 130.6, 131.5, 132.4, 132.5
			, 132.3, 133.3, 129.7, 130.8, 130.4, 131, 131.8, 131.7, 132.9, 130.9, 132.1, 132.1, 132.1, 132.3, 133.6, 131.9, 132.6, 132.4, 133.4, 133.5, 133.1, 127, 127.7, 105.3, 131.7, 130.5, 131.5, 105.6, 123.9, 125, 111, 122.5, 114.4, 104, 103.9, 109.5, 107, 120.6, 118.8, 121.6, 119.4, 117.1, 114.2, 104.1, 128.2, 126.1, 123.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
			, 0, 0, 0.4, 0, 0, 0, 0, 0, 0.6, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 102.1, 102.4, 103.2, 101.5, 105.8, 105.8, 105.9, 104.5, 104.6, 106.3, 109.9, 108.8, 108.1, 108.2, 108.3, 108.4, 109, 108, 108.2, 108.5, 107.8, 111.6, 112.8, 111.9, 112.2, 113.3, 112.7, 111.5, 112.2, 111.2, 112.8, 113.3, 112.6, 112.7, 111.4, 115.5, 115.7, 114.9, 116.6, 116.5, 115.6, 115.8, 116.2, 115.4, 116.7, 115.4, 116.3, 114.5, 119.9, 119.4, 117.9, 119.7, 118.1, 119.1, 118.5, 118.8, 119.5, 118.2, 118.2, 118.2, 119.6, 118.4, 119.1, 119.2, 118.7, 119.6, 120.2
			, 122.2, 123.1, 123.6, 121.4, 123.4, 121.9, 121.3, 121.3, 123.2, 123.4, 121.8, 123.2, 122.6, 121.9, 123.4, 122.4, 123.4, 122.4, 121.3, 123.6, 121.5, 121.2, 122.9, 124.7, 126.4, 126.1, 125.2, 126.8, 126.6, 124.7, 124.7, 126.7, 125.3, 125.1, 126, 126.8, 126.2, 125.5, 126.7, 124.8, 124.6, 124.6, 125.5, 126.9, 126.7, 128.6, 128.2, 130, 129.8, 128.4, 130.2, 129, 129.4, 129.3, 128.6, 129.6, 129.1, 128.9, 129.3, 129.4, 129.4, 129.7, 129, 128.5, 129.4, 129.3, 128.7, 128.9, 128.9, 129.5, 132.4, 133, 132, 132.1, 132.7, 132.8, 131.8, 132.9, 131.5, 132.2, 133.6, 133.6, 131.5, 132.6, 132.9, 132.4, 132.6, 133.5, 133.6, 132.8, 133, 131.9, 131.3, 133 };
		for (int i = 0; i < lLen; i++) LegLeo[i].z = LegLeoK[i] * zTbb;
		delete[]LegLeoK;
		double co = cos(fTbb);
		double si = sqrt(1 - co * co);
		for (int i = 0; i < lLen; i++)
		{
			LegLeo[i].x = xTbb + LegLeo[i].x * co - LegLeo[i].y * si;
			LegLeo[i].y = yTbb + LegLeo[i].x * si + LegLeo[i].y * co;
		}

		double LmaxX = LegLeo[0].x, LminX = LegLeo[0].x, LmaxY = LegLeo[0].y, LminY = LegLeo[0].y;

		for (int i = 1; i < lLen; i++)
		{
			if (LegLeo[i].x > LmaxX) LmaxX = LegLeo[i].x;
			if (LegLeo[i].x < LminX) LminX = LegLeo[i].x;
			if (LegLeo[i].y > LmaxY) LmaxY = LegLeo[i].y;
			if (LegLeo[i].y < LminY) LminY = LegLeo[i].y;
		}

		for (int z = 40 * VoxelS; z < VoxelZ; z++)
		{
			Vector3 Cen;
			double k = LminX + 0.6 * (LmaxX - LminY);
			int l = 0;
			for (int i = 0; i < lLen; i++) if (LegLeo[i].x < k && (LegLeo[i].z - k * VoxelS) < 5)
			{
				Cen = Cen + LegLeo[i];
				l++;
			}
			Cen = Cen / l;

			for (int x = -13; x <= 13; x++)for (int y = -13; y <= 13; y++) if (x*x + y * y < 13 * 13) voxel.Get(Cen.x + x, Cen.y + y, z) = 1;

			l = 360 / 6;
			int big = 360 * 4;

			double *voxelr = new double [big], *leor = new double[big];

			int del = big / l;

			for (int a = 0; a < l; a++)
			{
				double alpha = a * 2 * PI / (l);
				double co = cos(alpha), si = sqrt(1 - co * co), maxr = 0, tr;
				int oldx = Cen.x, oldy = Cen.y, x, y;
				for (int r = 1; r < 200; r++)
				{
					x = Cen.x + r * co;
					y = Cen.y + r * si;
					if (x == oldx && y == oldy) continue;
					else
					{
						if (x < 0 || x >= VoxelX || y < 0 || y >= VoxelY) break;
						if (voxel.USGet(x, y, z) == 0) break;
						tr = (Cen.x - x) * (Cen.x - x) + (Cen.y - y) * (Cen.y - y);
						if (tr > maxr) maxr = tr;
					}

				}
				voxelr[a * del] = maxr;

				double bestr2 = 15;
				double KritB = 1e20;
				for (int i = 0; i < lLen; i++) if (LegLeo[i].x < k && (LegLeo[i].z - k * VoxelS) < 5)
				{
					double r = (LegLeo[i].x - Cen.x) / (LegLeo[i].y - Cen.y);
					double beta = (abs(r) > 1000) ? r / abs(r) * PI / 2 : atan(r);
					if (LegLeo[i].x - Cen.x < 0) a += PI / 2;
					if (beta - alpha < 1/18*PI)
					{
						r = sqrt((LegLeo[i].x - Cen.x) * (LegLeo[i].x - Cen.x) + (LegLeo[i].y - Cen.y) * (LegLeo[i].y - Cen.y));
						double gamma = (r > 0) ? atan((LegLeo[i].z - z) / r) : 0;
						if ((LegLeo[i].x - Cen.x) < 0) gamma += PI;
						gamma = gamma * gamma + 4 * r * r;
						if (gamma < KritB)
						{
							bestr2 = r;
							KritB = gamma;
						}
					}
				}

				leor[a * del] = bestr2;

				if (a != 0)
				{
					for (int h = 1; h < del; h++)
					{
						voxelr[(a - 1) * del + h] = voxelr[(a - 1) * del] + h / (double)del * (voxelr[(a)* del] - voxelr[(a - 1) * del]);
						leor[(a - 1) * del + h] = leor[(a - 1) * del] + h / (double)del * (leor[(a)* del] - leor[(a - 1) * del]);
					}
				}
				if (a == l - 1)
				{
					for (int h = 1; h < del; h++)
					{
						voxelr[(a)* del + h] = voxelr[(a)* del] + h / (double)del * (voxelr[0] - voxelr[(a)* del]);
						leor[(a)* del + h] = leor[(a)* del] + h / (double)del * (leor[0] - leor[(a)* del]);
					}
				}
			}
			double r1 = 0, r2 = 0, r3 = 0, maxr = 0;
			for (int i = 0; i < big; i++)
			{
				r1 += voxelr[i];
				r2 += leor[i];
				r3 += abs(voxelr[i] - leor[i]);
				if (maxr < voxelr[i])maxr = voxelr[i];
				if (maxr < leor[i])maxr = leor[i];

			}

			double KO = 1 - exp(-0.4*sqrt(abs(r3 / ((r1 + r2) / 2 + 1e-20))));
			// Закладываем закономерность в радиус
			double KF = (1 - 1 / (1 + exp((k - 70) / 12))) / 0.92414;
			KF = KF + (1 - KF)*KO;
			double dx = Cen.x, dy = Cen.y;
			for (int x = 0; x < VoxelX; x++)for (int y = 0; y < VoxelY; y++) voxel.Get(Cen.x + x, Cen.y + y, z) = 0;

			for (int x = 0; x < VoxelX; x++, dx = Cen.x - x)for (int y = 0; y < VoxelY; y++, dy = (Cen.y - y)) if(dx * dx + dy * dy < maxr * maxr + 1)
			{
				double tg = (dx > 0) ? dy / dx : 1e20;
				double alpha = atan(tg); if (dx < 0) alpha += PI;
				int betak = alpha * big / (2 * PI);
				while (betak >= big) betak -= big;
				double r = voxelr[betak] + (leor[betak] - voxelr[betak]) * KF;
				r = r * r;
				if (r > dx * dx + dy * dy)
				{
					voxel.USGet(x, y, z) = 1;
				}
			}

			delete[]voxelr;
			delete[]leor;
		}

		if (oribb = -1) Orientation = 1;
		if (oribb = 1) Orientation = 2;

		blur.~Voxel();
	}

	void Centrovka()
	{

	}

	void MakeKakScaner()
	{
		// Код посметьева
		double *AF = new double[72];
		AF[0] = 0.000; AF[1] = 1.500; AF[2] = 1.495; AF[3] = 1.506; AF[4] = 1.525; AF[5] = 1.554; AF[6] = 1.599; AF[7] = 1.665; AF[8] = 1.753; AF[9] = 1.871;
		AF[10] = 2.020; AF[11] = 2.200; AF[12] = 2.435; AF[13] = 2.695; AF[14] = 2.976; AF[15] = 3.270; AF[16] = 3.565; AF[17] = 3.853; AF[18] = 4.125; AF[19] = 4.370;
		AF[20] = 4.579; AF[21] = 4.750; AF[22] = 4.849; AF[23] = 4.906; AF[24] = 4.920; AF[25] = 4.895; AF[26] = 4.840; AF[27] = 4.758; AF[28] = 4.656; AF[29] = 4.539;
		AF[30] = 4.414; AF[31] = 4.280; AF[32] = 4.162; AF[33] = 4.035; AF[34] = 3.899; AF[35] = 3.748; AF[36] = 3.576; AF[37] = 3.377; AF[38] = 3.145; AF[39] = 2.874;
		AF[40] = 2.557; AF[41] = 2.200; AF[42] = 1.761; AF[43] = 1.292; AF[44] = 0.795; AF[45] = 0.288; AF[46] = -0.212; AF[47] = -0.692; AF[48] = -1.136; AF[49] = -1.527;
		AF[50] = -1.849; AF[51] = -2.100; AF[52] = -2.221; AF[53] = -2.268; AF[54] = -2.239; AF[55] = -2.149; AF[56] = -2.009; AF[57] = -1.833; AF[58] = -1.632; AF[59] = -1.419;
		AF[60] = -1.207; AF[61] = -1.000; AF[62] = -0.835; AF[63] = -0.685; AF[64] = -0.556; AF[65] = -0.445; AF[66] = -0.347; AF[67] = -0.262; AF[68] = -0.187; AF[69] = -0.118;
		AF[70] = -0.055; AF[71] = 0.000;
		int begin = -1, end = -1;
		for (int x = 0; x < VoxelX; x++)for (int y = 0; y < VoxelY; y++)for (int z = 0; z < VoxelZ; z++)
		{
			if (begin == -1 && voxel.USGet(x, y, z) == 1) begin = x;
			if (end == -1 && voxel.USGet(VoxelX - 1 - x, y, z) == 1) end = x;
			if (begin == -1 && end == -1) { x = VoxelX; y = VoxelY; z = VoxelZ; }
		}
		double r = 0;
		uchar buff = 0, maskin = 0b00000010, maskout = 0b00000001;
		for (int x = begin; x <= end; x++)
		{
			r = (x - begin) / (double)(begin - end) * 100;
			if (r > 71) break;
			r = AF[(int)r] * VoxelS;
			for (int y = 0; y < VoxelY; y++)for (int z = 0; z < VoxelZ; z++)
			{
				buff = voxel.Get(x, y, z + (int)r);
				if (buff & maskout > 0)
					voxel.Get(x, y, z) = voxel.Get(x, y, z) | maskin;
			}
		}
		for (int x = begin; x <= end; x++)
		{
			for (int y = 0; y < VoxelY; y++)
			{
				for (int z = 0; z < VoxelZ; z++)
				{
					buff = voxel.Get(x, y, z);
					if (buff & maskin > 0 || (z < 3 && voxel.Get(x, y, 0) & maskout > 0))
						buff = maskout;
				}
			}
		}
	}

	void Podgonka()
	{

	}

#pragma endregion

#pragma region Inits and Prepare

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

	void InitUserStat(bool male)
	{
		ismale = male;
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

#pragma endregion

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