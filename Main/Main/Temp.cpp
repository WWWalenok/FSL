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
//	float minr = 1e30, temp = 0;
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

//float **buffer = new float*[100];
//for (int i = 0; i < 100; i++)
//{
//	buffer[i] = new float[100];
//}
//float x, y, dx = maxX - minX, dy = maxY - minY; dx /= 100.0; dy /= 100.0;
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


	//void GetCamPos(int u)
	//{
	//	Vector3 AT, BT, CT, DT, O;
	//	float dig = std::sqrt(immaxX * immaxX + immaxY * immaxY);
	//	float t = 43.26661530556787151743 / dig;
	//	Vector3 A(lists[u][0].x, lists[u][0].y, 0), B(lists[u][1].x, lists[u][1].y, 0), C(lists[u][2].x, lists[u][2].y, 0), D(lists[u][3].x, lists[u][3].y, 0), R1(immaxX / 2.0, immaxY / 2.0, 0), R2, R3, R4;
	//	A = (A - R1) * t; B = (B - R1) * t; C = (C - R1) * t; D = (D - R1) * t;
	//	R1 = A - B;
	//	R2 = B - C;
	//	R3 = C - D;
	//	R4 = D - A;
	//	float a, b, c, d, d1, d2, l1, l2, dmax = 0;
	//	float M[3][4], maxerr = 1e30;
	//	float lamda = 50;
	//	if (min(R1 * R1, R3 * R3) < min(R2 * R2, R4 * R4))
	//	{
	//		O = A;
	//		A = B; B = C; C = D; D = O;
	//	}
	//	int rot = 0;
	//	d = 6;
	//	Vector3 AO = A, BO = B, CO = C, DO = D;
	//	M[0][0] = 1; M[0][1] = -1; M[0][2] = 1; M[0][3] = 1;
	//	M[1][0] = A.x; M[1][1] = -B.x; M[1][2] = C.x; M[1][3] = D.x;
	//	M[2][0] = A.y; M[2][1] = -B.y; M[2][2] = C.y; M[2][3] = D.y;
	//	for (int j = 0; j < 3; j++)
	//		for (int k = 0; k < 3; k++)if (k != j)
	//		{
	//			t = M[k][j] / M[j][j];
	//			for (int i = j; i < 4; i++)
	//			{
	//				M[k][i] = M[k][i] - M[j][i] * t;
	//			}
	//		}
	//	for (int j = 0; j < 3; j++)
	//	{
	//		t = 1 / M[j][j];
	//		M[j][j] = 1;
	//		M[j][3] *= t;
	//	}
	//	A.z = 0; B.z = 0; C.z = 0; D.z = 0;
	//	a = M[0][3]; b = M[1][3]; c = M[2][3];
	//	lamda = -((D - A * a) * (D - C * c)) / (1 - a - c + a * c);
	//	lamda = sqrt(abs(lamda));
	//	A.z = lamda; B.z = lamda; C.z = lamda, D.z = lamda;
	//	R1 = A * M[0][3] - B * M[1][3]; R2 = C * M[2][3] - B * M[1][3];
	//	l1 = sqrt(R1*R1);
	//	l2 = sqrt(R2*R2);
	//	d = l1 / l2;
	//	if (rot == 4) rot = 0;
	//	R1 = Vector3(1, d, 0);
	//	R2 = Vector3(1, -d, 0);
	//	d1 = -(297 + d * 210) / (1 + d * d);
	//	d2 = -(297 - d * 210) / (1 + d * d);
	//	R1 = R1 * d1;
	//	R2 = R2 * d2;
	//	d1 = R1.GetLenght();
	//	d2 = R2.GetLenght();
	//	if (d1 < d2)
	//	{
	//		d1 = R1.x;
	//		d2 = R1.y;
	//	}
	//	else
	//	{
	//		d1 = R2.x;
	//		d2 = R2.y;
	//	}
	//	float t1 = (297 + d1) / l1, t2 = (210 + d2) / l2;
	//	d = (t1 + t2) / 2.0;
	//	switch (rot)
	//	{
	//	case 1:
	//		t = d;
	//		d = c * t; c = b * t; b = a * t; a = t;
	//		break;
	//	case 2:
	//		t = d;
	//		t1 = a;
	//		a = c * t; c = t1 * t; d = b * t; b = t;
	//		break;
	//	case 3:
	//		t = d;
	//		d = a * t; a = b * t; b = c * t; c = t;
	//		break;
	//	case 0:
	//		a = a * d;	b = b * d;	c = c * d;	d = d;
	//		break;
	//	default:
	//		break;
	//	}
	//	A = AO; B = BO; C = CO; D = DO;
	//	A.z = lamda; B.z = lamda; C.z = lamda, D.z = lamda;
	//	R1 = A * a - B * b;
	//	R2 = C * c - B * b;
	//	AT = A * a;
	//	BT = B * b;
	//	CT = C * c;
	//	DT = D * d;
	//	O = (AT + BT + CT + DT) / 4.0;
	//	R1 = AT - BT;
	//	R1 = R1 / sqrt(R1*R1);
	//	R3 = R1 / (DT - AT);
	//	R3.SetLenght((R3.z > 0) ? 1 : -1);
	//	R2 = R1 / R3;
	//	AT = AT - O;
	//	BT = BT - O;
	//	CT = CT - O;
	//	DT = DT - O;
	//	O = Vector3(O * R1, O * R2, O * R3);
	//	Vector3 WA(AT * R1, AT * R2, AT * R3), WB(BT * R1, BT * R2, BT * R3), WC(CT * R1, CT * R2, CT * R3), WD(DT * R1, DT * R2, DT * R3);
	//	Vector3 X(1, 0, 0), Y(0, 1, 0), Z(0, 0, 1);
	//	X = Vector3(X * R1, X * R2, X * R3);
	//	Y = Vector3(Y * R1, Y * R2, Y * R3);
	//	Z = Vector3(Z * R1, Z * R2, Z * R3);
	//	while (inworld.size() <= u)
	//	{
	//		Vector3* temp = new Vector3[4];
	//		inworld.push_back(temp);
	//	}
	//	inworld[u][0] = WA; inworld[u][1] = WB; inworld[u][2] = WC; inworld[u][3] = WD;
	//	focuss[u] = lamda;
	//	cams[u][0] = O; cams[u][1] = X; cams[u][2] = Y; cams[u][3] = Z;
	//}

/*

					float fi = 0;
					float k = 1;
					bool isCorner = false;
					V2 = (points[i2].loc + points[i0].loc) * 0.5;
					V2 = V2 - points[i1].loc;
					Y = points[i2].loc - points[i0].loc;
					if (Y.x == 0 && Y.y == 0) Y = points[i1].loc * -1;
					Y.SetLenght(1);
					X = (!Y) * -1;
					for (int i = -3; i < 4; i++)
					{
						V1 = points[i1].loc + X * ots[abs(i)] * ((i > 0) ? 1 : -1);
						t = imgs[u].Get((int)V1.x, (int)V1.y);
						buff[i + 3] = (t);
					}
					if (((points[i2].loc.x - points[i0].loc.x)*(points[i1].loc.y - points[i0].loc.y) - (points[i2].loc.y - points[i0].loc.y)*(points[i1].loc.x - points[i0].loc.x)) > 0) k = -1; else k = 1;
					for (int i = 0; i < 4; i++)if (i != deadline) if (abs(lineK[i][0] * points[i1].loc.x + lineK[i][1] * points[i1].loc.y + lineK[i][2]) < 30) isCorner = true;
					if (!isCorner)
						fi = g * (0.6 * (buff[2] - 2 * buff[3] + buff[4]) + 0.25 * (buff[1] - 2 * buff[3] + buff[5]) + 0.15 * (buff[0] - 2 * buff[3] + buff[6]))*k;
					if (abs(buff[0] - buff[4]) + abs(buff[1] - buff[5]) + abs(buff[0] - buff[6]) < 40 * 3) if (buff[1] < 40 && buff[5] < 40) fi += 1500 * k;
					//points[i1].dv = V2 * fi + V3 * 5.0;
					V1 = points[i2].loc - points[i1].loc;
					V2 = points[i0].loc - points[i1].loc;
					V3 = (points[i2].v - points[i1].v * 2 + points[i0].v);
					r = V1.GetLenght();
					V4 = Vector2(0, 0);
					if (r != 0) V4 = V1 * -c * (r - rmid) / r;
					r = V2.GetLenght();
					if (r != 0) V4 = V4 + V2 * -c * (r - rmid) / r;
					points[i1].dv = points[i1].dv + V4 + V3 * d;

*/