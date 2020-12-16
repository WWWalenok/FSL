
#include "FSL.h"
using namespace fsl;

uchar& Img::Get(int _x, int _y)
{
	_x = (_x >= x) ? x - 1 : ((_x < 0) ? 0 : x);
	_y = (_y >= y) ? y - 1 : ((_y < 0) ? 0 : y);
	return val[_x + x * _y];
}

void Img::Set(int _x, int _y, uchar value)
{
	_x = (_x >= x) ? x - 1 : ((_x < 0) ? 0 : x);
	_y = (_y >= y) ? y - 1 : ((_y < 0) ? 0 : y);
	val[_x + x * _y] = value;
}

uchar& Img::USGet(int _x, int _y)
{
	return val[_x + x * _y];
}

void Img::USSet(int _x, int _y, uchar value)
{
	val[_x + x * _y] = value;
}

void Img::Clear() { for (int i = 0; i < l; i++) val[i] = 0; }

int Img::X() { return x; }

int Img::Y() { return y; }

int Img::L() { return l; }

unsigned char *&Img::Value()
{
	return val;
}

Img::Img(int x, int y)
{
	Img::x = x;
	Img::y = y;
	l = x * y;
	val = new unsigned char[l];
	Clear();
}

Img::Img(int x, int y, unsigned char* frame, int oor = 0)
{
	Img::x = x;
	Img::y = y;
	l = x * y;
	val = new unsigned char[l];
	for (int i = 0; i < x; i++) for (int j = 0; j < y; j++) val[i + x * j] = frame[i + x * j];
}

Img::Img(int x, int y, unsigned char** frame, int oor = 0)
{
	Img::x = x;
	Img::y = y;
	l = x * y;
	val = new unsigned char[l];
	for (int i = 0; i < x; i++) for (int j = 0; j < y; j++) val[i + x * j] = frame[i][j];
}

~Img::Img()
{
	delete[] val;
}