
#include "FSL.h"

namespace fsl
{

	uch& Img::Get(int _x, int _y)
	{
		_x = (_x >= x) ? x - 1 : ((_x < 0) ? 0 : _x);
		_y = (_y >= y) ? y - 1 : ((_y < 0) ? 0 : _y);
		return val[_x + x * _y];
	}

	void Img::Set(int _x, int _y, uch value)
	{
		_x = (_x >= x) ? x - 1 : ((_x < 0) ? 0 : _x);
		_y = (_y >= y) ? y - 1 : ((_y < 0) ? 0 : _y);
		val[_x + x * _y] = value;
	}

	uch& Img::USGet(int _x, int _y)
	{
		return val[_x + x * _y];
	}

	void Img::USSet(int _x, int _y, uch value)
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

	Img::Img(int x, int y, unsigned char* frame, int oor)
	{
		Img::x = x;
		Img::y = y;
		l = x * y;
		val = new unsigned char[l];
		for (int i = 0; i < x; i++) for (int j = 0; j < y; j++) val[i + x * j] = frame[i + x * j];
	}

	Img::Img(int x, int y, unsigned char** frame, int oor)
	{

		l = x * y;
		val = new unsigned char[l];
		for (int i = 0; i < x; i++) for (int j = 0; j < y; j++) val[i + x * j] = frame[i][j];
		Img::x = x;
		Img::y = y;
	}

	/*Img::~Img()
	{
		if(val != nullptr)
			delete[] val;
	}*/
}