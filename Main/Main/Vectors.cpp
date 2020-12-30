#include "FSL.h"

using namespace std;

namespace fsl
{

	Vector3::Vector3()
	{
		x = 0;
		y = 0;
		z = 0;
	}

	Vector3::Vector3(double _x, double _y, double _z)
	{
		x = _x;
		y = _y;
		z = _z;
	}

	Vector3 Vector3::operator +(Vector3 a) { return Vector3(a.x + x, a.y + y, a.z + z); }

	Vector3 Vector3::operator -(Vector3 a) { return Vector3(-a.x + x, -a.y + y, -a.z + z); }

	Vector3 Vector3::operator *(double a) { return Vector3(x * a, y * a, z * a); }

	double Vector3::operator *(Vector3 a) { return a.x * x + a.y * y + a.z * z; }

	Vector3 Vector3::operator /(Vector3 a) { return Vector3(y * a.z - z * a.y, -x * a.z + z * a.x, x * a.y - y * a.x); }

	Vector3 Vector3::operator /(double a) { return Vector3(x / a, y / a, z / a); }

	double Vector3::GetLenght()
	{
		return std::sqrt(x*x + y * y + z * z);
	}

	void Vector3::SetLenght(double d)
	{
		d = d / GetLenght();
		x *= d;
		y *= d;
		z *= d;
	}


	Vector2::Vector2()
	{
		x = 0;
		y = 0;
	}

	Vector2::Vector2(double _x, double _y)
	{
		x = _x;
		y = _y;
	}

	Vector2 Vector2::operator +(Vector2 a) { return Vector2(a.x + x, a.y + y); }

	Vector2 Vector2::operator -(Vector2 a) { return Vector2(a.x - x, a.y - y); }

	Vector2 Vector2::operator *(double a) { return Vector2(x * a, y * a); }

	double Vector2::operator *(Vector2 a) { return a.x * x + a.y * y; }

	double Vector2::operator /(Vector2 a) { return x * a.y - y * a.x; }

	Vector2 Vector2::operator /(double a) { return Vector2(x / a, y / a); }

	Vector2 Vector2::operator !() { return Vector2(y, -x); }

	double Vector2::GetLenght()
	{

		return std::sqrt(x*x + y * y);

	}

	void Vector2::SetLenght(double d)
	{
		if(x != 0 && y != 0) d = d / GetLenght();
		x *= d;
		y *= d;
	}

	Quaternion::Quaternion()
	{
		x = 0;
		y = 0;
	}

	Quaternion::Quaternion(double _x, double _y, double _z, double _w)
	{
		x = _x;
		y = _y;
		z = _z;
		w = _w;
	}

	Quaternion Quaternion::operator +(Quaternion a) { return Quaternion(a.x + x, a.y + y, a.z + z, a.w + w); }

	Quaternion Quaternion::operator -(Quaternion a) { return Quaternion(a.x - x, a.y - y, a.z - z, a.w - w); }

	Quaternion Quaternion::operator *(double a) { return Quaternion(x * a, y * a, z * a, w * a); }

	Quaternion Quaternion::operator *(Quaternion a) { return Quaternion(w * a.w - (x * a.x + y * a.y + z * a.z), w * a.x + x * a.w + y * a.z - z * a.y, w * a.y + y * a.w + z * a.x - x * a.z, w * a.z + z * a.w + x * a.y - y * a.x); }

	Quaternion Quaternion::operator *(Vector3 a) { return Quaternion(-(x * a.x + y * a.y + z * a.z), w * a.x + y * a.z - z * a.y, w * a.y + z * a.x - x * a.z, w * a.z + x * a.y - y * a.x); }

	Quaternion Quaternion::operator /(Quaternion a) { return *this * ~a; }

	Vector3 Quaternion::operator &(Vector3 a) { Quaternion q = (*this * a) * ~*this; return Vector3(q.x, q.y, q.z); }

	Quaternion Quaternion::operator /(double a) { return Quaternion(x / a, y / a, z / a, w / a); }

	Quaternion Quaternion::operator !() { return Quaternion(-x, -y, -z, w); }

	Quaternion Quaternion::operator ~() { double d = 1 / (x * x + y * y + z * z + w * w); return Quaternion(-x * d, -y * d, -z * d, w * d); }

	double Quaternion::GetLenght()
	{

		return std::sqrt(x * x + y * y + z * +w * w);

	}

	void Quaternion::SetLenght(double d)
	{
		d = d / GetLenght();
		x *= d;
		y *= d;
		z *= d;
		w *= d;
	}
}