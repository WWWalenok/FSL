#include "FSL.h"

using namespace std;

namespace fsl
{

	float Q_rsqrt(float number)
	{
		const float x2 = number * 0.5F;
		const float threehalfs = 1.5F;

		union {
			float f;
			uint32_t i;
		} conv = { number }; // member 'f' set to value of 'number'.
		conv.i = 0x5f3759df - (conv.i >> 1);
		conv.f *= threehalfs - x2 * conv.f * conv.f;
		return conv.f;
	}

	Vector3::Vector3()
	{
		x = 0;
		y = 0;
		z = 0;
	}

	Vector3::Vector3(float _x, float _y, float _z)
	{
		x = _x;
		y = _y;
		z = _z;
	}

	Vector3 Vector3::operator +(Vector3 a) { return Vector3(a.x + x, a.y + y, a.z + z); }

	Vector3 Vector3::operator -(Vector3 a) { return Vector3(-a.x + x, -a.y + y, -a.z + z); }

	Vector3 Vector3::operator *(float a) { return Vector3(x * a, y * a, z * a); }

	float Vector3::operator *(Vector3 a) { return a.x * x + a.y * y + a.z * z; }

	Vector3 Vector3::operator /(Vector3 a) { return Vector3(y * a.z - z * a.y, -x * a.z + z * a.x, x * a.y - y * a.x); }

	Vector3 Vector3::operator /(float a) { return Vector3(x / a, y / a, z / a); }

	float Vector3::GetLenght()
	{
		return std::sqrt(x*x + y * y + z * z);
	}

	void Vector3::SetLenght(float d)
	{
		d = d * Q_rsqrt(x * x + y * y + z * z);
		x *= d;
		y *= d;
		z *= d;
	}


	Vector2::Vector2()
	{
		x = 0;
		y = 0;
	}

	Vector2::Vector2(float _x, float _y)
	{
		x = _x;
		y = _y;
	}

	Vector2 Vector2::operator +(Vector2 a) { return Vector2(a.x + x, a.y + y); }

	Vector2 Vector2::operator -(Vector2 a) { return Vector2(-a.x + x, -a.y + y); }

	Vector2 Vector2::operator *(float a) { return Vector2(x * a, y * a); }

	float Vector2::operator *(Vector2 a) { return a.x * x + a.y * y; }

	float Vector2::operator /(Vector2 a) { return x * a.y - y * a.x; }

	Vector2 Vector2::operator /(float a) { return Vector2(x / a, y / a); }

	Vector2 Vector2::operator !() { return Vector2(y, -x); }

	float Vector2::GetLenght()
	{

		return std::sqrt(x*x + y * y);

	}

	void Vector2::SetLenght(float d)
	{
		d = d * Q_rsqrt(x*x + y * y);
		x *= d;
		y *= d;
	}

	Quaternion::Quaternion()
	{
		x = 0;
		y = 0;
	}

	Quaternion::Quaternion(float _x, float _y, float _z, float _w)
	{
		x = _x;
		y = _y;
		z = _z;
		w = _w;
	}

	Quaternion Quaternion::operator +(Quaternion a) { return Quaternion(a.x + x, a.y + y, a.z + z, a.w + w); }

	Quaternion Quaternion::operator -(Quaternion a) { return Quaternion(a.x - x, a.y - y, a.z - z, a.w - w); }

	Quaternion Quaternion::operator *(float a) { return Quaternion(x * a, y * a, z * a, w * a); }

	Quaternion Quaternion::operator *(Quaternion a) { return Quaternion(w * a.w - (x * a.x + y * a.y + z * a.z), w * a.x + x * a.w + y * a.z - z * a.y, w * a.y + y * a.w + z * a.x - x * a.z, w * a.z + z * a.w + x * a.y - y * a.x); }

	Quaternion Quaternion::operator *(Vector3 a) { return Quaternion(-(x * a.x + y * a.y + z * a.z), w * a.x + y * a.z - z * a.y, w * a.y + z * a.x - x * a.z, w * a.z + x * a.y - y * a.x); }

	Quaternion Quaternion::operator /(Quaternion a) { return *this * ~a; }

	Vector3 Quaternion::operator &(Vector3 a) { Quaternion q = (*this * a) * ~*this; return Vector3(q.x, q.y, q.z); }

	Quaternion Quaternion::operator /(float a) { return Quaternion(x / a, y / a, z / a, w / a); }

	Quaternion Quaternion::operator !() { return Quaternion(-x, -y, -z, w); }

	Quaternion Quaternion::operator ~() { float d = 1 / (x * x + y * y + z * z + w * w); return Quaternion(-x * d, -y * d, -z * d, w * d); }

	float Quaternion::GetLenght()
	{

		return std::sqrt(x * x + y * y + z * +w * w);

	}

	void Quaternion::SetLenght(float d)
	{
		d = d * Q_rsqrt(x*x+y*y+z*z+ w * w);
		x *= d;
		y *= d;
		z *= d;
		w *= d;
	}
}