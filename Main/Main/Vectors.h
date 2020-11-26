#pragma once
#include <iostream>


struct Vector3
{
	double x, y, z;

	Vector3();

	Vector3(double _x, double _y, double _z);

	Vector3 operator +(Vector3 a);

	Vector3 operator -(Vector3 a);

	Vector3 operator *(double a);

	double operator *(Vector3 a);

	Vector3 operator /(Vector3 a);

	Vector3 operator /(double a);

	void SetLenght(double l);

	double GetLenght();
};

struct Vector2
{
	double x, y;

	Vector2();

	Vector2(double _x, double _y);

	Vector2 operator +(Vector2 a);

	Vector2 operator -(Vector2 a);

	Vector2 operator *(double a);

	double operator *(Vector2 a);

	double operator /(Vector2 a);

	Vector2 operator /(double a);

	Vector2 operator !();

	void SetLenght(double l);

	double GetLenght();
};

struct Quaternion
{
	double x, y, z, w;

	Quaternion();

	Quaternion(double _x, double _y, double _z, double _w);

	Quaternion operator +(Quaternion a);

	Quaternion operator -(Quaternion a);

	Quaternion operator *(double a);

	Quaternion operator *(Quaternion a);

	Quaternion operator *(Vector3 a);

	Quaternion operator /(double a);

	Quaternion operator /(Quaternion a);

	Vector3 operator &(Vector3 a);

	Quaternion operator !();

	Quaternion operator ~();

	void SetLenght(double l);

	double GetLenght();
};

