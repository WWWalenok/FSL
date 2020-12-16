#pragma once

#include <iostream>
#include <vector>
#include <thread>
#include <cmath>

namespace fsl
{

#define PI 3.141592653589793
#define VoxelX 400
#define VoxelY 256
#define VoxelZ 256
#define VoxelS 1.0
#define pointsDisp 16
#define pointsDeep 3

#define random rand() / (double)RAND_MAX

#define uchar unsigned char

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

	class Img
	{
	public:

		uchar& Get(int _x, int _y);

		void Set(int _x, int _y, uchar value);

		uchar& USGet(int _x, int _y);

		void USSet(int _x, int _y, uchar value);

		void Clear() { for (int i = 0; i < l; i++) val[i] = 0; }

		int X() { return x; }

		int Y() { return y; }

		int L() { return l; }

		unsigned char *&Value();

		Img(int x, int y);

		Img(int x, int y, unsigned char* frame, int oor = 0);

		Img(int x, int y, unsigned char** frame, int oor = 0);

		~Img();
	private:

		unsigned char *val;
		int x, y, l;
	};

	class Voxel
	{
		unsigned char *voxel;

		int sx, sy, sz;

	public:

		unsigned char &Get(int x, int y, int z);
		void Set(int x, int y, int z, unsigned char value);
		unsigned char &USGet(int x, int y, int z);
		void USSet(int x, int y, int z, unsigned char value);
		Voxel(int x, int y, int z);
		~Voxel();
	};

	struct PhisicPoint
	{
		Vector2 loc, v, dv;
	};

	
	void InitEtalon(std::vector<Vector3*> _male, std::vector<int> _malesizes, std::vector<Vector3*> _female, std::vector<int> _femalesizes);

	void InitFrame(int inframecount, unsigned char ***inframes, int frameX, int frameY);


	void Run();
}