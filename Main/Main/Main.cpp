// Main.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <thread>

namespace fsl
{
	using namespace std;

	void GetFirsPos(double** ret, int*** imgs, int fcount, int maxX, int maxY)
	{
		double IMax[7], IMin[7], TMin[7], TMax[7];
	}


	void MainFunc(int thcount, int*** imgs, int fcount, int maxX, int maxY)
	{
		int*** buffer = new int**[fcount*4], int **img;
		for (int i = 0; i < fcount*4; i++) { buffer[i] = new int*[maxX]; for (int j = 0; j < maxX; j++) buffer[i][j] = new int[maxY]; }

		thread **threads = new thread *[thcount - 1];

		double gisto[256];

		for (int o = 0; o < fcount; o++)
		{
			img = imgs[o];
			for (int i = 0; i < maxX; i++)for (int j = 0; j < maxY; j++)
				gisto[(img[i][j] < 0) ? 0 : ((img[i][j] > 255) ? 255 : img[i][j])]++;

			for (int g = 0; g < 255; g++) gisto[g] /= (double)maxX*maxY;

			double t = 0.08, d = 0;
			int u = 0;
			while (t > 0)
			{
				t -= gisto[u];
				u++;
			}
			t = 0.08;
			int q = 255;
			while (t > 0)
			{
				t -= gisto[q];
				q--;
			}
			d = 1 + (u + q) / 256.0;
			for (int i = 0; i < maxX; i++) for (int j = 0; j < maxY; j++)
			{
				t = (img[i][j] - u) * d;
				if (t > 255) t = 255;
				if (t < 0) t = 0;
				buffer[o][i][j] = t;
			}

			for (int i = 0; i < maxX; i++) for (int j = 0; j < maxY; j++)
			{
				t = 0;
				u = 0;
				for (int _i = -2; _i < 3; _i++) for (int _j = -2; _j < 3; _j++)
					if (i + _i >= 0 && i + _i < maxX && j + _j >= 0 && j + _j < maxY) { t += buffer[o][i][j]; u++; }
				buffer[o + fcount][i][j] = t / (double)u;
			}
		}

		double ***rets = new double**[thcount];
		int ****inframes = new int***[thcount];
		int *framecounts = new int[thcount];
		
		int k = fcount;
		while (k > 0)
		{
			for (int i = 0; i < thcount; i++) if (k > 0) { framecounts[i]++; k--; }
		}
		k = 0;
		for (int i = 0; i < thcount; i++) { inframes[i] = new int**[framecounts[i]]; for (int j = 0; j < framecounts[i]; j++) { inframes[i][j] = buffer[k + fcount]; k++; } }

		for (int th = 0; th < thcount; th++)
		{
			threads[th] = new thread{ GetFirsPos ,std::ref(rets[th+1]) ,  std::ref(inframes[th + 1]),  framecounts[th + 1], maxX, maxY };
		}

		GetFirsPos(std::ref(rets[0]), std::ref(inframes[0]), framecounts[0], maxX, maxY);


	}
}

using namespace fsl;

int main()
{
    
}