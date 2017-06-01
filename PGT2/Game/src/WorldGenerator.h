#ifndef __WorldGenerator_h_

#define __WorldGenerator_h_


class WorldGenerator
{
public:
	WorldGenerator();
	virtual ~WorldGenerator();
	void Seed(int seed);
	void Generate();
	void PlaceMeshes();

private:
	double fade(double t);
	double lerp(double t, double a, double b);
	double grad(int hash, double x, double y, double z);
};

#endif // #ifndef __WorldGenerator_h_