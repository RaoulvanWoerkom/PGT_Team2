#include "WorldGenerator.h"
#include <stdlib.h>

#include <iostream>
#include <string>
#include <iomanip>
#include "Helper.h"

using std::cout;

int _seed = 1;

WorldGenerator::WorldGenerator()
{
	
}


WorldGenerator::~WorldGenerator()
{

}

void WorldGenerator::Seed(int seed) 
{
	_seed = seed;
}

void WorldGenerator::Generate() 
{
	srand(_seed);
	Helper help;
	int sreed;
	srand(_seed);
	sreed = rand() % 10;
	srand(time(NULL));
	sreed = rand() % 10;
	srand(_seed);
	sreed = rand() % 10;
	srand(time(NULL));
	sreed = rand() % 10;
}

void WorldGenerator::PlaceMeshes()
{

}