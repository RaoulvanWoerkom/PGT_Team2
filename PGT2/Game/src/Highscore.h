#pragma once
#include <string>

#include "BaseApplication.h"

class Highscore
{
public:
	Highscore();
	~Highscore();
	bool addToScoreboard(std::string name, double score);
};

