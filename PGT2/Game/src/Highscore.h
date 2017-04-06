#pragma once
#include <string>

class Highscore
{
public:
	Highscore();
	~Highscore();
	void addToScoreboard(std::string name, double score);
};

