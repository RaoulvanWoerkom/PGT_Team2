#pragma once
#include <string>

class Highscore
{
public:
	Highscore();
	~Highscore();
	void addToScoreboard(double score);
	static double getHighscore();
	bool checkIfExceedsPreviousHighscore(double score);
};

