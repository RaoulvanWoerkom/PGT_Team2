#pragma once
#include <string>

class Highscore
{
public:
	Highscore();
	~Highscore();
	static void Highscore::addToScoreboard(int score);
	static int Highscore::getHighscore();
	static bool Highscore::checkIfExceedsPreviousHighscore(int score);
	static int Highscore::getHighscoreFromFile();

};

