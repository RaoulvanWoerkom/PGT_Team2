#include "Highscore.h"
#include <time.h>
#include <iostream>
#include <fstream>
using namespace std;

namespace highscores {
	struct highscore {
		std::string name;
		std::string timestamp;
		double score;		
	};
}	

// Copyright http://stackoverflow.com/questions/997946/how-to-get-current-time-and-date-in-c

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
	time_t     now = time(0);
	struct tm  tstruct;
	char       buf[80];
	//tstruct = *localtime(&now);
	// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
	// for more information about date/time format
	//strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

	return buf;
}

Highscore::Highscore()
{
}

Highscore::~Highscore()
{
}

// https://cboard.cprogramming.com/c-programming/135750-scoreboard-saved-txt-file.html
// https://stackoverflow.com/questions/15388041/how-to-write-stdstring-to-file
void Highscore::addToScoreboard(std::string name, double score)
{	
	std::ofstream file("scoreboard.txt");

	highscores::highscore highscore;

	highscore.name = name;
	highscore.score = score;
	highscore.timestamp = currentDateTime();

	file << ("Name: %s \nPoints: %d\n\n\n", highscore.name, highscore.score);

	file.close();
};
