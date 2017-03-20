#include "Highscore.h"
#include <time.h>
#include "Utils/json.hpp"

// https://github.com/nlohmann/json

using json = nlohmann::json;

namespace highscores {
	// a simple struct to model a Highscore
	struct highscore {
		std::string name;
		std::string timestamp;
		double score;		
	};

	void to_json(json& j, const highscore& s) {
		j = json{ { "name", s.name },{ "timestamp", s.timestamp },{ "score", s.score } };
	}

	void from_json(const json& j, highscore& s) {
		s.name = j["name"].get<std::string>();		
		s.timestamp = j["timestamp"].get<std::string>();
		s.score = j["score"].get<std::double>();
	}
}

// Copyright http://stackoverflow.com/questions/997946/how-to-get-current-time-and-date-in-c

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
	time_t     now = time(0);
	struct tm  tstruct;
	char       buf[80];
	tstruct = *localtime(&now);
	// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
	// for more information about date/time format
	strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

	return buf;
}

Highscore::Highscore()
{
	

	// create an empty structure (null)
	json j;

	// add a number that is stored as double (note the implicit conversion of j to an object)
	j["pi"] = 3.141;

	// add a Boolean that is stored as bool
	j["happy"] = true;

	// add a string that is stored as std::string
	j["name"] = "Niels";

	// add another null object by passing nullptr
	j["nothing"] = nullptr;

	// add an object inside the object
	j["answer"]["everything"] = 42;

	// add an array that is stored as std::vector (using an initializer list)
	j["list"] = { 1, 0, 2 };

	// add another object (using an initializer list of pairs)
	j["object"] = { { "currency", "USD" },{ "value", 42.99 } };

}


Highscore::~Highscore()
{
}
