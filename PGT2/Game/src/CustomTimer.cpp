#include "CustomTimer.h"

const int START_GAME_TIME = 60; /// game time the player has at the start of the game, in seconds.

								/// Constructor
CustomTimer::CustomTimer(void)
{

}

/// Initializes the CustomTimer and sets its attributes to the starting values.
void CustomTimer::init()
{
	timer = new Ogre::Timer();
	reset();
}

/// Resets the attributes of this CustomTimer to the starting values.
void CustomTimer::reset()
{
	totalGameTime = START_GAME_TIME;
	elapsedTime = 0.0;
	timer->reset();
}

/// \brief Sets the remaining time at 0.0.
///
/// The game is over when remainingTime is equal or less than zero.
/// Displaying remainingTime this way results in a strange number (-0.0, -0.1 etc.) which might confuse the player,
/// so make sure the more appropriate '0.0' is always shown.
void CustomTimer::stopAtZero()
{
	remainingTime = 0.0;
}

/// \brief Updates the timer.
///
/// Updates timer with the amount of time since the last reset() call.
/// Ogre's timer can only return milliseconds, but time needs to be displayed as seconds
void CustomTimer::update()
{
	elapsedTime = timer->getMilliseconds() / 1000.0;
	remainingTime = totalGameTime - elapsedTime;
}

/// \brief Adds an amount of time to the (playable) game time.
///
/// Since remainingTime is calculated based on the total game time and elapsedtime,
/// the most logical way to add extra gametime is to increase totalGameTime.
void CustomTimer::increase(double amount)
{
	totalGameTime += amount;
}

/// Checks if the player has run out of game time.
bool CustomTimer::isTimeUp()
{
	return (remainingTime <= 0);
}

/// Returns the remaining game time.
double CustomTimer::getRemainingTime()
{
	return remainingTime;
}