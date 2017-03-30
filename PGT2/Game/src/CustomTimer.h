#ifndef __CustomTimer_h_
#define __CustomTimer_h_

#include "Ogre.h"

class CustomTimer
{
public:
	CustomTimer(void);
	void init();
	void reset();
	void stopAtZero();
	void update();
	void increase(double amount);
	bool isTimeUp();
	double getRemainingTime();

	Ogre::Timer* timer;

private:
	double elapsedTime;
	double remainingTime;
	double totalGameTime;
};

#endif // #ifndef __CustomTimer_h_