#ifndef __TestApplication_h_
#define __TestApplication_h_

#include "BaseApplication.h"

class TestApplication : public BaseApplication
{
public:
	TestApplication();
	virtual ~TestApplication();

protected:
	virtual void createScene();
	virtual void createCamera();
	virtual void createViewports();
};

#endif // #ifndef __TestApplication_h_