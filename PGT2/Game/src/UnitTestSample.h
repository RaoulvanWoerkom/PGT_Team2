#pragma once

#include <cppUnit/TestCase.h>
#include <cppUnit/extensions/HelperMacros.h>

#include "Highscore.h"   // the class we want to test

class UnitTestSample : public CppUnit::TestFixture
{
		      CPPUNIT_TEST_SUITE(UnitTestSample);
		     CPPUNIT_TEST(test1);
		      // ... name each method that should be executed
			CPPUNIT_TEST_SUITE_END();
public:
	UnitTestSample();
	~UnitTestSample();

	int sum;
	int testsum;

	void test1()
		      {
		         Highscore nc;
		          bool success = nc.addToScoreboard("Player", 10.0);
		         CPPUNIT_ASSERT(success);
		      }
};

