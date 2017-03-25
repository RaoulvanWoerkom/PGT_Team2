#include "stdafx.h"
#include "CppUnitTest.h"
#include "src/UnitTestSample.h"

namespace Testing
{		
	TEST_CLASS(UnitTest1)
	{
	public:
		
		TEST_METHOD(SumFALSE)
		{
			UnitTestSample test = UnitTestSample();
			Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(test.sum, test.testsum, 0.0, L"Incorrect sum!", LINE_INFO());
		}

		TEST_METHOD(DivideTRUE)
		{
			UnitTestSample test = UnitTestSample();
			Microsoft::VisualStudio::CppUnitTestFramework::Assert::AreEqual(test.sum, test.testsum/test.sum, 0.0, L"Incorrect sum!", LINE_INFO());
		}
	};
}

UnitTestSample::UnitTestSample()
{
	sum = 3;
	testsum = 9;
}


UnitTestSample::~UnitTestSample()
{
	//sum = 3;
	//testsum = 9;
}