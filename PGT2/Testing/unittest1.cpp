#include "stdafx.h"
#include "CppUnitTest.h"
#include "UnitTestSample.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace Testing
{		
	TEST_CLASS(UnitTest1)
	{
	public:
		
		TEST_METHOD(SumFALSE)
		{
			// TODO: Your test code here

			//BaseApplication base_application;

			//TestApplication test = TestApplication();

			//Assert::AreEqual(60.0, test.remainingTime, 1.0, L"Incorrect starting time", LINE_INFO());

			UnitTestSample test = UnitTestSample();

			//test.count();

			Assert::AreEqual(test.sum, test.testsum, 0.0, L"Incorrect sum!", LINE_INFO());

			//Assert::IsTrue(true);
		}

		TEST_METHOD(DivideTRUE)
		{
			// TODO: Your test code here

			//BaseApplication base_application;

			//TestApplication test = TestApplication();

			//Assert::AreEqual(60.0, test.remainingTime, 1.0, L"Incorrect starting time", LINE_INFO());

			UnitTestSample test = UnitTestSample();

			//test.count();

			Assert::AreEqual(test.sum, test.testsum/test.sum, 0.0, L"Incorrect sum!", LINE_INFO());

			//Assert::IsTrue(true);
		}


	};
}