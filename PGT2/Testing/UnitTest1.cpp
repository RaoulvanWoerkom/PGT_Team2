#include "stdafx.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

namespace Testing
{
	[TestClass]
	public ref class UnitTest1
	{
	public: 
		[TestMethod]
		void TestMethodFail()
		{
			// TODO: Your test code here

			//BaseApplication base_application;

			//TestApplication test = TestApplication();

			//Assert::AreEqual(60.0, test.remainingTime, 1.0, L"Incorrect starting time", LINE_INFO());

			//UnitTestSample test = UnitTestSample();

			//test.count();

			//Assert::AreEqual(test.sum, test.testsum, 0.0, L"Incorrect sum!", LINE_INFO());

			//Assert::IsTrue(true);

			Assert::Fail();
		}

		[TestMethod]
		void TestMethodPass()
		{
			// TODO: Your test code here

			//BaseApplication base_application;

			//TestApplication test = TestApplication();

			//Assert::AreEqual(60.0, test.remainingTime, 1.0, L"Incorrect starting time", LINE_INFO());

			//UnitTestSample test = UnitTestSample();

			//test.count();

			//Assert::AreEqual(test.sum, test.testsum / test.sum, 0.0, L"Incorrect sum!", LINE_INFO());

			//Assert::IsTrue(true);


			Assert::IsTrue(true);
		}
	};
}
