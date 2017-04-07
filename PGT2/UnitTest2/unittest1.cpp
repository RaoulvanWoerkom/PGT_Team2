#include "stdafx.h"
#include "CppUnitTest.h"
#include "../Game/src/UnitTestSample.h"
#include "../Game/src/World.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace UnitTest2
{		
	TEST_CLASS(UnitTest1)
	{
	public:
		
		TEST_METHOD(TestSumFail)
		{
			auto test = UnitTestSample();			

			Assert::AreEqual(test.sum, test.testsum, 0.0, L"Incorrect sum!", LINE_INFO());
		}

		TEST_METHOD(TestMathPass)
		{
			auto test = UnitTestSample();

			Assert::AreEqual(test.testsum/test.sum, test.sum, 0.0, L"Incorrect sum!", LINE_INFO());
		}

		TEST_METHOD(Ogre)
		{
			auto test = InputManager();
		}
	};


}	