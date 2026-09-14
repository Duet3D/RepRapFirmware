#ifndef RRF_TEST_RUNNER_H
#define RRF_TEST_RUNNER_H

#include <cmath>
#include <cstdio>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace test
{
	using TestFunction = void (*)();

	struct TestCase
	{
		const char* name;
		TestFunction function;
	};

	inline std::vector<TestCase>& Registry()
	{
		static std::vector<TestCase> registry;
		return registry;
	}

	class Registrar
	{
	public:
		Registrar(const char* name, TestFunction function)
		{
			Registry().push_back({name, function});
		}
	};

	class Failure : public std::runtime_error
	{
	public:
		explicit Failure(const std::string& message)
			: std::runtime_error(message)
		{
		}
	};

	[[noreturn]] inline void Fail(const char* file, int line, const std::string& message)
	{
		std::ostringstream stream;
		stream << file << ':' << line << ": " << message;
		throw Failure(stream.str());
	}

	template <typename T, typename U>
	void ExpectEqual(const T& actual, const U& expected, const char* file, int line, const char* actualExpr, const char* expectedExpr)
	{
		if (!(actual == expected))
		{
			std::ostringstream stream;
			stream << "expected " << actualExpr << " == " << expectedExpr << ", got " << actual << " and " << expected;
			Fail(file, line, stream.str());
		}
	}

	template <typename T, typename U, typename V>
	void ExpectNear(const T& actual, const U& expected, const V& tolerance, const char* file, int line, const char* actualExpr, const char* expectedExpr, const char* toleranceExpr)
	{
		const auto delta = std::fabs(static_cast<double>(actual) - static_cast<double>(expected));
		if (delta > static_cast<double>(tolerance))
		{
			std::ostringstream stream;
			stream << "expected " << actualExpr << " ~= " << expectedExpr << " within " << toleranceExpr
				   << ", got delta " << delta;
			Fail(file, line, stream.str());
		}
	}

	inline void ExpectTrue(bool condition, const char* file, int line, const char* expression)
	{
		if (!condition)
		{
			std::ostringstream stream;
			stream << "expected true: " << expression;
			Fail(file, line, stream.str());
		}
	}

	inline int RunAllTests()
	{
		int failures = 0;
		for (const TestCase& testCase : Registry())
		{
			try
			{
				testCase.function();
			}
			catch (const Failure& failure)
			{
				++failures;
				std::fprintf(stderr, "[FAIL] %s\n  %s\n", testCase.name, failure.what());
				continue;
			}
			catch (const std::exception& exception)
			{
				++failures;
				std::fprintf(stderr, "[FAIL] %s\n  unexpected exception: %s\n", testCase.name, exception.what());
				continue;
			}

			std::fprintf(stdout, "[PASS] %s\n", testCase.name);
		}

		std::fprintf(stdout, "Ran %zu test(s), %d failure(s)\n", Registry().size(), failures);
		return failures;
	}
}

#define TEST_CASE(name) \
	static void name(); \
	static test::Registrar name##_registrar(#name, &name); \
	static void name()

#define EXPECT_TRUE(expr) test::ExpectTrue((expr), __FILE__, __LINE__, #expr)
#define EXPECT_EQ(actual, expected) test::ExpectEqual((actual), (expected), __FILE__, __LINE__, #actual, #expected)
#define EXPECT_NEAR(actual, expected, tolerance) test::ExpectNear((actual), (expected), (tolerance), __FILE__, __LINE__, #actual, #expected, #tolerance)

#endif