//
// Copyright 2014-2023 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//
// Simply include this file to enable the Code Library testing framework.
//

#ifndef CODELIBRARY_BASE_TESTING_H_
#define CODELIBRARY_BASE_TESTING_H_

#include <chrono>
#include <map>
#include <regex>
#include <string>

#include "codelibrary/base/ansi.h"
#include "codelibrary/base/array.h"
#include "codelibrary/base/equal.h"
#include "codelibrary/base/message.h"

namespace cl {

/**
 * TestFilter filters tests based on regex patterns from command line arguments.
 */
class TestFilter {
public:
    /**
     * Construct TestFilter from command line arguments.
     * Returns an error code if parsing fails.
     */
    static std::pair<TestFilter, int> Create(int argc, char* argv[]);

    /**
     * Check if a test should run based on the filter.
     */
    bool ShouldRunTest(const std::string& test_case_name,
                       const std::string& test_name) const;

    /**
     * Check if help was requested.
     */
    bool IsHelpRequested() const {
        return help_requested_;
    }

    /**
     * Print help message.
     */
    void PrintHelp(const char* program_name) const;

private:
    TestFilter() = default;

    Array<std::regex> patterns_;
    bool help_requested_ = false;
};

/**
 * The abstract test class that all tests should inherit from.
 *
 * We do not need to define a Test object explicitly. The TEST macro
 * automatically do this for you.
 *
 * The only time you derive from Test is when defining a test fixture to be used
 * a TEST_F.  For example:
 *
 *   class IsPrimeTest : public Test {
 *   protected:
 *       virtual void SetUp() { ... }
 *       virtual void Finish() { ... }
 *       ...
 *   };
 *
 *   TEST_F(IsPrimeTest, TestSmallPrimes) { ... }
 *   TEST_F(IsPrimeTest, TestLargePrimes) { ... }
 */
class Test {
public:
    Test() = default;

    Test(const Test&) = delete;

    virtual ~Test() = default;

    Test& operator=(const Test&) = delete;

    /**
     * Set up the test fixture.
     */
    virtual void SetUp() {}

    /**
     * Call when test finished.
     */
    virtual void Finish() {}

    /**
     * Set up, execute, and finish the test.
     */
    void Run() {
        SetUp();
        TestBody();
        Finish();
    }

private:
    /**
     * Run the test after the test fixture has been set up.
     *
     * A sub-class must implement this to define the test logic.
     *
     * DO NOT OVERRIDE THIS FUNCTION DIRECTLY IN A USER PROGRAM.
     * Instead, use the TEST or TEST_F macros.
     */
    virtual void TestBody() {}
};

/**
 * TestProgram is a singleton class, which is consist of a set of test cases.
 * It has only one instance, which is created by TestProgram::GetInstance().
 * This instance is never deleted.
 *
 * TestProgram is not copyable.
 */
class TestProgram {
public:
    /**
     * A TestInfo object stores the following information about a test:
     *   test case name;
     *   test name;
     *   a pointer to the test.
     *
     * The constructor of TestInfo registers itself with the UnitTest singleton
     * such that the RUN_ALL_TESTS() function knows which tests to run.
     */
    struct TestInfo {
        TestInfo(const char* test_case_name,
                 const char* test_name,
                 Test* test_ptr)
            : test_case_name(test_case_name), name(test_name), test(test_ptr) {}

        TestInfo() = delete;

        TestInfo(const TestInfo&) = delete;

        /**
         * Destructs a TestInfo object.
         * This function is not virtual, so don't inherit from TestInfo.
         */
        ~TestInfo() {
            delete test;
        }

        TestInfo& operator=(const TestInfo&) = delete;

        // Test case name.
        std::string test_case_name;

        // Test name.
        std::string name;

        // Pointer to the test.
        Test* test = nullptr;
    };

private:
    /**
     * Result for TestProgram.
     */
    struct Result {
        // The number of all test cases.
        int total_test_case_count = 0;

        // The number of successful tests.
        int successful_test_count = 0;

        // The number of failed tests.
        int failed_test_count = 0;

        // The number of all tests.
        int total_test_count = 0;

        // The elapsed time, in milliseconds.
        int64_t elapsed_time = 0;

        // Store failed tests.
        Array<TestInfo*> failed_tests;
    };

    /**
     * TestProgram is consist of a set of test cases.
     */
    struct TestCase {
        /**
         * Creates a TestCase with the given name.
         */
        explicit TestCase(const char* test_case_name) : name(test_case_name) {}

        ~TestCase() {
            for (TestInfo* test_info : test_info_list) {
                delete test_info;
            }
        }

        // Test case name.
        std::string name;

        // Successful test count. A test case has several TestBlock, this member
        // is used to count the number of successful blocks.
        int successful_test_count = 0;

        // Failed test count.
        int failed_test_count = 0;

        // Elapsed time (in milliseconds). It is the sum of the running time of
        // all TestBlocks of this test case.
        int64_t elapsed_time = 0;

        // Test info list.
        Array<TestInfo*> test_info_list;
    };

private:
    // Prevent users define their own TestProgram.
    TestProgram() {
        support_ansi_ = ansi::SupportAnsiEscapeCode();
    }

public:
    /**
     * Each ASSERT macro, such as ASSERT(), ASSERT_EQ(), has a result.
     */
    struct AssertResult {
        AssertResult(const char* filename,
                     int line,
                     const char* msg,
                     bool success_or_fail)
            : file_name(filename),
              line_number(line),
              message(msg),
              success(success_or_fail) {}

        AssertResult& operator=(const Message& msg) {
            if (!msg.empty())
                message += "\n" + msg.text();
            return *this;
        }

        // The source file where the test took place.
        std::string file_name;

        // The line number in the source file where the test took place.
        int line_number = -1;

        // The failure message.
        std::string message;

        // The ASSERT macro is success or not.
        bool success = false;
    };

    // We disallow the copy constructor.
    TestProgram(const TestProgram&) = delete;

    // We disallow the copy assignment.
    TestProgram& operator=(const TestProgram&) = delete;

    ~TestProgram() {
        for (auto test_case : test_case_list_) {
            delete test_case;
        }
    }

    /**
     * Get the singleton TestProgram object.
     *
     * A TestProgram object is constructed when this method is called at first
     * time. The consecutive calls will return the same object.
     */
    static TestProgram* GetInstance() {
        static TestProgram instance;
        return &instance;
    }

    /**
     * Register a Test.
     */
    TestInfo* RegisterTest(const char* test_case_name,
                           const char* name,
                           Test* test) {
        ++result_.total_test_count;

        TestInfo* test_info = new TestInfo(test_case_name, name, test);

        if (test_case_map_.find(test_case_name) == test_case_map_.end()) {
            TestCase* test_case = new TestCase(test_case_name);
            test_case->test_info_list.emplace_back(test_info);
            test_case_list_.push_back(test_case);
            ++n_test_cases_;
            test_case_map_[test_case_name] = test_case;
        } else {
            test_case_map_[test_case_name]->test_info_list.emplace_back(
                test_info);
        }

        return test_info;
    }

    /**
     * Add a AssertResult.
     *
     * All ASSERT macros (e.g. ASSERT, ASSERT_EQ, etc) implicitly call this to
     * report their results.
     * Users should use the ASSERT macros instead of calling this directly.
     */
    void operator+=(const AssertResult& result) {
        assert_results_.push_back(result);
        PrintOnAssertResult(result);
    }

    /**
     * Run all tests in this TestProgram object and print the result.
     *
     * This method can only be called from the main thread.
     *
     * Return 0 if successful, or 1 otherwise.
     */
    int Run(int argc = 0, char* argv[] = nullptr) {
        // Create test filter from command line arguments
        auto [filter, error_code] = TestFilter::Create(argc, argv);
        if (error_code != 0) {
            return error_code;
        }

        // Handle help request
        if (filter.IsHelpRequested()) {
            filter.PrintHelp(argc > 0 ? argv[0] : nullptr);
            return 0;
        }

        result_.total_test_case_count = n_test_cases_;

        PrintOnTestProgramStart(filter);

        std::chrono::time_point<std::chrono::system_clock> start, end;
        for (TestCase* test_case : test_case_list_) {
            int64_t elapsed_time = 0;

            PrintOnTestCaseStart(test_case, filter);
            for (TestInfo* test_info : test_case->test_info_list) {
                // Check if test should run based on filter
                if (!filter.ShouldRunTest(test_info->test_case_name,
                                          test_info->name)) {
                    continue;  // Skip this test
                }

                PrintOnTestStart(test_info);

                start = std::chrono::system_clock::now();
                test_info->test->Run();
                end = std::chrono::system_clock::now();

                int64_t time =
                    std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                          start)
                        .count();

                elapsed_time += time;

                if (IsPassed()) {
                    ++result_.successful_test_count;
                    PrintOnTestSuccessed(test_info, time);
                } else {
                    result_.failed_tests.push_back(test_info);
                    ++result_.failed_test_count;
                    PrintOnTestFailed(test_info, time);
                }

                assert_results_.clear();
            }

            test_case->elapsed_time = elapsed_time;
            PrintOnTestCaseEnd(test_case, filter);

            result_.elapsed_time += elapsed_time;
        }

        PrintOnTestProgramEnd(filter);

        return (result_.failed_test_count > 0) ? 1 : 0;
    }

    /**
     * Return a failure message for ASSERT.
     */
    static std::string AssertFailureMessage(const char* expression_text) {
        return Format("ASSERT({}) failed!\n", expression_text);
    }

    /**
     * Return a failure message for ASSERT_FALSE.
     */
    static std::string AssertFalseFailureMessage(const char* expression_text) {
        return Format("ASSERT_FALSE({}) failed!\n", expression_text);
    }

private:
    /**
     * Return true if the current test successes.
     */
    bool IsPassed() const {
        for (const AssertResult& result : assert_results_) {
            if (!result.success) {
                return false;
            }
        }
        return true;
    }

    /**
     * Count tests in a test case that match the filter.
     */
    int CountFilteredTests(const TestCase* test_case,
                           const TestFilter& filter) const {
        int count = 0;
        for (const TestInfo* test_info : test_case->test_info_list) {
            if (filter.ShouldRunTest(test_info->test_case_name,
                                     test_info->name)) {
                ++count;
            }
        }
        return count;
    }

    /**
     * Count total tests and test cases that match the filter.
     */
    std::pair<int, int> CountFilteredTestsAndCases(
        const TestFilter& filter) const {
        int total_tests = 0;
        int total_cases = 0;
        for (const TestCase* test_case : test_case_list_) {
            int filtered_tests = CountFilteredTests(test_case, filter);
            if (filtered_tests > 0) {
                total_tests += filtered_tests;
                ++total_cases;
            }
        }
        return {total_tests, total_cases};
    }

    /**
     * Print information before any test activity starts.
     */
    void PrintOnTestProgramStart(const TestFilter& filter) const {
        auto [filtered_test_count, filtered_case_count] =
            CountFilteredTestsAndCases(filter);

        // Don't print anything if no tests will run
        if (filtered_test_count == 0) {
            return;
        }

        Print("[==========] ");
        Print("Running {} from {}.\n",
              PrintTestCount(filtered_test_count),
              PrintTestCaseCount(filtered_case_count));
        fflush(stdout);
    }

    /**
     * Print information before the test case starts.
     */
    void PrintOnTestCaseStart(TestCase* test_case,
                              const TestFilter& filter) const {
        int filtered_test_count = CountFilteredTests(test_case, filter);

        // Don't print anything if no tests will run in this case
        if (filtered_test_count == 0) {
            return;
        }

        Print("[----------] ");
        Print("{} from {}\n",
                   PrintTestCount(filtered_test_count),
                   test_case->name);
        fflush(stdout);
    }

    /**
     * Print information before the test starts.
     */
    void PrintOnTestStart(TestInfo* test_info) const {
        if (support_ansi_) {
            Print("[ \033[0;32mRUN\033[0m      ] ");
        } else {
            Print("[ RUN      ] ");
        }
        PrintTestName(test_info->test_case_name, test_info->name);
        Print("\n");
        fflush(stdout);
    }

    /**
     * Print information after a failed assertion or a SUCCEED() invocation.
     */
    void PrintOnAssertResult(const AssertResult& result) const {
        // If the ASSERT succeeds, we don't need to do anything.
        if (result.success)
            return;

        Print("[          ]\n");
        if (support_ansi_) {
            Print("[ \033[0;31mFAILURE\033[0m  ] At ");
        } else {
            Print("[ FAILURE  ] At ");
        }
        Print("{}({})\n", result.file_name.c_str(), result.line_number);

        // Print failure message.
        if (support_ansi_) {
            Print("\n\033[0;36m{}\033[0m\n\n", result.message.c_str());
        } else {
            Print("\n{}\n\n", result.message.c_str());
        }

        fflush(stdout);
    }

    /**
     * Print information after the test successes.
     */
    void PrintOnTestSuccessed(TestInfo* test_info, int64_t test_time) const {
        if (support_ansi_) {
            Print("[       \033[0;32mOK\033[0m ] ");
        } else {
            Print("[       OK ] ");
        }
        PrintTestName(test_info->test_case_name, test_info->name);
        Print(" ({} ms)\n", test_time);

        fflush(stdout);
    }

    /**
     * Print information after the test failed.
     */
    void PrintOnTestFailed(TestInfo* test_info, int64_t test_time) const {
        PrintFailed();
        PrintTestName(test_info->test_case_name, test_info->name);
        Print(" ({} ms)\n", test_time);

        fflush(stdout);
    }

    /**
     * Print information after the test case ends.
     */
    void PrintOnTestCaseEnd(TestCase* test_case,
                            const TestFilter& filter) const {
        int filtered_test_count = CountFilteredTests(test_case, filter);

        // Don't print anything if no tests ran in this case
        if (filtered_test_count == 0) {
            return;
        }

        Print("[----------] ");
        Print("{} from {} ({} ms total)\n\n",
                   PrintTestCount(filtered_test_count),
                   test_case->name,
                   test_case->elapsed_time);
        fflush(stdout);
    }

    /**
     * Print information after all test activities have ended.
     */
    void PrintOnTestProgramEnd(const TestFilter& filter) const {
        auto [filtered_test_count, filtered_case_count] =
            CountFilteredTestsAndCases(filter);

        // Don't print anything if no tests ran
        if (filtered_test_count == 0) {
            return;
        }

        Print("[==========] ");
        Print("{} from {} ran.",
                   PrintTestCount(filtered_test_count),
                   PrintTestCaseCount(filtered_case_count));
        Print(" ({} ms total)\n", result_.elapsed_time);
        if (support_ansi_) {
            Print("[  \033[0;34mPASSED\033[0m  ] ");
        } else {
            Print("[  PASSED  ] ");
        }
        Print("{}.\n", PrintTestCount(result_.successful_test_count));

        if (result_.failed_test_count > 0) {
            PrintFailed();
            Print("{}, listed below:\n",
                       PrintTestCount(result_.failed_test_count));

            for (TestInfo* test_info : result_.failed_tests) {
                PrintFailed();
                PrintTestName(test_info->test_case_name, test_info->name);
                Print("\n");
            }

            if (support_ansi_) {
                Print("\n\033[0;31m{:2} FAILED {}\033[0m\n",
                           result_.failed_test_count,
                           result_.failed_test_count == 1 ? "TEST" : "TESTS");
            } else {
                Print("\n{:2} FAILED {}\n",
                           result_.failed_test_count,
                           result_.failed_test_count == 1 ? "TEST" : "TESTS");
            }
        }
        fflush(stdout);
    }

    /**
     * Print Test name.
     */
    static void PrintTestName(const std::string& test_case,
                              const std::string& test) {
        Print("{}.{}", test_case, test);
    }

    /**
     * Print a countable noun.
     * Depending on its quantity, either the singular form or the plural form is
     * used. e.g.
     *
     * PrintCountableNoun(1, "formula", "formuli") returns "1 formula".
     * PrintCountableNoun(5, "book", "books") returns "5 books".
     */
    static std::string PrintCountableNoun(int count,
                                          const char* singular_form,
                                          const char* plural_form) {
        return Format("{} {}", count, (count <= 1) ? singular_form
                                                   : plural_form);
    }

    /**
     * Print the count of tests.
     */
    static std::string PrintTestCount(int test_count) {
        return PrintCountableNoun(test_count, "test", "tests");
    }

    /**
     * Print the count of test cases.
     */
    static std::string PrintTestCaseCount(int test_case_count) {
        return PrintCountableNoun(test_case_count, "test case", "test cases");
    }

    /**
     * Print [  FAILED  ].
     */
    void PrintFailed() const {
        if (support_ansi_) {
            Print("[  \033[0;31mFAILED\033[0m  ] ");

        } else {
            Print("[  FAILED  ] ");
        }
    }

    // The number of test cases.
    int n_test_cases_ = 0;

    // Store the registered test cases.
    Array<TestCase*> test_case_list_;

    // Map the test by its test case.
    std::map<std::string, TestCase*> test_case_map_;

    // The array of AssertResult.
    Array<AssertResult> assert_results_;

    // The test program result.
    Result result_;

    // Support ANSI color code or not.
    bool support_ansi_ = false;
};

// TestFilter implementation
inline std::pair<TestFilter, int> TestFilter::Create(int argc, char* argv[]) {
    TestFilter filter;

    // Check for --help flag
    if (argc > 1) {
        for (int i = 1; i < argc; ++i) {
            if (std::string(argv[i]) == "--help") {
                filter.help_requested_ = true;
                return {filter, 0};
            }
        }
    }

    // Build regex patterns from command line arguments
    if (argc > 1) {
        for (int i = 1; i < argc; ++i) {
            try {
                filter.patterns_.emplace_back(argv[i]);
            } catch (const std::regex_error& e) {
                Print(
                    "Invalid regex pattern '{}': {}\n", argv[i], e.what());
                return {filter, 1};
            }
        }
    }

    return {filter, 0};
}

inline bool TestFilter::ShouldRunTest(const std::string& test_case_name,
                                      const std::string& test_name) const {
    // If no patterns specified, run all tests
    if (patterns_.empty()) {
        return true;
    }

    // Check if test matches any pattern
    std::string full_test_name = test_case_name + "." + test_name;
    for (const auto& pattern : patterns_) {
        if (std::regex_search(full_test_name, pattern)) {
            return true;
        }
    }

    return false;
}

inline void TestFilter::PrintHelp(const char* program_name) const {
    Print("Usage: {} [regex_patterns...]\n",
               program_name ? program_name : "test_program");
    Print("\n");
    Print("Options:\n");
    Print("  --help                Show this help message\n");
    Print(
        "  regex_patterns...     Only run tests whose names match one of the "
        "given regex patterns\n");
    Print("\n");
    Print("Examples:\n");
    Print("  {}                    Run all tests\n",
               program_name ? program_name : "test_program");
    Print("  {} \"MyTest.*\"         Run all tests in MyTest test case\n",
               program_name ? program_name : "test_program");
    Print(
        "  {} \".*Add.*\" \".*Sub.*\" Run tests matching Add or Sub patterns\n",
        program_name ? program_name : "test_program");
}

}  // namespace cl

#define TESTING_MESSAGE_AT(file, line, message, success_or_fail)        \
    (*cl::TestProgram::GetInstance()) += cl::TestProgram::AssertResult( \
        file, line, message, success_or_fail) = cl::Message()

#define TESTING_MESSAGE(message, success_or_fail) \
    TESTING_MESSAGE_AT(__FILE__, __LINE__, message, success_or_fail)

#define TESTING_FAILURE(message) return TESTING_MESSAGE(message, false)

/**
 * Assert expression is true.
 */
#define ASSERT(expression) \
    if (!(expression))     \
    TESTING_FAILURE("ASSERT(" #expression ") failed!")

/**
 * Assert expression is false.
 */
#define ASSERT_FALSE(expression) \
    if (expression)              \
    TESTING_FAILURE("ASSERT_FALSE(" #expression ") failed!")

// Note: the ASSERT_* macros below intentionally expand to an `if (...) return
// TESTING_MESSAGE(...)` expression rather than a `do { ... } while (0)` block.
// This preserves the ability to chain extra diagnostics via operator<<, e.g.
//     ASSERT_EQ(a, b) << "context: " << i;
// Side note: arguments are evaluated more than once on failure. Avoid passing
// expressions with side effects (e.g. ASSERT_EQ(x++, 5)).

/**
 * Assert the expected equal to actual.
 */
#define ASSERT_EQ(actual, expected)                                         \
    if (!cl::Equal(actual, expected))                                       \
    TESTING_FAILURE(("ASSERT_" + cl::CheckEQFailureMessage(                 \
                    #actual, #expected, actual, expected)).c_str())

/**
 * Assert if two parameter is nearly the same.
 */
#define ASSERT_EQ_NEAR(a, b, absolute_range)                             \
    if (!cl::Equal(a, b, absolute_range))                                \
    TESTING_FAILURE(                                                     \
        ("ASSERT_" + cl::CheckEQNearFailureMessage(                      \
         #a, #b, #absolute_range, a, b, absolute_range)).c_str())

/**
 * Range EQ assert.
 */
#define ASSERT_EQ_RANGE(                                                \
    actual_first, actual_last, expected_first, expected_last)           \
    if (!(cl::Equal(                                                    \
            actual_first, actual_last, expected_first, expected_last))) \
    TESTING_FAILURE(                                                    \
        ("ASSERT_" + cl::CheckEQRangeFailureMessage(#actual_first,      \
                                                    #actual_last,       \
                                                    #expected_first,    \
                                                    #expected_last,     \
                                                    actual_first,       \
                                                    actual_last,        \
                                                    expected_first,     \
                                                    expected_last)).c_str())

/**
 * Expands to the name of the class that implements the given test.
 */
#define TEST_CLASS_NAME(test_case_name, test_name) \
    test_case_name##_##test_name##_Test

/**
 * Helper macro for defining tests.
 */
#define TESTING(test_case_name, test_name, parent_test_class)             \
    class TEST_CLASS_NAME(test_case_name, test_name)                      \
        : public parent_test_class {                                      \
    public:                                                               \
        TEST_CLASS_NAME(test_case_name, test_name)() = default;           \
        TEST_CLASS_NAME(test_case_name, test_name)                        \
        (const TEST_CLASS_NAME(test_case_name, test_name) &) = delete;    \
        TEST_CLASS_NAME(test_case_name, test_name) & operator=(           \
            const TEST_CLASS_NAME(test_case_name, test_name) &) = delete; \
                                                                          \
    private:                                                              \
        void TestBody();                                                  \
        static cl::TestProgram::TestInfo* TEST_INFO;                      \
    };                                                                    \
                                                                          \
    cl::TestProgram::TestInfo* TEST_CLASS_NAME(test_case_name,            \
                                               test_name)::TEST_INFO =    \
        cl::TestProgram::GetInstance()->RegisterTest(                     \
            #test_case_name,                                              \
            #test_name,                                                   \
            new TEST_CLASS_NAME(test_case_name, test_name));              \
    void TEST_CLASS_NAME(test_case_name, test_name)::TestBody()

/**
 * Define a test.
 *
 * Each test will be grouped into test cases according to its 'test_case_name'.
 * And each test has its own name, 'test_name', within the test case.
 *
 * Users should put the test code between braces after using this macro.
 *
 * For example:
 *
 * TEST(IsPrimeTest, IsPrime) {
 *     ASSERT(Isprime(23));
 * }
 */
#define TEST(test_case_name, test_name) \
    TESTING(test_case_name, test_name, cl::Test)

/**
 * Define a test that uses a test fixture.
 *
 * A test fixture class must be declared earlier.
 * Users should put the test code between braces after using this macro.
 *
 * For example:
 *
 * class IsPrimeTest : public cl::Test {
 * protected:
 *     virtual void SetUp() { random_.set_seed(3); }
 *
 *     Random random_;
 * };
 *
 * TEST_F(IsPrimeTest, TestRandomPriems) {
 *     ASSERT(IsPrime(random_());
 * }
 */
#define TEST_F(test_case_name, test_name) \
    TESTING(test_case_name, test_name, test_case_name)

/**
 * Use this macro in main() to run all tests.
 * It returns 0 if all tests are successful, or 1 otherwise.
 */
#define RUN_ALL_TESTS() cl::TestProgram::GetInstance()->Run()

/**
 * Use this macro in main() to run all tests with command line arguments.
 * It returns 0 if all tests are successful, or 1 otherwise.
 */
#define RUN_ALL_TESTS_WITH_ARGS(argc, argv) \
    cl::TestProgram::GetInstance()->Run(argc, argv)

#endif  // CODELIBRARY_BASE_TESTING_H_
