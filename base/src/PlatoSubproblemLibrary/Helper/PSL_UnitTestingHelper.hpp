// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

//#define PLATO_SUBPROBLEM_LIBRARY_USES_IN_LIBRARY_UNIT_TESTER
#ifdef PLATO_SUBPROBLEM_LIBRARY_USES_IN_LIBRARY_UNIT_TESTER

#include "PSL_UnitTestingMain.hpp"

#define PSL_TEST(_1, _2) UNIT_TEST(_1, _2)

#endif

#define PLATO_SUBPROBLEM_LIBRARY_USES_GTEST_UNIT_TESTER
#ifdef PLATO_SUBPROBLEM_LIBRARY_USES_GTEST_UNIT_TESTER

#include <gtest/gtest.h>

#define PSL_TEST(_1, _2) TEST(PlatoSubproblemLibrary##_1, _2)

#endif

#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Interface_MpiWrapper.hpp"

#include <vector>
#include <cassert>
#include <mpi.h>
#include <set>

namespace PlatoSubproblemLibrary
{

#define PSL_SerialOnlyTest \
        example::Interface_BasicGlobalUtilities SOT_u; \
        MPI_Comm SOT_c = MPI_COMM_WORLD; \
        example::Interface_MpiWrapper SOT_mi(&SOT_u, &SOT_c); \
        if(SOT_mi.get_size() > 1u) return;

template<typename t>
void expect_equal_sets(const std::set<t>& a, const std::set<t>& b)
{
    EXPECT_EQ(true, (a==b));
}

template<typename t1, typename t2>
void expect_equal_vectors(const std::vector<t1>& a, const std::vector<t2>& b);

template<typename t1, typename t2>
void expect_equal_vectors(const std::vector<std::vector<t1>>& a, const std::vector<std::vector<t2>>& b)
{
    const size_t length = a.size();
    ASSERT_EQ(length, b.size());
    for(size_t i = 0u; i < length; i++)
    {
        expect_equal_vectors(a[i], b[i]);
    }
}

template<typename t1, typename t2>
void expect_equal_vectors(const std::vector<t1>& a, const std::vector<t2>& b)
{
    const size_t length = a.size();
    ASSERT_EQ(length, b.size());
    for(size_t i = 0u; i < length; i++)
    {
        EXPECT_EQ(a[i], b[i]);
    }
}

template<typename t1, typename t2>
void expect_equal_float_vectors(const std::vector<t1>& a, const std::vector<t2>& b)
{
    const size_t length = a.size();
    ASSERT_EQ(length, b.size());
    for(size_t i = 0u; i < length; i++)
    {
        EXPECT_FLOAT_EQ(a[i], b[i]);
    }
}

template<typename t>
void expect_near_vectors(const std::vector<t>& a, const std::vector<t>& b, const t& tol)
{
    const size_t length = a.size();
    ASSERT_EQ(length, b.size());
    for(size_t i = 0u; i < length; i++)
    {
        EXPECT_NEAR(a[i], b[i], tol);
    }
}

template<typename t1, typename t2>
void expect_equal_matrix(t1* a, t2* b)
{
    const size_t num_rows = a->get_num_rows();
    ASSERT_EQ(num_rows, b->get_num_rows());
    const size_t num_columns = a->get_num_columns();
    ASSERT_EQ(num_columns, b->get_num_columns());

    for(size_t r = 0u; r < num_rows; r++)
    {
        for(size_t c = 0u; c < num_columns; c++)
        {
            const double a_val = a->get_value(r, c);
            const double b_val = b->get_value(r, c);

            EXPECT_FLOAT_EQ(a_val, b_val);
        }
    }
}

template<typename t1, typename t2>
void expect_near_matrix(t1* a, t2* b, const double& tol)
{
    const size_t num_rows = a->get_num_rows();
    ASSERT_EQ(num_rows, b->get_num_rows());
    const size_t num_columns = a->get_num_columns();
    ASSERT_EQ(num_columns, b->get_num_columns());

    for(size_t r = 0u; r < num_rows; r++)
    {
        for(size_t c = 0u; c < num_columns; c++)
        {
            const double a_val = a->get_value(r, c);
            const double b_val = b->get_value(r, c);

            EXPECT_NEAR(a_val, b_val, tol);
        }
    }
}

typedef bool (*bool_function)();

template<typename t>
void stocastic_test(t func, const int& min_num_repeat, const int& max_num_repeat, const double& expected_accuracy)
{
    assert(min_num_repeat <= max_num_repeat);

    // get accuracies
    double actual_accuracy = 0.;
    for(int repeat = 0; repeat < max_num_repeat; repeat++)
    {
        actual_accuracy = (actual_accuracy * double(repeat) + double(func())) / double(repeat + 1);

        // if done minimum and better than expectation
        if((min_num_repeat <= repeat + 1) && (actual_accuracy >= expected_accuracy))
        {
            break;
        }
    }

    // compare to expectation
    EXPECT_GE(actual_accuracy, expected_accuracy);
}

typedef double (*double_function)();

template<typename t>
void stocastic_test(t func, const int& num_repeat, const double& center, const double& tol)
{
    // get accuracy
    double average = 0.;
    for(int repeat = 0; repeat < num_repeat; repeat++)
    {
        average += func();
    }
    average /= double(num_repeat);

    // compare to expectation
    EXPECT_NEAR(average, center, tol);
}

}
