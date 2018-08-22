// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace example
{
namespace dense_matrix
{

void matvec(size_t num_rows_,
            size_t num_columns,
            double** matrix_,
            const double* vec_in_,
            double* vec_out_,
            bool transpose = false);

double** create(size_t nrows_, size_t ncols_);
void fill(const double alpha_, double** matrix_, size_t nrows_, size_t ncols_);
void fill_by_row_major(const double* vec_, double** matrix_, size_t nrows_, size_t ncols_);
void identity(double** matrix_, size_t nrows_, size_t ncols_);
void copy(double** input_, double** output_, size_t nrows_, size_t ncols_);
void aXpY(const double alpha_, double** matrix_X, double** matrix_Y, size_t nrows_, size_t ncols_);
void matrix_matrix_product(const double alpha_,
                           double** matrix_X,
                           size_t X_nrows_,
                           size_t X_ncols_,
                           bool X_transpose,
                           double** matrix_Y,
                           size_t Y_nrows_,
                           size_t Y_ncols_,
                           bool Y_transpose,
                           double** result);
void scale(const double alpha_, double** matrix, size_t nrows_, size_t ncols_);
void print(double** matrix_, size_t nrows_, size_t ncols_);
void destroy(double** matrix_, size_t nrows_, size_t ncols_);

}
}
}
