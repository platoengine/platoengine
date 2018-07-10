// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{

namespace AbstractInterface
{

class SparseMatrix
{
public:
    SparseMatrix();
    virtual ~SparseMatrix();

    virtual size_t getNumRows() = 0;
    virtual size_t getNumColumns() = 0;

    virtual void matVec(const std::vector<double>& input, std::vector<double>& output, bool transpose) = 0;
    virtual void matVecToReduced(const std::vector<double>& input, std::vector<double>& output, bool transpose) = 0;

    virtual void rowNormalize(const std::vector<double>& rowNormalizationFactors) = 0;
    virtual void columnNormalize(const std::vector<double>& columnNormalizationFactors) = 0;

    virtual void getNonZeroSortedRows(std::vector<size_t>& nonZeroRows) = 0;
    virtual size_t getNumNonZeroSortedRows() = 0;
    virtual void getNonZeroSortedColumns(std::vector<size_t>& nonZeroColumns) = 0;
    virtual size_t getNumNonZeroSortedColumns() = 0;

    virtual void getRow(size_t row, std::vector<double>& data, std::vector<size_t>& columns) = 0;
    virtual void setRow(size_t row, const std::vector<double>& data) = 0;

protected:
};

}

}
