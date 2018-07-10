/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
*/

#pragma once

#include "PSL_Classifier.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class DenseMatrix;
}
class AbstractAuthority;
class ParameterData;
class ClassificationArchive;
class VectorKernel;

class SupportVectorMachine : public Classifier
{
public:
    SupportVectorMachine(AbstractAuthority* authority);
    virtual ~SupportVectorMachine();

    virtual void unpack_data();

    virtual void initialize(ParameterData* parameters, ClassificationArchive* archive);
    virtual void train(ParameterData* parameters, ClassificationArchive* archive);
    virtual int classify(const std::vector<double>& input_scalars, const std::vector<int>& input_enums);

protected:

    void clear_built_data();
    void allocate_classifier_data(ParameterData* parameters, ClassificationArchive* archive);
    void initialize_vectors(ParameterData* parameters, const int& input_dimension);
    void train_1v1(ParameterData* parameters,
                   AbstractInterface::DenseMatrix* dataset,
                   const std::vector<int>& negative_class_rows,
                   const std::vector<int>& positive_class_rows,
                   std::vector<std::vector<double>*>& support_vector_snapshots);
    void train_an_epoch(const double& structural_risk_parameter,
                        AbstractInterface::DenseMatrix* dataset,
                        const std::vector<int>& negative_class_rows,
                        const std::vector<int>& positive_class_rows,
                        std::vector<double>* support_vector,
                        double& experience);
    void check_parameters(ParameterData* parameters);

    // DataSequence data
    int* m_output_enum_size;
    int* m_kernel_function_enum;
    int* m_kernel_function_parameter;
    int* m_num_snapshots;
    std::vector<int>* m_input_enum_sizes;
    std::vector<std::vector<double>*> m_support_vectors;

    // built from DataSequence
    VectorKernel* m_kernel_function;

};

}
