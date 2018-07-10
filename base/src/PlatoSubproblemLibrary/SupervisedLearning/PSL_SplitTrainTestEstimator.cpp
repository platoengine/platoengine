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

#include "PSL_SplitTrainTestEstimator.hpp"

#include "PSL_ClassificationAccuracyEstimator.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_Classifier.hpp"
#include "PSL_ClassificationAssessor.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_ClassificationArchive.hpp"
#include "PSL_InputOutputManager.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_DataOrganizerFactory.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <string>
#include <cassert>

namespace PlatoSubproblemLibrary
{

SplitTrainTestEstimator::SplitTrainTestEstimator(AbstractAuthority* authority) :
        ClassificationAccuracyEstimator(authority),
        m_archive_filename(),
        m_delete_archive(),
        m_testing_holdout(-1.)
{
}
SplitTrainTestEstimator::~SplitTrainTestEstimator()
{
}

void SplitTrainTestEstimator::initialize(const std::string& archive_filename,
                                         const bool& delete_archive,
                                         const double& testing_holdout)
{
    m_archive_filename = archive_filename;
    m_delete_archive = delete_archive;
    m_testing_holdout = testing_holdout;
}

void SplitTrainTestEstimator::estimate_accuracy(ParameterData* parameter_data,
                                                ClassificationAssessor* assessor)
{
    // if not initialized, abort
    if(m_archive_filename.size() == 0u)
    {
        m_authority->utilities->fatal_error("SplitTrainTestEstimator: estimate_accuracy invoked before initialize. Aborting.\n\n");
        return;
    }

    // fill filename
    parameter_data->set_archive_filename(m_archive_filename);

    // split archive
    ClassificationArchive splitting_archive(m_authority);
    splitting_archive.initialize(parameter_data);
    std::string testing_filename;
    std::string training_filename;
    splitting_archive.split_dataset(m_testing_holdout, testing_filename, training_filename);

    // training archive
    parameter_data->set_archive_filename(training_filename);
    ClassificationArchive training_archive(m_authority);
    training_archive.initialize(parameter_data);

    // build classifier
    Classifier* classifier = build_classifier(parameter_data->get_classifier(), m_authority);

    // train network
    classifier->initialize(parameter_data, &training_archive);
    classifier->train(parameter_data, &training_archive);

    // testing archive
    parameter_data->set_archive_filename(testing_filename);
    ClassificationArchive testing_archive(m_authority);
    testing_archive.initialize(parameter_data);

    // get sizes
    std::vector<int> input_enum_sizes;
    int output_enum_size = -1;
    testing_archive.get_enum_size(input_enum_sizes, output_enum_size);

    // assess
    assessor->initialize(parameter_data, output_enum_size);
    assessor->assess(classifier, &testing_archive);

    // clean-up
    training_archive.get_io_manager()->delete_file();
    testing_archive.get_io_manager()->delete_file();
    if(m_delete_archive)
    {
        splitting_archive.get_io_manager()->delete_file();
    }
    safe_free(classifier);

    // conditionally announce accuracy
    if(parameter_data->get_verbose())
    {
        m_authority->utilities->print(std::string("testing accuracy: ") + std::to_string(assessor->get_classification_accuracy())
                                      + std::string("\n"));
    }
}

}
