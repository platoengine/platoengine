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

#include "PSL_SupervisedGeneratorTester.hpp"

#include "PSL_SupervisedGenerator.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_ClassificationArchive.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_InputOutputManager.hpp"
#include "PSL_DataSequence.hpp"
#include "PSL_Classifier.hpp"
#include "PSL_DataOrganizerFactory.hpp"
#include "PSL_FeedForwardNeuralNetwork.hpp"
#include "PSL_ClassificationAssessor.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <string>
#include <cassert>

namespace PlatoSubproblemLibrary
{

SupervisedGeneratorTester::SupervisedGeneratorTester(AbstractAuthority* authority) :
        m_authority(authority)
{
}
SupervisedGeneratorTester::~SupervisedGeneratorTester()
{
}

double SupervisedGeneratorTester::get_accuracy(SupervisedGenerator* generator,
                                               const int& num_training,
                                               ParameterData* parameters,
                                               const int& num_testing)
{
    // files
    const std::string archive_filename = "_supervised_generator_tester_archive.psl";
    parameters->set_archive_filename(archive_filename);
    const std::string classifier_filename = "_supervised_generator_tester_classifier.psl";

    // build training archive
    build_archive(generator, num_training, parameters);

    // do training and save classifier
    train_on_archive(parameters, classifier_filename);

    // test classifier with new testing samples
    const double testing_accuracy = test_classifier(generator, parameters, num_testing, classifier_filename);
    return testing_accuracy;
}

void SupervisedGeneratorTester::build_archive(SupervisedGenerator* generator, const int& num_training, ParameterData* parameters)
{
    // initialize archive
    ClassificationArchive archive(m_authority);
    archive.initialize(parameters);

    // get sizes from generator
    int output_enum_size = -1;
    int num_input_double_scalars = -1;
    std::vector<int> input_enum_sizes;
    generator->get_sizes(output_enum_size, num_input_double_scalars, input_enum_sizes);

    // set sizes in archive
    archive.set_enum_size(input_enum_sizes, output_enum_size);

    // build training
    for(int train = 0; train < num_training; train++)
    {
        // get a sample
        std::vector<double> input_scalars;
        std::vector<int> input_enums;
        int actual_output_enum = -1;
        generator->generate(input_scalars, input_enums, actual_output_enum);

        // add to archive
        archive.add_row(input_scalars, input_enums, actual_output_enum);
    }
    archive.finalize_archive();
}

void SupervisedGeneratorTester::train_on_archive(ParameterData* parameters, const std::string& classifier_filename)
{
    ClassificationArchive archive(m_authority);
    archive.initialize(parameters);

    // build classifier
    Classifier* classifier = build_classifier(parameters->get_classifier(), m_authority);

    // train classifier
    classifier->initialize(parameters, &archive);
    classifier->train(parameters, &archive);
    const bool do_prefer_binary = true;
    classifier->get_data_sequence()->save_to_file(classifier_filename, do_prefer_binary);

    // cleanup
    archive.get_io_manager()->delete_file();
    safe_free(classifier);
}

double SupervisedGeneratorTester::test_classifier(SupervisedGenerator* generator,
                                                  ParameterData* parameters,
                                                  const int& num_testing,
                                                  const std::string& classifier_filename)
{
    // load trained classifier
    Classifier* loadedClassifier = load_classifier(classifier_filename, m_authority);

    // delete classifier file
    InputOutputManager io_manager(m_authority);
    io_manager.set_filename(classifier_filename);
    io_manager.delete_file();

    // if not a classifier, abort
    if(!loadedClassifier)
    {
        m_authority->utilities->fatal_error("failed to cast loaded classifier. Aborting.\n\n");
    }

    // get sizes from generator
    int output_enum_size = -1;
    int num_input_double_scalars = -1;
    std::vector<int> input_enum_sizes;
    generator->get_sizes(output_enum_size, num_input_double_scalars, input_enum_sizes);

    // build assessor
    ClassificationAssessor assessor(m_authority);
    assessor.initialize(parameters, output_enum_size);

    // do assess on testing
    for(int test = 0; test < num_testing; test++)
    {
        // get a sample
        std::vector<double> input_scalars;
        std::vector<int> input_enums;
        int actual_output_enum = -1;
        generator->generate(input_scalars, input_enums, actual_output_enum);

        // predict
        const int predicted_output_enum = loadedClassifier->classify(input_scalars, input_enums);

        // assess
        assessor.add_to_assessment(actual_output_enum, predicted_output_enum);
    }

    delete loadedClassifier;

    // assess accuracy
    const double testing_accuracy = assessor.get_classification_accuracy();
    return testing_accuracy;
}

}

