#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Interface_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_ClassificationAssessor.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_Random.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{
namespace TestingClassificationAssessor
{

PSL_TEST(ClassificationAssessor, simple)
{
    set_rand_seed();
    AbstractAuthority authority;
    AbstractInterface::DenseMatrix* expected_confusion_matrix = NULL;
    AbstractInterface::DenseMatrix* actual_confusion_matrix = NULL;
    ParameterData parameter_data;

    ClassificationAssessor assessor(&authority);
    const int num_output_enums = 2;
    assessor.initialize(&parameter_data, num_output_enums);

    // ensure assessor responds fine with no data given
    EXPECT_FLOAT_EQ(0., assessor.get_classification_accuracy());
    expected_confusion_matrix = authority.dense_builder->build_by_fill(num_output_enums, num_output_enums, 0.);
    actual_confusion_matrix = assessor.get_confusion_matrix();
    expect_equal_matrix(expected_confusion_matrix, actual_confusion_matrix);
    delete expected_confusion_matrix;

    // give assessor data
    assessor.add_to_assessment(0, 0);
    assessor.add_to_assessment(1, 1);

    // ensure assessor assessed accurately
    EXPECT_FLOAT_EQ(1., assessor.get_classification_accuracy());
    std::vector<double> first_confusion_state_vector = {1., 0., 0., 1.};
    expected_confusion_matrix = authority.dense_builder->build_by_row_major(num_output_enums, num_output_enums, first_confusion_state_vector);
    actual_confusion_matrix = assessor.get_confusion_matrix();
    expect_equal_matrix(expected_confusion_matrix, actual_confusion_matrix);
    delete expected_confusion_matrix;

    // give assessor data
    assessor.add_to_assessment(0, 1);
    assessor.add_to_assessment(1, 1);

    // ensure assessor assessed accurately
    EXPECT_FLOAT_EQ(0.75, assessor.get_classification_accuracy());
    std::vector<double> second_confusion_state_vector = {1., 1., 0., 2.};
    expected_confusion_matrix = authority.dense_builder->build_by_row_major(num_output_enums, num_output_enums, second_confusion_state_vector);
    actual_confusion_matrix = assessor.get_confusion_matrix();
    expect_equal_matrix(expected_confusion_matrix, actual_confusion_matrix);
    delete expected_confusion_matrix;
}

PSL_TEST(ClassificationAssessor, detailed)
{
    set_rand_seed();
    AbstractAuthority authority;
    ParameterData parameter_data;

    ClassificationAssessor assessor(&authority);
    const int num_output_enums = 5;
    assessor.initialize(&parameter_data, num_output_enums);

    // give assessor data
    int entry_number = 0;
    for(int row = 0; row < num_output_enums; row++)
    {
        for(int column = 0; column < num_output_enums; column++)
        {
            for(int entry_repeat = 0; entry_repeat < entry_number; entry_repeat++)
            {
                assessor.add_to_assessment(row, column);
            }
            entry_number++;
        }
    }

    // ensure assessor assessed accurately
    EXPECT_FLOAT_EQ(1./num_output_enums, assessor.get_classification_accuracy());
    std::vector<double> first_confusion_state_vector = {0., 1., 2., 3., 4., 5., 6., 7., 8., 9., 10., 11., 12., 13., 14., 15., 16.,
                                                        17., 18., 19., 20., 21., 22., 23., 24.};
    AbstractInterface::DenseMatrix* expected_confusion_matrix =
            authority.dense_builder->build_by_row_major(num_output_enums, num_output_enums, first_confusion_state_vector);
    AbstractInterface::DenseMatrix* actual_confusion_matrix = assessor.get_confusion_matrix();
    expect_equal_matrix(expected_confusion_matrix, actual_confusion_matrix);
    delete expected_confusion_matrix;
}

}
}
