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

/*
 * Plato_Test_SimpleRocket.cpp
 *
 *  Created on: Aug 27, 2018
 */

#include "gtest/gtest.h"

#include <mpi.h>

#include "Plato_ErrorChecks.hpp"
#include "Plato_SharedValue.hpp"
#include "Plato_Communication.hpp"
#include "Plato_AppErrorChecks.hpp"
#include "Plato_StandardMultiVector.hpp"
#include "Plato_AlgebraicRocketModel.hpp"

namespace PlatoTest
{

TEST(PlatoTest, AlgebraicRocketModel)
{
    Plato::AlgebraicRocketModel<double> tDriver;
    tDriver.solve();

    std::vector<double> tTimes = tDriver.getTimeProfile();
    std::vector<double> tThrustProfile = tDriver.getThrustProfile();
    std::vector<double> tPressureProfile = tDriver.getPressuresProfile();

    const double tTolerance = 1e-4;
    std::vector<double> tGoldTimes = {0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8,
                                      1.9, 2, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7,
                                      3.8, 3.9, 4, 4.1, 4.2, 4.3, 4.4, 4.5, 4.6, 4.7, 4.8, 4.9, 5, 5.1, 5.2, 5.3, 5.4, 5.5, 5.6,
                                      5.7, 5.8, 5.9, 6, 6.1, 6.2, 6.3, 6.4, 6.5, 6.6, 6.7, 6.8, 6.9, 7, 7.1, 7.2, 7.3, 7.4, 7.5,
                                      7.6, 7.7, 7.8, 7.9, 8, 8.1, 8.2, 8.3, 8.4, 8.5, 8.6, 8.7, 8.8, 8.9, 9, 9.1, 9.2, 9.3, 9.4,
                                      9.5, 9.6, 9.7, 9.8, 9.9};

    std::vector<double> tGoldThrust =
        { 0, 6796.7769344285716215, 6867.4666589320704588, 6938.7140813601199625, 7010.5222074219882415,
                7082.8940515426593265, 7155.8326368664666006, 7229.3409952607416926, 7303.4221673194269897,
                7378.0792023667154353, 7453.3151584606503093, 7529.1331023967359215, 7605.5361097115401208,
                7682.5272646862767942, 7760.1096603504056475, 7838.2863984852019712, 7917.0605896273045801,
                7996.4353530723155927, 8076.4138168783356377, 8156.9991178695054259, 8238.1944016395609651,
                8320.0028225553669472, 8402.427543760421031, 8485.4717371784081479, 8569.138583516696599,
                8653.4312722698141442, 8738.3530017229986697, 8823.9069789556815522, 8910.0964198449109972,
                8996.9245490689172584, 9084.3946001105268806, 9172.5098152606296935, 9261.2734456216821854,
                9350.6887511110835476, 9440.7590004646899615, 9531.4874712402197474, 9622.8774498206785211,
                9714.9322314177934459, 9807.6551200754420279, 9901.0494286730263411, 9995.1184789289254695,
                10089.86560140385518, 10185.29413550427671, 10281.40742948577099, 10378.208840456431062,
                10475.701734380227208, 10573.889486080362985, 10672.775479242660367, 10772.363106418877578,
                10872.655769030066949, 10973.656877369950962, 11075.369850608207344, 11177.798116793806912,
                11280.945112858358698, 11384.814284619407772, 11489.409086783754901, 11594.732982950727092,
                11700.789445615524528, 11807.581956172476566, 11915.114004918334103, 12023.389091055551035,
                12132.410722695569348, 12242.182416862038735, 12352.707699494156259, 12463.990105449835028,
                12576.03317850901476, 12688.840471376890491, 12802.415545687126723, 12916.761972005117059,
                13031.883329831192896, 13147.783207603848496, 13264.465202702955139, 13381.932921452975279,
                13500.189979126145772, 13619.239999945690215, 13739.086617089016727, 13859.733472690882991,
                13981.184217846597676, 14103.442512615172745, 14226.512026022497594, 14350.396436064524096,
                14475.099429710398908, 14600.624702905623053, 14726.975960575182398, 14854.156916626747261,
                14982.171293953710119, 15111.022824438417956, 15240.715248955209972, 15371.252317373619007,
                15502.637788561391062, 15634.875430387684901, 15767.969019726122497, 15901.922342457881314,
                16036.739193474852073, 16172.423376682612798, 16308.97870500362842, 16446.409000380237558,
                16584.718093777799368, 16723.909825187685783, 16863.988043630397442 };

    std::vector<double> tGoldPressure = {3.5e+06, 3189475.86890643, 3222646.886584704, 3256079.602944708, 3289775.428409753,
                                         3323735.777492982, 3357962.068799077, 3392455.72502597, 3427218.172966537,
                                         3462250.84351031, 3497555.171645161, 3533132.596459002, 3568984.561141468,
                                         3605112.512985606, 3641517.903389558, 3678202.18785824, 3715166.826005007,
                                         3752413.281553337, 3789943.022338498, 3827757.520309201, 3865858.251529277,
                                         3904246.696179328, 3942924.338558375, 3981892.667085526, 4021153.174301613,
                                         4060707.356870828, 4100556.715582388, 4140702.755352164, 4181146.985224297,
                                         4221890.918372861, 4262936.072103474, 4304283.967854919, 4345936.131200789,
                                         4387894.091851073, 4430159.383653805, 4472733.544596657, 4515618.116808556,
                                         4558814.646561285, 4602324.684271098, 4646149.784500302, 4690291.505958878,
                                         4734751.411506055, 4779531.068151912, 4824632.04705896, 4870055.923543734,
                                         4915804.277078374, 4961878.691292197, 5008280.753973287, 5055012.057070049,
                                         5102074.196692795, 5149468.773115309, 5197197.390776409, 5245261.658281502,
                                         5293663.188404156, 5342403.598087644, 5391484.508446511, 5440907.544768098,
                                         5490674.336514118, 5540786.517322188, 5591245.725007365, 5642053.601563688,
                                         5693211.793165727, 5744721.950170086, 5796585.72711697, 5848804.782731681,
                                         5901380.779926158, 5954315.385800499, 6007610.271644478, 6061267.112939054,
                                         6115287.589357896, 6169673.384768886, 6224426.187235633, 6279547.689018974,
                                         6335039.586578473, 6390903.580573932, 6447141.375866881, 6503754.681522069,
                                         6560745.210808973, 6618114.681203265, 6675864.814388307, 6733997.336256648,
                                         6792513.976911491, 6851416.470668174, 6910706.556055644, 6970385.975817952,
                                         7030456.476915687, 7090919.810527483, 7151777.73205145, 7213032.001106677,
                                         7274684.381534644, 7336736.641400733, 7399190.552995646, 7462047.892836868,
                                         7525310.441670142, 7588979.984470865, 7653058.310445594, 7717547.213033441,
                                         7782448.48990756, 7847763.942976533, 7913495.378385854};

    for(size_t tIndex = 0; tIndex < tTimes.size(); tIndex++)
    {
        EXPECT_NEAR(tTimes[tIndex], tGoldTimes[tIndex], tTolerance);
        EXPECT_NEAR(tThrustProfile[tIndex], tGoldThrust[tIndex], tTolerance);
        EXPECT_NEAR(tPressureProfile[tIndex], tGoldPressure[tIndex], tTolerance);
    }
}

TEST(PlatoTest, IsOperationDefinedCheck)
{
    std::vector<std::string> tNames = {"SetNormalizationConstants", "ObjectiveValue", "ObjectiveGradient"};
    ASSERT_NO_THROW(Plato::check_operation_definition("SetNormalizationConstants", tNames));
    ASSERT_NO_THROW(Plato::check_operation_definition("ObjectiveValue", tNames));
    ASSERT_NO_THROW(Plato::check_operation_definition("ObjectiveGradient", tNames));
    ASSERT_THROW(Plato::check_operation_definition("InequalityGradient", tNames), std::invalid_argument);
}

TEST(PlatoTest, IsSharedDataLayoutDefinedCheck)
{
    ASSERT_NO_THROW(Plato::check_data_layout(Plato::data::SCALAR_FIELD, Plato::data::SCALAR_FIELD));
    ASSERT_THROW(Plato::check_data_layout(Plato::data::SCALAR_FIELD, Plato::data::VECTOR_FIELD), std::invalid_argument);
    ASSERT_THROW(Plato::check_data_layout(Plato::data::SCALAR_FIELD, Plato::data::TENSOR_FIELD), std::invalid_argument);
    ASSERT_THROW(Plato::check_data_layout(Plato::data::SCALAR_FIELD, Plato::data::ELEMENT_FIELD), std::invalid_argument);
    ASSERT_THROW(Plato::check_data_layout(Plato::data::SCALAR_FIELD, Plato::data::SCALAR_PARAMETER), std::invalid_argument);

    ASSERT_NO_THROW(Plato::check_data_layout(Plato::data::VECTOR_FIELD, Plato::data::VECTOR_FIELD));
    ASSERT_THROW(Plato::check_data_layout(Plato::data::VECTOR_FIELD, Plato::data::SCALAR_FIELD), std::invalid_argument);
    ASSERT_THROW(Plato::check_data_layout(Plato::data::VECTOR_FIELD, Plato::data::TENSOR_FIELD), std::invalid_argument);
    ASSERT_THROW(Plato::check_data_layout(Plato::data::VECTOR_FIELD, Plato::data::ELEMENT_FIELD), std::invalid_argument);
    ASSERT_THROW(Plato::check_data_layout(Plato::data::VECTOR_FIELD, Plato::data::SCALAR_PARAMETER), std::invalid_argument);

    ASSERT_NO_THROW(Plato::check_data_layout(Plato::data::TENSOR_FIELD, Plato::data::TENSOR_FIELD));
    ASSERT_THROW(Plato::check_data_layout(Plato::data::TENSOR_FIELD, Plato::data::SCALAR_FIELD), std::invalid_argument);
    ASSERT_THROW(Plato::check_data_layout(Plato::data::TENSOR_FIELD, Plato::data::VECTOR_FIELD), std::invalid_argument);
    ASSERT_THROW(Plato::check_data_layout(Plato::data::TENSOR_FIELD, Plato::data::ELEMENT_FIELD), std::invalid_argument);
    ASSERT_THROW(Plato::check_data_layout(Plato::data::TENSOR_FIELD, Plato::data::SCALAR_PARAMETER), std::invalid_argument);

    ASSERT_NO_THROW(Plato::check_data_layout(Plato::data::ELEMENT_FIELD, Plato::data::ELEMENT_FIELD));
    ASSERT_THROW(Plato::check_data_layout(Plato::data::ELEMENT_FIELD, Plato::data::SCALAR_FIELD), std::invalid_argument);
    ASSERT_THROW(Plato::check_data_layout(Plato::data::ELEMENT_FIELD, Plato::data::VECTOR_FIELD), std::invalid_argument);
    ASSERT_THROW(Plato::check_data_layout(Plato::data::ELEMENT_FIELD, Plato::data::TENSOR_FIELD), std::invalid_argument);
    ASSERT_THROW(Plato::check_data_layout(Plato::data::ELEMENT_FIELD, Plato::data::SCALAR_PARAMETER), std::invalid_argument);

    ASSERT_NO_THROW(Plato::check_data_layout(Plato::data::SCALAR_PARAMETER, Plato::data::SCALAR_PARAMETER));
    ASSERT_THROW(Plato::check_data_layout(Plato::data::SCALAR_PARAMETER, Plato::data::SCALAR_FIELD), std::invalid_argument);
    ASSERT_THROW(Plato::check_data_layout(Plato::data::SCALAR_PARAMETER, Plato::data::VECTOR_FIELD), std::invalid_argument);
    ASSERT_THROW(Plato::check_data_layout(Plato::data::SCALAR_PARAMETER, Plato::data::TENSOR_FIELD), std::invalid_argument);
    ASSERT_THROW(Plato::check_data_layout(Plato::data::SCALAR_PARAMETER, Plato::data::ELEMENT_FIELD), std::invalid_argument);
}

TEST(PlatoTest, IsSharedDataArgumentDefinedCheck)
{
    std::map<std::string, std::vector<double>> tSharedDataMap;

    const size_t tLength = 1;
    std::string tName = "ThrustMisfitObjective";
    tSharedDataMap[tName] = std::vector<double>(tLength);
    tName = "DesignVariables";
    const size_t tNumDesigVariables = 2;
    tSharedDataMap[tName] = std::vector<double>(tNumDesigVariables);
    tName = "ThrustMisfitObjectiveGradient";
    tSharedDataMap[tName] = std::vector<double>(tNumDesigVariables);
    tName = "UpperBounds";
    tSharedDataMap[tName] = std::vector<double>(tNumDesigVariables);

    ASSERT_NO_THROW(Plato::check_shared_data_argument_definition("UpperBounds", tSharedDataMap));
    ASSERT_NO_THROW(Plato::check_shared_data_argument_definition("DesignVariables", tSharedDataMap));
    ASSERT_NO_THROW(Plato::check_shared_data_argument_definition("ThrustMisfitObjective", tSharedDataMap));
    ASSERT_NO_THROW(Plato::check_shared_data_argument_definition("ThrustMisfitObjectiveGradient", tSharedDataMap));
    ASSERT_THROW(Plato::check_shared_data_argument_definition("InequalityGradient", tSharedDataMap), std::invalid_argument);
}

} // namespace PlatoTest
