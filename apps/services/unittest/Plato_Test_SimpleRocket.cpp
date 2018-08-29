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

#include "Plato_SimpleRocket.hpp"

namespace PlatoTest
{

TEST(PlatoTest, SimpleRocket)
{
    Plato::SimpleRocket<double> tDriver;
    tDriver.disableOutput();
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

    std::vector<double> tGoldThrust = {0, 1656714.377766964, 1684717.520617273, 1713123.001583093, 1741935.586049868,
                                       1771160.083875437, 1800801.349693849, 1830864.28322051, 1861353.829558637,
                                       1892274.979507048, 1923632.769869272, 1955432.283763989, 1987678.650936801,
                                       2020377.048073344, 2053532.699113719, 2087150.875568287, 2121236.896834771,
                                       2155796.130516737, 2190833.992743404, 2226355.948490792, 2262367.511904243,
                                       2298874.246622283, 2335881.766101836, 2373395.733944806, 2411421.864226017,
                                       2449965.921822503, 2489033.722744186, 2528631.134465915, 2568764.076260844,
                                       2609438.519535244, 2650660.488164633, 2692436.058831303, 2734771.361363255,
                                       2777672.579074459, 2821145.949106557, 2865197.762771913, 2909834.365898075,
                                       2955062.159173611, 3000887.598495364, 3047317.195317072, 3094357.516999425,
                                       3142015.18716148, 3190296.886033527, 3239209.350811319, 3288759.376011737,
                                       3338953.813829865, 3389799.574497465, 3441303.626642879, 3493472.997652346,
                                       3546314.774032734, 3599836.101775718, 3654044.186723352, 3708946.294935087,
                                       3764549.753056224, 3820861.948687783, 3877890.330757833, 3935642.409894215,
                                       3994125.758798767, 4053348.012622938, 4113316.869344868, 4174040.090147917,
                                       4235525.499800648, 4297780.987038235, 4360814.504945371, 4424634.071340578,
                                       4489247.76916203, 4554663.746854796, 4620890.218759571, 4687935.465502855,
                                       4755807.834388626, 4824515.739791448, 4894067.663551098, 4964472.155368621,
                                       5035737.83320389, 5107873.383674653, 5180887.562457044, 5254789.194687578,
                                       5329587.175366664, 5405290.469763565, 5481908.11382287, 5559449.214572486,
                                       5637922.950533082, 5717338.572129052, 5797705.402100981, 5879032.835919643,
                                       5961330.342201422, 6044607.46312535, 6128873.814851565, 6214139.087941348,
                                       6300413.047778608, 6387705.534992979, 6476026.465884338, 6565385.832848894,
                                       6655793.704806847, 6747260.227631442, 6839795.624579719, 6933410.196724654,
                                       7028114.32338894, 7123918.462580209, 7220833.151427887};

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

} // namespace PlatoTest
