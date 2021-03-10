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
 * Plato_Operations_incl.hpp
 *
 *  Created on: Jun 27, 2019
 */

#pragma once

#include "Plato_Filter.hpp"
#include "Plato_CopyField.hpp"
#include "Plato_CopyValue.hpp"
#include "Plato_Roughness.hpp"
#include "Plato_SystemCall.hpp"
#include "Plato_Aggregator.hpp"
#include "Plato_DesignVolume.hpp"
#include "Plato_EnforceBounds.hpp"
#include "Plato_UpdateProblem.hpp"
#include "Plato_ComputeVolume.hpp"
#include "Plato_CSMMeshOutput.hpp"
#include "Plato_SetUpperBounds.hpp"
#include "Plato_SetLowerBounds.hpp"
#include "Plato_PlatoMainOutput.hpp"
#include "Plato_InitializeField.hpp"
#include "Plato_InitializeValues.hpp"
#include "Plato_WriteGlobalValue.hpp"
#include "Plato_CSMParameterOutput.hpp"
#include "Plato_OperationsUtilities.hpp"
#include "Plato_NormalizeObjectiveValue.hpp"
#include "Plato_MeanPlusVarianceMeasure.hpp"
#include "Plato_MeanPlusVarianceGradient.hpp"
#include "Plato_ReciprocateObjectiveValue.hpp"
#include "Plato_NormalizeObjectiveGradient.hpp"
#include "Plato_OutputNodalFieldSharedData.hpp"
#include "Plato_ReciprocateObjectiveGradient.hpp"

#ifdef GEOMETRY
#include "Plato_MapMLSField.hpp"
#include "Plato_MetaDataMLS.hpp"
#include "Plato_ComputeMLSField.hpp"
#include "Plato_InitializeMLSPoints.hpp"
#endif
