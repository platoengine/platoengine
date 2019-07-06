/*
 * Plato_Operations_incl.hpp
 *
 *  Created on: Jun 27, 2019
 */

#pragma once

#include "Plato_Filter.hpp"
#include "Plato_Roughness.hpp"
#include "Plato_Aggregator.hpp"
#include "Plato_DesignVolume.hpp"
#include "Plato_EnforceBounds.hpp"
#include "Plato_UpdateProblem.hpp"
#include "Plato_ComputeVolume.hpp"
#include "Plato_SetUpperBounds.hpp"
#include "Plato_SetLowerBounds.hpp"
#include "Plato_PlatoMainOutput.hpp"
#include "Plato_InitializeField.hpp"
#include "Plato_InitializeValues.hpp"
#include "Plato_WriteGlobalValue.hpp"
#include "Plato_OperationsUtilities.hpp"
#include "Plato_MeanPlusVarianceMeasure.hpp"
#include "Plato_NormalizeObjectiveValue.hpp"
#include "Plato_ReciprocateObjectiveValue.hpp"
#include "Plato_NormalizeObjectiveGradient.hpp"
#include "Plato_ReciprocateObjectiveGradient.hpp"

#ifdef GEOMETRY
#include "Plato_MapMLSField.hpp"
#include "Plato_MetaDataMLS.hpp"
#include "Plato_ComputeMLSField.hpp"
#include "Plato_InitializeMLSPoints.hpp"
#endif
