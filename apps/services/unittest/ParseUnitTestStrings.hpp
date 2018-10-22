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

#include <string>

const std::string InterfaceHeader = "<?xml version=\"1.0\"?>";
const std::string InterfacePerf0Block = 
"<Performer>" 
"  <Name>PlatoMain</Name>"
"  <Code>Plato_Main</Code>"
"  <CommID>0</CommID>"
"</Performer> ";
const std::string InterfacePerf1Block = 
"<Performer> "
"  <Name>LightMP_static_load1</Name>"
"  <Code>LightMP</Code>"
"  <CommID>1</CommID>"
"</Performer> ";
const std::string InterfacePerf2Block = 
"<Performer> "
"  <Name>LightMP_static_load2</Name>"
"  <Code>LightMP</Code>"
"  <CommID>2</CommID>"
"</Performer> ";
const std::string InterfaceInternalEnergy1GradientSharedDataBlock = 
"<SharedData>"
"  <Name>Internal Energy 1 Gradient</Name>"
"  <Type>Scalar</Type>"
"  <Layout>Nodal Field</Layout>"
"  <OwnerName>LightMP_static_load1</OwnerName>"
"  <UserName>PlatoMain</UserName>"
"</SharedData>";
const std::string InterfaceInternalEnergy2GradientSharedDataBlock = 
"<SharedData>"
"  <Name>Internal Energy 2 Gradient</Name>"
"  <Type>Scalar</Type>"
"  <Layout>Nodal Field</Layout>"
"  <OwnerName>LightMP_static_load2</OwnerName>"
"  <UserName>PlatoMain</UserName>"
"</SharedData>";
const std::string InterfaceInternalEnergyGradientSharedDataBlock = 
"<SharedData>"
"  <Name>Internal Energy Gradient</Name>"
"  <Type>Scalar</Type>"
"  <Layout>Nodal Field</Layout>"
"  <OwnerName>PlatoMain</OwnerName>"
"  <UserName>PlatoMain</UserName>"
"</SharedData>";
const std::string InterfaceVolumeGradientSharedDataBlock = 
"<SharedData>"
"  <Name>Volume Gradient</Name>"
"  <Type>Scalar</Type>"
"  <Layout>Nodal Field</Layout>"
"  <OwnerName>PlatoMain</OwnerName>"
"  <UserName>PlatoMain</UserName>"
"</SharedData>";
const std::string InterfaceOptimizationDOFsSharedDataBlock = 
"<SharedData>"
"  <Name>Optimization DOFs</Name>"
"  <Type>Scalar</Type>"
"  <Layout>Nodal Field</Layout>"
"  <OwnerName>PlatoMain</OwnerName>"
"  <UserName>PlatoMain</UserName>"
"  <UserName>LightMP_static_load1</UserName>"
"  <UserName>LightMP_static_load2</UserName>"
"</SharedData>";
const std::string InterfaceTopologySharedDataBlock = 
"<SharedData>"
"  <Name>Topology</Name>"
"  <Type>Scalar</Type>"
"  <Layout>Nodal Field</Layout>"
"  <OwnerName>PlatoMain</OwnerName>"
"  <UserName>PlatoMain</UserName>"
"  <UserName>LightMP_static_load1</UserName>"
"  <UserName>LightMP_static_load2</UserName>"
"</SharedData>";
const std::string InterfaceVolumeSharedDataBlock = 
"<SharedData>"
"  <Name>Volume</Name>"
"  <Type>Scalar</Type>"
"  <Layout>Mesh Global</Layout>"
"  <OwnerName>PlatoMain</OwnerName>"
"  <UserName>PlatoMain</UserName>"
"</SharedData>";
const std::string InterfaceDesignVolumeSharedDataBlock = 
"<SharedData>"
"  <Name>Design Volume</Name>"
"  <Type>Scalar</Type>"
"  <Layout>Mesh Global</Layout>"
"  <OwnerName>PlatoMain</OwnerName>"
"  <UserName>PlatoMain</UserName>"
"</SharedData>";
const std::string InterfaceInternalEnergy1SharedDataBlock = 
"<SharedData>"
"  <Name>Internal Energy 1</Name>"
"  <Type>Scalar</Type>"
"  <Layout>Mesh Global</Layout>"
"  <OwnerName>LightMP_static_load1</OwnerName>"
"  <UserName>PlatoMain</UserName>"
"</SharedData>";
const std::string InterfaceInternalEnergy2SharedDataBlock = 
"<SharedData>"
"  <Name>Internal Energy 2</Name>"
"  <Type>Scalar</Type>"
"  <Layout>Mesh Global</Layout>"
"  <OwnerName>LightMP_static_load2</OwnerName>"
"  <UserName>PlatoMain</UserName>"
"</SharedData>";
const std::string InterfaceInternalEnergySharedDataBlock = 
"<SharedData>"
"  <Name>Internal Energy</Name>"
"  <Type>Scalar</Type>"
"  <Layout>Mesh Global</Layout>"
"  <OwnerName>PlatoMain</OwnerName>"
"  <UserName>PlatoMain</UserName>"
"</SharedData>";
const std::string InterfaceOutputToFileStageBlock = 
"<Stage>"
"    <Name>Output To File</Name>"
"    <InputDataName>Topology</InputDataName>"
"    <InputDataName>Internal Energy Gradient</InputDataName>"
"    <InputDataName>Volume Gradient</InputDataName>"
"    <Operation>"
"        <Name>PlatoMainOutput</Name>"
"        <PerformerName>PlatoMain</PerformerName>"
"        <InputDataName>Topology</InputDataName>"
"        <InputDataName>Internal Energy Gradient</InputDataName>"
"        <InputDataName>Volume Gradient</InputDataName>"
"    </Operation>"
"</Stage>";
const std::string InterfaceInitializeOptimizationDOFsStageBlock = 
"<Stage>"
"  <Name>Initialize Optimization DOFs</Name>"
"  <Operation>"
"    <Name>Initialize Field</Name>"
"    <OutputDataName>Optimization DOFs</OutputDataName>"
"    <PerformerName>PlatoMain</PerformerName>"
"  </Operation>"
"  <OutputDataName>Optimization DOFs</OutputDataName>"
"</Stage>";
const std::string InterfaceFilterControlStageBlock = 
"<Stage>"
"  <Name>FilterControl</Name>"
"  <InputDataName>Optimization DOFs</InputDataName>"
"  <Operation>"
"    <Name>FilterControl</Name>"
"    <InputDataName>Optimization DOFs</InputDataName>"
"    <PerformerName>PlatoMain</PerformerName>"
"    <OutputDataName>Topology</OutputDataName>"
"  </Operation>"
"  <OutputDataName>Topology</OutputDataName>"
"</Stage>";
const std::string InterfaceDesignVolumeStageBlock = 
"<Stage>"
"  <Name>Design Volume</Name>"
"  <Operation>"
"    <Name>Design Volume</Name>"
"    <OutputDataName>Design Volume</OutputDataName>"
"    <PerformerName>PlatoMain</PerformerName>"
"  </Operation>"
"  <OutputDataName>Design Volume</OutputDataName>"
"</Stage>";
const std::string InterfaceVolumeStageBlock = 
"<Stage>"
"  <Name>Volume</Name>"
"  <InputDataName>Optimization DOFs</InputDataName>"
"  <InputDataName>Topology</InputDataName>"
"  <Operation>"
"    <Name>Compute Current Volume</Name>"
"    <InputDataName>Topology</InputDataName>"
"    <OutputDataName>Volume</OutputDataName>"
"    <OutputDataName>Volume Gradient</OutputDataName>"
"    <PerformerName>PlatoMain</PerformerName>"
"  </Operation>"
"  <Operation>"
"    <Name>FilterConstraintGradient</Name>"
"    <PerformerName>PlatoMain</PerformerName>"
"    <InputDataName>Optimization DOFs</InputDataName>"
"    <InputDataName>Volume Gradient</InputDataName>"
"    <OutputDataName>Volume Gradient</OutputDataName>"
"  </Operation>"
"  <OutputDataName>Volume</OutputDataName>"
"  <OutputDataName>Volume Gradient</OutputDataName>"
"</Stage>";
const std::string InterfaceInternalEnergyStageBlock = 
"<Stage>"
"  <Name>Internal Energy</Name>"
"  <InputDataName>Optimization DOFs</InputDataName>"
"  <InputDataName>Topology</InputDataName>"
"  <Operation>"
"    <Name>Compute Internal Energy</Name>"
"    <InputDataName>Topology</InputDataName>"
"    <PerformerName>LightMP_static_load1</PerformerName>"
"    <PerformerName>LightMP_static_load2</PerformerName>"
"    <OutputDataName>Internal Energy 1</OutputDataName>"
"    <OutputDataName>Internal Energy 2</OutputDataName>"
"    <OutputDataName>Internal Energy 1 Gradient</OutputDataName>"
"    <OutputDataName>Internal Energy 2 Gradient</OutputDataName>"
"  </Operation>"
"  <Operation>"
"    <Name>AggregateEnergy</Name>"
"    <InputDataName>Internal Energy 1</InputDataName>"
"    <InputDataName>Internal Energy 2</InputDataName>"
"    <InputDataName>Internal Energy 1 Gradient</InputDataName>"
"    <InputDataName>Internal Energy 2 Gradient</InputDataName>"
"    <PerformerName>PlatoMain</PerformerName>"
"    <OutputDataName>Internal Energy</OutputDataName>"
"    <OutputDataName>Internal Energy Gradient</OutputDataName>"
"  </Operation>"
"  <Operation>"
"    <Name>FilterObjectiveGradient</Name>"
"    <PerformerName>PlatoMain</PerformerName>"
"    <InputDataName>Optimization DOFs</InputDataName>"
"    <InputDataName>Internal Energy Gradient</InputDataName>"
"    <OutputDataName>Internal Energy Gradient</OutputDataName>"
"  </Operation>"
"  <OutputDataName>Internal Energy</OutputDataName>"
"  <OutputDataName>Internal Energy Gradient</OutputDataName>"
"</Stage>";
const std::string InterfaceFilterBlock = 
"<Filter>"
"  <Name>Kernel</Name>"
"  <Scale>1.48</Scale>"
"  <Absolute>-1.0</Absolute>"
"</Filter>";
const std::string InterfaceOptimizerBlock = 
"<Optimizer>"
"  <Package>OC</Package>"
"  <NLopt>"
"    <Method>MMA</Method>"
"  </NLopt>"
"  <Output>"
"    <OutputStage>Output To File</OutputStage>"
"  </Output>"
"  <OC>"
"    <MoveLimiter>1.0</MoveLimiter>"
"    <StabilizationParameter>0.5</StabilizationParameter>"
"    <UseNewtonSearch>True</UseNewtonSearch>"
"    <ConstraintMaxIterations>10</ConstraintMaxIterations>"
"  </OC>"
"  <OptimizationVariables>"
"    <ValueName>Optimization DOFs</ValueName>"
"    <FilteredName>Topology</FilteredName>"
"    <InitializationStage>Initialize Optimization DOFs</InitializationStage>"
"  </OptimizationVariables>"
"  <Objective>"
"    <ValueName>Internal Energy</ValueName>"
"    <GradientName>Internal Energy Gradient</GradientName>"
"  </Objective>"
"  <BoundConstraint>"
"    <Upper>1.0</Upper>"
"    <Lower>0.0</Lower>"
"  </BoundConstraint>"
"  <Constraint>"
"    <Equality>True</Equality>"
"    <Linear>True</Linear>"
"    <ValueName>Volume</ValueName>"
"    <ReferenceValueName>Design Volume</ReferenceValueName>"
"    <GradientName>Volume Gradient</GradientName>"
"    <TargetValue>0.2</TargetValue>"
"    <Tolerance>1e-3</Tolerance>"
"  </Constraint>"
"  <Convergence>"
"    <ComboType>OR</ComboType>"
"    <Test>"
"      <Type>Maximum Iterations</Type>"
"      <Value>10</Value>"
"    </Test>"
"    <Test>"
"      <ComboType>AND</ComboType>"
"      <Test>"
"        <Type>Minimum Iterations</Type>"
"        <Value>5</Value>"
"      </Test>"
"      <Test>"
"        <ComboType>OR</ComboType>"
"        <Test> "
"          <Type>Relative Topology Change</Type>"
"          <Value>1e-4</Value>"
"        </Test>"
"        <Test> "
"          <Type>Relative Objective Change</Type>"
"          <Value>1e-4</Value>"
"        </Test>"
"      </Test>"
"    </Test>"
"  </Convergence>"
"</Optimizer>";
const std::string InterfaceMeshBlock = 
"<mesh>"
"  <type>unstructured</type>"
"  <format>exodus</format>"
"  <mesh>unsmesh</mesh>"
"  <block>"
"    <index>1</index>"
"    <integration>"
"      <type>gauss</type>"
"      <order>2</order>"
"    </integration>"
"    <material>1</material>"
"  </block>"
"</mesh>";
const std::string InterfaceOutputBlock = 
"<output>"
"  <file>plato</file>"
"  <format>exodus</format>"
"</output>";

const std::string PlatoAppHeader = "<?xml version=\"1.0\"?>";

const std::string PlatoAppPlatoMainOutputOperationBlock = 
"<Operation>"
"  <Function>PlatoMainOutput</Function>"
"  <Name>PlatoMainOutput</Name>"
"  <Topology>"
"    <Name>Topology</Name>"
"  </Topology>"
"  <ObjectiveGradient>"
"    <Name>Internal Energy Gradient</Name>"
"  </ObjectiveGradient>"
"  <ConstraintGradient>"
"    <Name>Volume Gradient</Name>"
"  </ConstraintGradient>"
"</Operation>";
const std::string PlatoAppFilterControlOperationBlock = 
"<Operation>"
"  <Function>FilterControl</Function>"
"  <Name>FilterControl</Name>"
"  <InputTopology>"
"    <Name>Optimization DOFs</Name>"
"  </InputTopology>"
"  <OutputTopology>"
"    <Name>Topology</Name>"
"  </OutputTopology>"
"</Operation>";
const std::string PlatoAppFilterObjectiveGradientOperationBlock = 
"<Operation>"
"  <Function>FilterObjectiveGradient</Function>"
"  <Name>FilterObjectiveGradient</Name>"
"  <InputTopology>"
"    <Name>Optimization DOFs</Name>"
"  </InputTopology>"
"  <ObjectiveGradient>"
"    <Name>Internal Energy Gradient</Name>"
"  </ObjectiveGradient>"
"</Operation>";
const std::string PlatoAppFilterConstraintGradientOperationBlock = 
"<Operation>"
"  <Function>FilterConstraintGradient</Function>"
"  <Name>FilterConstraintGradient</Name>"
"  <InputTopology>"
"    <Name>Optimization DOFs</Name>"
"  </InputTopology>"
"  <ConstraintGradient>"
"    <Name>Volume Gradient</Name>"
"  </ConstraintGradient>"
"</Operation>";
const std::string PlatoAppInitializeFieldOperationBlock = 
"<Operation>"
"  <Function>InitializeField</Function>"
"  <Name>Initialize Field</Name>"
"  <Method>Uniform</Method>"
"  <Uniform>"
"    <Value>0.25</Value>"
"  </Uniform>"
"  <FromFile>"
"    <Name>YoMama.exo</Name>"
"    <VariableName>density</VariableName>"
"  </FromFile>"
"  <OutputValue>"
"    <Name>Optimization DOFs</Name>"
"  </OutputValue>"
"</Operation>";
const std::string PlatoAppDesignVolumeOperationBlock = 
"<Operation>"
"  <Function>DesignVolume</Function>"
"  <Name>Design Volume</Name>"
"  <Blocks>1</Blocks>"
"  <OutputValue>"
"    <Name>Design Volume</Name>"
"  </OutputValue>"
"</Operation>";
const std::string PlatoAppComputeVolumeOperationBlock = 
"<Operation>"
"  <Function>ComputeVolume</Function>"
"  <Name>Compute Current Volume</Name>"
"  <Topology>"
"    <Name>Topology</Name>"
"  </Topology>"
"  <PenaltyModel>SIMP</PenaltyModel>"
"  <SIMP>"
"    <PenaltyExponent>1.0</PenaltyExponent>"
"    <MinimumValue>0.0</MinimumValue>"
"  </SIMP>"
"  <OutputValue>"
"    <Name>Volume</Name>"
"  </OutputValue>"
"  <OutputGradient>"
"    <Name>Volume Gradient</Name>"
"  </OutputGradient>"
"</Operation>";
const std::string PlatoAppAggregatorOperationBlock = 
"<Operation>"
"  <Function>Aggregator</Function>"
"  <Name>AggregateEnergy</Name>"
"  <Weighting>"
"    <Weight>"
"      <Value>0.50</Value>"
"    </Weight>"
"    <Weight>"
"      <Value>0.50</Value>"
"    </Weight>"
"  </Weighting>"
"  <Aggregate>"
"    <Layout>Value</Layout>"
"    <Input>"
"      <Name>Internal Energy 1</Name>"
"      <Weight>0.5</Weight>"
"    </Input>"
"    <Input>"
"      <Name>Internal Energy 2</Name>"
"      <Weight>0.5</Weight>"
"    </Input>"
"    <Output>"
"      <Name>Internal Energy</Name>"
"    </Output>"
"  </Aggregate>"
"  <Aggregate>"
"    <Layout>Field</Layout>"
"    <Input>"
"      <Name>Internal Energy 1 Gradient</Name>"
"      <Weight>0.5</Weight>"
"    </Input>"
"    <Input>"
"      <Name>Internal Energy 2 Gradient</Name>"
"      <Weight>0.5</Weight>"
"    </Input>"
"    <Output>"
"      <Name>Internal Energy Gradient</Name>"
"    </Output>"
"  </Aggregate>"
"</Operation>";
const std::string PlatoMainMeshBlock = 
"<mesh>"
"  <type>unstructured</type>"
"  <format>exodus</format>"
"  <mesh>unsmesh</mesh>"
"  <block>"
"    <index>1</index>"
"    <integration>"
"      <type>gauss</type>"
"      <order>2</order>"
"    </integration>"
"    <material>1</material>"
"  </block>"
"</mesh>";
const std::string PlatoMainOutputBlock = 
"<output>"
"  <file>platomain</file>"
"  <format>exodus</format>"
"</output>";









