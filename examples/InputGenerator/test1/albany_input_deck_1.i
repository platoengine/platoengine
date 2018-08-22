<?xml version="1.0"?>
<ParameterList>
  <ParameterList name="Problem">
    <Parameter name="Name" type="string" value="Poissons Equation 3D" />
    <ParameterList name="Topologies">
      <Parameter name="Number of Topologies" type="int" value="1" />
      <ParameterList name="Topology 0">
        <Parameter name="Topology Name" type="string" value="Rho" />
        <Parameter name="Entity Type" type="string" value="State Variable" />
        <Parameter name="Bounds" type="Array(double)" value="{0.0,1.0}" />
        <Parameter name="Initial Value" type="double" value=".5" />
        <ParameterList name="Functions">
          <Parameter name="Number of Functions" type="int" value="1" />
          <ParameterList name="Function 0">
            <Parameter name="Function Type" type="string" value="SIMP" />
            <Parameter name="Minimum" type="double" value="0.001" />
            <Parameter name="Penalization Parameter" type="double" value="3" />
          </ParameterList>
        </ParameterList>
        <Parameter name="Spatial Filter" type="int" value="0" />
      </ParameterList>
    </ParameterList>
    <ParameterList name="Configuration">
      <ParameterList name="Element Blocks">
        <Parameter name="Number of Element Blocks" type="int" value="1" />
        <ParameterList name="Element Block 0">
          <Parameter name="Name" type="string" value="block_1" />
          <ParameterList name="Material">
            <Parameter name="Elastic Modulus" type="double" value="1e9" />
            <Parameter name="Poissons Ratio" type="double" value=".3" />
            <Parameter name="Isotropic Modulus" type="double" value="1e5" />
          </ParameterList>
        </ParameterList>
      </ParameterList>
    </ParameterList>
    <ParameterList name="Dirichlet BCs">
      <Parameter name="DBC on NS nodelist_2 for DOF P" type="double" value="0.0" />
    </ParameterList>
    <ParameterList name="Neumann BCs">
      <Parameter name="NBC on SS surface_1 for DOF P set (dudx, dudy, dudz)" type="Array(double)" value="{100,0.0,0.0}" />
    </ParameterList>
    <ParameterList name="Apply Topology Weight Functions">
      <Parameter name="Number of Fields" type="int" value="1" />
      <ParameterList name="Field 0">
        <Parameter name="Name" type="string" value="kinVar" />
        <Parameter name="Layout" type="string" value="QP Vector" />
        <Parameter name="Topology Index" type="int" value="0" />
        <Parameter name="Function Index" type="int" value="0" />
      </ParameterList>
    </ParameterList>
    <ParameterList name="Response Functions">
      <Parameter name="Number of Response Vectors" type="int" value="1" />
      <ParameterList name="Response Vector 0">
        <Parameter name="Name" type="string" value="Stiffness Objective" />
        <Parameter name="Gradient Field Name" type="string" value="Phi Gradient" />
        <Parameter name="Gradient Field Layout" type="string" value="QP Vector" />
        <Parameter name="Work Conjugate Name" type="string" value="kinVar" />
        <Parameter name="Work Conjugate Layout" type="string" value="QP Vector" />
        <Parameter name="Topology Index" type="int" value="0" />
        <Parameter name="Function Index" type="int" value="0" />
        <Parameter name="Response Name" type="string" value="R0" />
        <Parameter name="Response Derivative Name" type="string" value="dR0dRho" />
      </ParameterList>
    </ParameterList>
  </ParameterList>
  <ParameterList name="Discretization">
    <Parameter name="Method" type="string" value="Ioss" />
    <Parameter name="Exodus Input File Name" type="string" value="plato_test1.gen" />
    <Parameter name="Exodus Output File Name" type="string" value="plato_test1_alb_1.exo" />
    <Parameter name="Separate Evaluators by Element Block" type="bool" value="true" />
  </ParameterList>
  <ParameterList name="Piro">
    <ParameterList name="LOCA">
      <ParameterList name="Bifurcation" />
      <ParameterList name="Constraints" />
      <ParameterList name="Predictor">
        <ParameterList name="First Step Predictor" />
        <ParameterList name="Last Step Predictor" />
      </ParameterList>
      <ParameterList name="Step Size" />
      <ParameterList name="Stepper">
        <ParameterList name="Eigensolver" />
      </ParameterList>
    </ParameterList>
    <ParameterList name="NOX">
      <ParameterList name="Status Tests">
        <Parameter name="Test Type" type="string" value="Combo" />
        <Parameter name="Combo Type" type="string" value="OR" />
        <Parameter name="Number of Tests" type="int" value="2" />
        <ParameterList name="Test 0">
          <Parameter name="Test Type" type="string" value="NormF" />
          <Parameter name="Norm Type" type="string" value="Two Norm" />
          <Parameter name="Scale Type" type="string" value="Scaled" />
          <Parameter name="Tolerance" type="double" value="1e-8" />
        </ParameterList>
        <ParameterList name="Test 1">
          <Parameter name="Test Type" type="string" value="MaxIters" />
          <Parameter name="Maximum Iterations" type="int" value="20" />
        </ParameterList>
        <ParameterList name="Test 2">
          <Parameter name="Test Type" type="string" value="NormUpdate" />
          <Parameter name="Norm Type" type="string" value="Two Norm" />
          <Parameter name="Scale Type" type="string" value="Scaled" />
          <Parameter name="Tolerance" type="double" value="1e-8" />
        </ParameterList>
      </ParameterList>
      <ParameterList name="Direction">
        <Parameter name="Method" type="string" value="Newton" />
        <ParameterList name="Newton">
          <Parameter name="Forcing Term Method" type="string" value="Constant" />
          <Parameter name="Rescue Bad Newton Solve" type="bool" value="1" />
          <ParameterList name="Stratimikos Linear Solver">
            <ParameterList name="NOX Stratimikos Options" />
            <ParameterList name="Stratimikos">
              <Parameter name="Linear Solver Type" type="string" value="Belos" />
              <ParameterList name="Linear Solver Types">
                <ParameterList name="AztecOO">
                  <ParameterList name="Forward Solve">
                    <ParameterList name="AztecOO Settings">
                      <Parameter name="Aztec Solver" type="string" value="GMRES" />
                      <Parameter name="Convergence Test" type="string" value="r0" />
                      <Parameter name="Size of Krylov Subspace" type="int" value="200" />
                      <Parameter name="Output Frequency" type="int" value="10" />
                    </ParameterList>
                    <Parameter name="Max Iterations" type="int" value="200" />
                    <Parameter name="Tolerance" type="double" value="1e-5" />
                  </ParameterList>
                </ParameterList>
                <ParameterList name="Belos">
                  <Parameter name="Solver Type" type="string" value="Block GMRES" />
                  <ParameterList name="Solver Types">
                    <ParameterList name="Block GMRES">
                      <Parameter name="Convergence Tolerance" type="double" value="1e-12" />
                      <Parameter name="Output Frequency" type="int" value="2" />
                      <Parameter name="Output Style" type="int" value="1" />
                      <Parameter name="Verbosity" type="int" value="0" />
                      <Parameter name="Maximum Iterations" type="int" value="200" />
                      <Parameter name="Block Size" type="int" value="1" />
                      <Parameter name="Num Blocks" type="int" value="200" />
                      <Parameter name="Flexible Gmres" type="bool" value="0" />
                    </ParameterList>
                  </ParameterList>
                </ParameterList>
              </ParameterList>
              <Parameter name="Preconditioner Type" type="string" value="Ifpack2" />
              <ParameterList name="Preconditioner Types">
                <ParameterList name="Ifpack2">
                  <Parameter name="Overlap" type="int" value="2" />
                  <Parameter name="Prec Type" type="string" value="ILUT" />
                  <ParameterList name="Ifpack2 Settings">
                    <Parameter name="fact: drop tolerance" type="double" value="0" />
                    <Parameter name="fact: ilut level-of-fill" type="double" value="1" />
                  </ParameterList>
                  <ParameterList name="VerboseObject">
                    <Parameter name="Verbosity Level" type="string" value="medium" />
                  </ParameterList>
                </ParameterList>
              </ParameterList>
            </ParameterList>
          </ParameterList>
        </ParameterList>
      </ParameterList>
      <ParameterList name="Line Search">
        <ParameterList name="Full Step">
          <Parameter name="Full Step" type="double" value="1" />
        </ParameterList>
        <Parameter name="Method" type="string" value="Full Step" />
      </ParameterList>
      <Parameter name="Nonlinear Solver" type="string" value="Line Search Based" />
      <ParameterList name="Printing">
        <Parameter name="Output Information" type="int" value="103" />
        <Parameter name="Output Precision" type="int" value="3" />
        <Parameter name="Output Processor" type="int" value="0" />
      </ParameterList>
      <ParameterList name="Solver Options">
        <Parameter name="Status Test Check Type" type="string" value="Minimal" />
      </ParameterList>
    </ParameterList>
  </ParameterList>
</ParameterList>
