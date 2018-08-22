SOLUTION
  case '2'
  topology_optimization
  solver gdsw
END
INVERSE-PROBLEM
  data_truth_table truth table filename
  real_data_file real_data_filename
  imaginary_data_file imag_data_filename
END
OPTIMIZATION
  optimization_package ROL_lib
  ROLmethod linesearch
  LSstep Newton-Krylov
  LS_curvature_condition null
  Max_iter_Krylov 50
  Use_FD_hessvec false
  Use_inexact_hessvec false
END
DAMPING
  alpha 1e-3
  beta 5e-5
END
FREQUENCY
  freq_min 1
  freq_max 1
  freq_step 1
  disp
  block 1
END
FUNCTION 1
  type linear
  data 0 1
  data 1e6 1
END
GDSW
  diag_scaling diagonal
  solver_tol = 1e-11
END
OUTPUTS
  topology
END
ECHO
  topology
END
MATERIAL 1
  isotropic
  E = 1e9
  nu = .3
  material_penalty_model = simp
  penalty_coefficient = 3
END
BLOCK 1
  hex8u
  material 1
  inverse_material_type homogeneous
END
TOPOLOGY-OPTIMIZATION
  algorithm = plato_engine
  case = inverse_methods
  inverse_method_objective = directfrf-plato-density-method
  volume_fraction = .5
END
FILE
  geometry_file 'plato_test1.gen'
END
LOADS
  nodeset 1 force 0 0 1 scale 100
END
BOUNDARY
  nodeset 2 fixed
END
