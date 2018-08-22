SOLUTION
  case '1'
  topology_optimization
  solver gdsw
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
END
TOPOLOGY-OPTIMIZATION
  algorithm = plato_engine
  case = compliance_min
  volume_fraction = .5
END
FILE
  geometry_file 'plato_test1.gen'
END
LOADS
  sideset 1 traction 0 1 0 scale 100
END
BOUNDARY
  nodeset 2 fixed
END
