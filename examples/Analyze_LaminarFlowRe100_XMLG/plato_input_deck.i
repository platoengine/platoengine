begin service 1
  code platomain
  number_processors 1
end service

begin service 2
  code plato_analyze
  number_processors 1
end service

begin output
  service 2
  native_service_output false
end output

begin criterion 1
  type composite
  criterion_ids 2 3
  criterion_weights 0.01 -0.01
end criterion

begin criterion 2
  type surface_pressure
  surfaces inlet
end criterion

begin criterion 3
  type surface_pressure
  surfaces outlet
end criterion

begin criterion 4
  type volume
end criterion

begin scenario 1
  physics steady_state_incompressible_fluids
  dimensions 2
  boundary_conditions 1 2 3 4 5
  material 1
end scenario

begin objective
  scenarios 1
  criteria 1
  services 2
  type weighted_sum
  weights 1
end objective

begin constraint 1
  criterion 4
  relative_target 0.25
  type less_than
  service 1
  scenario 1
end constraint

begin boundary_condition 1
  type zero_value
  location_type nodeset
  location_name no_slip
  degree_of_freedom velx
end boundary_condition

begin boundary_condition 2
  type zero_value
  location_type nodeset
  location_name no_slip
  degree_of_freedom vely
end boundary_condition

begin boundary_condition 3
  type fixed_value
  location_type nodeset
  location_name inlet
  degree_of_freedom velx
  value 1.5
end boundary_condition

begin boundary_condition 4
  type fixed_value
  location_type nodeset
  location_name inlet
  degree_of_freedom vely
  value 0
end boundary_condition

begin boundary_condition 5
  type zero_value
  location_type nodeset
  location_name outlet
  degree_of_freedom press
end boundary_condition

begin block 1
  material 1
  name block_1
end block

begin material 1
  material_model incompressible_flow
  reynolds_number 100
end material

begin optimization_parameters
  optimization_algorithm mma
  discretization density
  max_iterations 5
  mma_move_limit 0.25
  filter_radius_scale 1.75
end optimization_parameters

begin mesh
  name pipe_flow.exo
end mesh

begin paths
  code PlatoMain PlatoMain
  code plato_analyze analyze_MPMD
end paths

