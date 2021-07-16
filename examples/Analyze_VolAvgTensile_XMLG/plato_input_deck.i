begin service 1
  code platomain
  number_processors 1
  number_ranks 1
  update_problem true
end service

begin service 2
  code plato_analyze
  number_processors 1
  number_ranks 1
  update_problem false
end service

begin criterion 1
  type composite
  criterion_ids 3 4
  criterion_weights 3.0e4 1.0e6
end criterion

begin criterion 2
  type volume
end criterion

begin criterion 3
  type mechanical_compliance
  material_penalty_exponent 3.0
  minimum_ersatz_material_value 1e-6
end criterion

begin criterion 4
  type volume_average
  local_measure tensileenergydensity
  spatial_weighting_function 1.0
  material_penalty_exponent 3.0
  minimum_ersatz_material_value 1e-6
end criterion

begin scenario 1
  physics steady_state_mechanics
  dimensions 3
  loads 1
  boundary_conditions 1
  material 1
  material_penalty_exponent 3.0
  minimum_ersatz_material_value 1e-6
  tolerance 1e-12
end scenario

begin objective
  type weighted_sum
  criteria 1
  services 2
  scenarios 1
  weights 1
end objective

begin output
   service 2
   output_data true
   data dispx dispy dispz
   native_service_output true
end output

begin boundary_condition 1
    type fixed_value
    location_type nodeset
    location_name ns_1
    degree_of_freedom dispx dispz dispy
    value 0 0 0
end boundary_condition

begin load 1
    type traction
    location_type sideset
    location_name ss_1
    value 0 -6 0
end load
begin load 2
    type traction
    location_type sideset
    location_name ss_2
    value 0 -6 0
end load
      
begin constraint 1
  criterion 2
  relative_target 0.3
  type less_than
  service 1
  scenario 1
end constraint

begin block 1
   material 1
end block

begin material 1
   material_model isotropic_linear_elastic 
   poissons_ratio 0.3
   youngs_modulus 1e8
end material

begin optimization_parameters
   filter_type kernel_then_tanh
   filter_use_additive_continuation true
   filter_projection_start_iteration 20
   filter_projection_update_interval 5
   filter_heaviside_max 10.0
   filter_heaviside_min 1.0
   filter_heaviside_update 0.5
   filter_radius_scale 1.75
   filter_in_engine true

   optimization_type topology
   max_iterations 5
   output_frequency 1000 
   discretization density 
   initial_density_value 0.3
   normalize_in_aggregator false
   enforce_bounds false
   problem_update_frequency 1
   optimization_algorithm mma
   mma_move_limit 0.1
end optimization_parameters

begin mesh
   name bolted_bracket_coarse.exo
end mesh

begin paths
code PlatoMain PlatoMain
code plato_analyze analyze_MPMD
end paths
begin paths
code PlatoMain /home/jbruss/platoengine/build/apps/services/PlatoMain
code plato_analyze analyze_MPMD
end paths
