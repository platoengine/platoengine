begin service 1
  code platomain
  number_processors 1
  number_ranks 1
end service

begin service 2
  code plato_analyze
  number_processors 1
  number_ranks 1
end service

begin criterion 1
  type total_work
  material_penalty_exponent 2.0
  minimum_ersatz_material_value 1e-6
end criterion

begin criterion 2
  type volume
end criterion

begin scenario 1
  physics thermoplasticity
  dimensions 3
  loads 1 2
  boundary_conditions 1 2 3 4
  material 1
  // Penalty function in Elliptic...
  material_penalty_exponent 2.0
  minimum_ersatz_material_value 1e-6
  // Linear Solver
  tolerance 1e-12
  // Scaling
  pressure_scaling 200.0
  temperature_scaling 100.0
  // Time Step Control
  number_time_steps 8
  max_number_time_steps 8
end scenario

begin objective
  type weighted_sum
  criteria 1
  services 2
  scenarios 1
  weights 1e2
end objective

begin boundary_condition 1
   type fixed_value
   location_type sideset
   location_name fixed_xy
   degree_of_freedom dispx
   value 0.0
end boundary_condition

begin boundary_condition 2
   type fixed_value
   location_type sideset
   location_name fixed_xy
   degree_of_freedom dispy
   value 0.0
end boundary_condition

begin boundary_condition 3
   type fixed_value
   location_type sideset
   location_name fixed_z
   degree_of_freedom dispz
   value 0.0
end boundary_condition

begin boundary_condition 4
   type time_function
   location_type sideset
   location_name fixed_xy
   degree_of_freedom temp
   value 100*t+100
end boundary_condition

begin load 1
    type traction
    location_type sideset
    location_name traction
    value 0 -1*t 0
end load

begin load 2
    type uniform_surface_flux
    location_type sideset
    location_name flux
    value 50*t
end load

begin constraint 1
  criterion 2
  relative_target 0.5
  type less_than
  service 1
end constraint

begin block 1
   material 1
end block

begin material 1
   material_model thermoplasticity
   youngs_modulus 75.0e3
   poissons_ratio .3
   thermal_expansivity 2.32e-5
   thermal_conductivity 16.0
   reference_temperature 100.0
   initial_yield_stress 344.0
   hardening_modulus_isotropic 1000.0
   hardening_modulus_kinematic 1000.0
   elastic_properties_penalty_exponent 2.0
   elastic_properties_minimum_ersatz   1.0e-6
   plastic_properties_penalty_exponent 1.5
   plastic_properties_minimum_ersatz   1.0e-3
end material

begin output
   service 2
   output_data false
end output

begin optimization_parameters
   filter_radius_scale 2.48
   max_iterations 2 
   output_frequency 5
   optimization_algorithm mma
   discretization density 
   initial_density_value .5
   mma_move_limit 0.1
   mma_asymptote_expansion 1.2
   mma_asymptote_contraction 0.7
   mma_max_sub_problem_iterations 30
end optimization_parameters

begin mesh
   name pin.exo
end mesh

begin paths
   code PlatoMain PlatoMain
   code plato_analyze analyze_MPMD
end paths
