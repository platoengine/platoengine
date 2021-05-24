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
  type mechanical_compliance
  minimum_ersatz_material_value 1e-3
end criterion

begin criterion 2
  type volume
end criterion

begin scenario 1
  physics steady_state_mechanics
  dimensions 3
  loads 1 2 3 4
  boundary_conditions 1
  material 1
  minimum_ersatz_material_value 1e-3
  tolerance 5e-8
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
   data dispx dispy dispz vonmises
end output

begin boundary_condition 1
    type fixed_value
    location_type sideset
    location_name sideset1
    degree_of_freedom dispx dispz dispy
    value 0 0 0
end boundary_condition

begin load 1
    type traction
    location_type sideset
    location_name sideset2
    value 0 0 -5000
end load

begin load 2
    type traction
    location_type sideset
    location_name sideset3
    value 0 5000 0
end load

begin load 3
    type traction
    location_type sideset
    location_name sideset4
    value 0 0 5000
end load
begin load 4
    type traction
    location_type sideset
    location_name sideset5
    value 0 -5000 0
end load
      
begin constraint 1
  criterion 2
  relative_target 0.08
  type less_than
  service 1
end constraint

begin block 1
   material 1
end block

begin block 2
   material 1
end block

begin material 1
   material_model isotropic_linear_elastic 
   poissons_ratio .3
   youngs_modulus 1e8
end material

begin optimization_parameters
   verbose true
   filter_radius_scale 1.5
   max_iterations 5 
   output_frequency 1000 
   optimization_algorithm rol_linear_constraint
   discretization density 
   initial_density_value .08
   normalize_in_aggregator false
   fixed_block_ids 1
end optimization_parameters

begin mesh
   name mesh.exo
end mesh

