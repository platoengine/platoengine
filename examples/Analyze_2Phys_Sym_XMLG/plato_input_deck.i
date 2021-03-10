// Services
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
begin service 3
  code plato_analyze
  number_processors 1
  number_ranks 1
end service

// Criteria
begin criterion 1
  type mechanical_compliance
  minimum_ersatz_material_value 1e-3
end criterion
begin criterion 2
  type volume
  minimum_ersatz_material_value 0.0
  material_penalty_exponent 1.0
end criterion
begin criterion 3
  type stress_p-norm
  minimum_ersatz_material_value 1e-3
end criterion

// Scenarios
begin scenario 1
  physics steady_state_mechanics
  dimensions 3
  loads 1
  boundary_conditions 1
  material 1
  minimum_ersatz_material_value 1e-3
  tolerance 5e-14
end scenario
begin scenario 2
  physics transient_mechanics
  dimensions 3
  loads 2
  boundary_conditions 1
  material 1
  minimum_ersatz_material_value 1e-3
  tolerance 5e-14
  newmark_beta .25
  newmark_gamma .5
  number_time_steps 5
  time_step 2e-6
  symmetry_plane_normal 1 0 0
  symmetry_plane_origin 0 0 0
  mesh_map_filter_radius .15
  filter_before_symmetry_enforcement false
end scenario

// Objective
begin objective
  type weighted_sum
  criteria 3 1
  services 2 3
  scenarios 2 1
  weights .25 .75
end objective

// Output
begin output
   service 2
   output_data true
   data dispx dispy dispz
end output
begin output
   service 3
   output_data true
   data dispx dispy dispz
end output

// BCs
begin boundary_condition 1
    type fixed_value
    location_type sideset
    location_name ss_1
    degree_of_freedom dispx dispz dispy
    value 0 0 0
end boundary_condition

// Loads
begin load 1
    type traction
    location_type sideset
    location_name ss_2
    value 0 -3e3 0
end load
begin load 2
    type traction
    location_type sideset
    location_name ss_2
    value 0 0 -1e4 
end load
      
// Constraint
begin constraint 1
  criterion 2
  relative_target 0.4
  type less_than
  service 3
  scenario 1 
end constraint

// Blocks
begin block 1
   material 1
end block

// Materials
begin material 1
   material_model isotropic_linear_elastic 
   poissons_ratio .3
   youngs_modulus 1e8
end material
//begin material 2
//   material_model isotropic_linear_elastic 
//   mass_density 2.7
//   poissons_ratio .36
//   youngs_modulus 68e10
//end material

begin optimization_parameters
   filter_radius_scale 4.48
   max_iterations 1 
   output_frequency 1000 
   optimization_algorithm mma
   discretization density 
   initial_density_value .4
   normalize_in_aggregator true
   symmetry_plane_normal 1 0 0
   symmetry_plane_origin 0 0 0
   mesh_map_filter_radius .15
   filter_before_symmetry_enforcement false
   filter_in_engine false
end optimization_parameters

begin mesh
   name design_vol.exo
end mesh

begin paths
code PlatoMain PlatoMain
code plato_analyze analyze_MPMD
end paths
begin paths
code PlatoMain /ascldap/users/bwclark/spack2/platoengine/RELEASE/apps/services/PlatoMain
code plato_analyze analyze_MPMD
end paths
begin paths
code PlatoMain /ascldap/users/bwclark/spack2/platoengine/RELEASE/apps/services/PlatoMain
code plato_analyze analyze_MPMD
end paths
