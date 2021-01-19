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
  minimum_ersatz_material_value 1e-9
end criterion

begin criterion 2
  type volume
  minimum_ersatz_material_value 0
  material_penalty_exponent 1
end criterion

begin scenario 1
  physics steady_state_mechanics
  dimensions 2
  loads 1
  boundary_conditions 1
  material 1
  minimum_ersatz_material_value 1e-9
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
   data dispx dispy dispz
end output

begin boundary_condition 1
    type fixed_value
    location_type nodeset
    location_name fix
    degree_of_freedom dispx dispy 
    value 0 0 
end boundary_condition

begin load 1
    type traction
    location_type sideset
    location_name load
    value 0 -1e5 
end load
      
begin constraint 1
  criterion 2
  relative_target 0.4
  type less_than
  service 1
  scenario 1
end constraint

begin block 1
   material 1
end block

begin material 1
   material_model isotropic_linear_elastic 
   poissons_ratio .3
   youngs_modulus 200e9
end material

begin optimization_parameters
   filter_radius_scale 6
   max_iterations 20 
   output_frequency 1000 
   optimization_algorithm oc
   discretization density 
   initial_density_value .5
   normalize_in_aggregator false
end optimization_parameters

begin mesh
   name lbracket.exo
end mesh

begin paths
code PlatoMain PlatoMain
code plato_analyze analyze_MPMD
end paths
begin paths
code PlatoMain /ascldap/users/bwclark/spack2/platoengine/RELEASE/apps/services/PlatoMain
code plato_analyze analyze_MPMD
end paths
