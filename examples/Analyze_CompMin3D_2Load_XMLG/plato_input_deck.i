begin service 1
  code platomain
  number_processors 1
end service

begin service 2
  code plato_analyze
  number_processors 1
end service

begin service 3
  code plato_analyze
  number_processors 1
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
  loads 1
  boundary_conditions 1 2 3
  material 1
  minimum_ersatz_material_value 1e-3
  tolerance 5e-8
end scenario

begin scenario 2
  physics steady_state_mechanics
  dimensions 3
  loads 2
  boundary_conditions 1 2 3
  material 1
  minimum_ersatz_material_value 1e-3
  tolerance 5e-8
end scenario

begin objective
  type weighted_sum
  criteria 1 1
  services 2 3
  scenarios 1 2
  weights 1 1 
end objective

//begin objective
//  type weighted_sum
//  criteria  1
//  services  2
//  scenarios  1
//  weights  1 
//end objective

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

begin boundary_condition 1
    type fixed_value
    location_type nodeset
    location_name ns_1
    degree_of_freedom dispx
    value 0 
end boundary_condition

begin boundary_condition 2
    type fixed_value
    location_type nodeset
    location_name ns_1
    degree_of_freedom dispy
    value 0 
end boundary_condition

begin boundary_condition 3
    type fixed_value
    location_type nodeset
    location_name ns_1
    degree_of_freedom dispz
    value 0 
end boundary_condition

begin load 2
    type traction
    location_type sideset
    location_name ss_2
    value 0 0 3e3
end load

begin load 1
    type traction
    location_type sideset
    location_name ss_2
    value 0 -3e3 0
end load
      
begin constraint
  criterion 2
  relative_target 0.5
  type less_than
  service 1
end constraint

begin block 1
   material 1
end block

begin material 1
   material_model isotropic_linear_elastic 
   poissons_ratio .3
   youngs_modulus 1e8
end material

begin optimization_parameters
   filter_radius_scale 2
   max_iterations 10 
   output_frequency 1000 
   optimization_algorithm oc
   discretization density 
   initial_density_value .5
end optimization_parameters

begin mesh
   name beam.exo
   //name bolted_bracket.exo
end mesh

begin paths
code PlatoMain /ascldap/users/bwclark/spack2/platoengine/RELEASE/apps/services/PlatoMain
code plato_analyze analyze_MPMD
end paths
