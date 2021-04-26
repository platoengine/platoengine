begin service 1
  code platomain
  number_processors 4
  number_ranks 1
end service

begin service 2
  code plato_analyze
  number_processors 1
  number_ranks 1
end service

begin criterion 1
  type thermomechanical_compliance
  minimum_ersatz_material_value 1e-9
end criterion

begin criterion 2
  type volume
end criterion

begin scenario 1
  physics steady_state_thermomechanics
  dimensions 3
  loads 1 2
  boundary_conditions 1 2 3 4 5 6 7 8
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

begin boundary_condition 1
   type fixed_value
   location_type nodeset
   location_name ns_1
   degree_of_freedom dispy
   value 0.0
end boundary_condition

begin boundary_condition 2
   type fixed_value
   location_type nodeset
   location_name ns_1
   degree_of_freedom dispz
   value 0.0
end boundary_condition

begin boundary_condition 3
   type fixed_value
   location_type nodeset
   location_name ns_1
   degree_of_freedom temp
   value 0.0
end boundary_condition

begin boundary_condition 4
   type fixed_value
   location_type nodeset
   location_name ns_11
   degree_of_freedom dispx
   value 0.0
end boundary_condition

begin boundary_condition 5
   type fixed_value
   location_type nodeset
   location_name ns_2
   degree_of_freedom dispy
   value 0.0
end boundary_condition

begin boundary_condition 6
   type fixed_value
   location_type nodeset
   location_name ns_2
   degree_of_freedom dispz
   value 0.0
end boundary_condition

begin boundary_condition 7
   type fixed_value
   location_type nodeset
   location_name ns_2
   degree_of_freedom temp
   value 0.0
end boundary_condition

begin boundary_condition 8
   type fixed_value
   location_type nodeset
   location_name ns_21
   degree_of_freedom dispx
   value 0.0
end boundary_condition

begin load 2
    type uniform_surface_flux
    location_type sideset
    location_name ss_1
    value -0.0e2
end load

begin load 1
    type traction
    location_type sideset
    location_name ss_1
    value 0 1e5 0
end load

begin constraint 1
  criterion 2
  relative_target 0.2
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
   material_model isotropic_linear_thermoelastic
   youngs_modulus 1e11
   thermal_expansivity 1e-5
   thermal_conductivity 910
   reference_temperature 1e-2
   poissons_ratio .3
end material

begin output
   service 2
   output_data false
//   data dispx dispy dispz temperature
end output

begin optimization_parameters
   filter_radius_scale 2.48
   max_iterations 30 
   output_frequency 1000 
   optimization_algorithm ksal
   discretization density 
   initial_density_value .2
   al_penalty_parameter 1
   al_penalty_scale_factor 1.05
   fixed_block_ids 2
end optimization_parameters

begin mesh
   name tm2.exo
end mesh


begin paths
code PlatoMain /ascldap/users/bwclark/spack2/platoengine/RELEASE/apps/services/PlatoMain
code plato_analyze analyze_MPMD
end paths
