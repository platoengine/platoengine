begin service 1
    code platomain
    number_processors 1
end service

begin service 2
    code plato_analyze
    number_processors 3
//    device_ids 0 1
end service

begin criterion 1
    type mechanical_compliance
    minimum_ersatz_material_value 1e-9
end criterion

begin criterion 2
  type volume
end criterion

begin scenario 1
  physics steady_state_mechanics
  dimensions 3
  loads 10
  boundary_conditions 1
  material 1
  minimum_ersatz_material_value 1e-9
  tolerance 1e-8
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
    statistics vonmises
end output

begin boundary_condition 1
    type fixed_value
    location_type nodeset
    location_name fixed
    degree_of_freedom dispx dispy dispz
    value 0 0 0
end boundary_condition

begin load 10
   type traction
   location_type sideset
   location_name load
   value 0 -5e2 0
   //value 0 -5e6 0
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
   poissons_ratio .33
   youngs_modulus 1e9
end material

begin uncertainty
    category load
    tag angle_variation
    attribute X
    load_id 10
    distribution beta
    mean 0.0
    upper_bound 45.0
    lower_bound -45.0
    standard_deviation 12.5
    number_samples 3
end uncertainty

begin optimization_parameters
   filter_radius_scale 2
   max_iterations 3
   output_frequency 1000
   optimization_algorithm oc
   discretization density
   initial_density_value .5
   normalize_in_aggregator false
//   fixed_block_ids 1
   objective_number_standard_deviations 2
end optimization_parameters

begin mesh
   name beam.exo
end mesh

begin code
   code PlatoMain
   code analyze_MPMD
end code
begin paths
code PlatoMain /ascldap/users/bwclark/spack2/platoengine/RELEASE/apps/services/PlatoMain
code plato_analyze analyze_MPMD
end paths
