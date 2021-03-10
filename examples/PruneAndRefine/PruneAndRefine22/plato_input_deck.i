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
  loads 1
  boundary_conditions 1
  material 1
  minimum_ersatz_material_value 1e-3
  tolerance 5e-8
end scenario

begin output
    service 2
    output_data_to_file true
end output

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
    degree_of_freedom dispx dispz dispy
    value 0 0 0
end boundary_condition

begin load 1
    type traction
    location_type sideset
    location_name ss_1
    value 0 -3e4 0
end load

begin constraint 1
  criterion 2
  absolute_target 20
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
   filter_radius_absolute .6
   max_iterations 5
   output_frequency 5
   optimization_algorithm oc
   discretization density
   initial_density_value .5

   // The default field name for restarting with
   // an initial guess is "optimizationdofs". However,
   // Plato Analyze uses the word "control" for its field
   // name so we have to specify it explicitly here.
   initial_guess_field_name control

   // When running from the GUI the name of the mesh file
   // containing the initial guess for the densities
   // is automatically generated to be "restart_<iteration#.exo".
   // Since we aren't running from the GUI we will explicitly
   // specify the filename.
   initial_guess_file_name restart_data.exo

   // Number of extra mesh element layers to keep
   // around the actual initial guess for the design.
   // This important so that the design can still 
   // adjust without hitting the boundaries of the 
   // pruned mesh.
   number_buffer_layers 2

   // Specifies whether to prune the mesh or not.
   prune_mesh true

   // Specifies the number of global refinements to do.
   number_refines 1

   // Non-zero value indicates we are restarting from
   // an initial guess. This is typically for when you
   // are running from the GUI but at the moment
   // probably needs to be in here anyway. If you do 
   // not specify an "initial guess filename" this
   // iteration number will be used to automatically build
   // a file name based on the iteration you want to 
   // restart from.
   restart_iteration 100

   normalize_in_aggregator false

end optimization_parameters

begin mesh
   name pa_comp_min1.exo
end mesh
   
begin paths
code PlatoMain /ascldap/users/bwclark/spack2/platoengine/RELEASE/apps/services/PlatoMain
code plato_analyze analyze_MPMD
end paths
