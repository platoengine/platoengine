begin service 1
  code platomain
  number processors 1
  number ranks 1
end service

begin service 2
  code plato_analyze
  number processors 1
  number ranks 1
end service

begin criterion 1
  type heat_conduction
  minimum_ersatz_material_value 1e-3
end criterion

begin criterion 2
  type volume
end criterion

begin scenario 1
  physics thermal
  dimensions 3
  loads 1
  boundary_conditions 1 2
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

begin boundary conditions
   fixed temperature nodeset name ns_1 bc id 1
   fixed temperature nodeset name ns_2 bc id 2
end boundary conditions

begin loads
    heat flux sideset name ss_1 value -1e2 load id 1
end loads
      
begin constraint
  criterion 2
  relative_target .2
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
   material_model isotropic linear elastic 
   thermal_conductivity 210
   mass_density 2703
   specific_heat 900 
end material

begin output
    service 2
   output_data true
   data temperature
end output

begin optimization parameters
   filter radius scale 2.48
   max iterations 30 
//   output frequency 1000 
   algorithm mma
   discretization density 
   initial density value .2
   fixed blocks 2
   al penalty parameter 1
   al penalty scale factor 1.05
end optimization parameters

begin mesh
   name tm2.exo
end mesh


