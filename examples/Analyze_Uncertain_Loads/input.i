begin objective
   type maximize stiffness
   load ids 10
   boundary condition ids 11
   code plato_analyze
   number processors 1
   weight 1 
   analyze new workflow true
   number ranks 2
end objective

begin boundary conditions
   fixed displacement nodeset name 1 bc id 11
end boundary conditions

begin loads
    traction sideset name 2 value 0 -5e4 0 load id 10
end loads

begin uncertainty
    type angle variation
    load 10
    axis X
    distribution beta
    mean 0.0
    upper bound 45.0
    lower bound -45.0
    standard deviation 22.5
    num samples 2
end uncertainty

begin constraint 
   type volume
   volume fraction .5
end constraint

begin block 1
   material 1
end block

begin material 1
   poissons ratio .33
   youngs modulus 1e9
end material

begin optimization parameters
   number processors 1
   filter radius scale  2
   max iterations 10 
   output frequency 0 
   algorithm oc
   discretization density 
   initial density value .5
   objective number standard deviations 1
end optimization parameters

begin mesh
   name mesh_file.exo
end mesh
