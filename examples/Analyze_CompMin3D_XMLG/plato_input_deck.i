begin objective
   type maximize stiffness 
   load ids 1 
   boundary condition ids 1  
   code plato_analyze
   number processors 1
//   output for plotting dispx dispy vonmises
end objective

begin boundary conditions
   fixed displacement nodeset ns_1 bc id 1
end boundary conditions

begin loads
    traction sideset ss_2 value 0 -3e3 0 load id 1
end loads
      
begin constraint 
   type volume
   volume fraction .25
end constraint

begin block 1
   material 1
end block
begin block 2
   material 1
end block

begin material 1
   poissons ratio .3
   youngs modulus 1e8
end material

begin optimization parameters
   number processors 1
   filter radius scale 4.48
   max iterations 10 
   output frequency 1000 
   algorithm oc
   discretization density 
   initial density value .5
end optimization parameters

begin mesh
   name bolted_bracket.exo
end mesh

