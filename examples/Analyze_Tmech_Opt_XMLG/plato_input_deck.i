begin objective
   type minimize thermoelastic energy
   load ids 1 2
   boundary condition ids 1 2 3 4 5 6 7 8 
   code plato_analyze
   number processors 1
end objective

begin boundary conditions
   fixed displacement nodeset name ns_1 y bc id 1
   fixed displacement nodeset name ns_1 z bc id 2
   fixed temperature nodeset name ns_1 bc id 3
   fixed displacement nodeset name ns_11 x bc id 4
   fixed displacement nodeset name ns_2 y bc id 5
   fixed displacement nodeset name ns_2 z bc id 6
   fixed temperature nodeset name ns_2 bc id 7
   fixed displacement nodeset name ns_21 x bc id 8
end boundary conditions

begin loads
    traction sideset name ss_1 value 0 1e5 0 load id 1
    heat flux sideset name ss_1 value -0.0e2 load id 2
end loads
      
begin constraint 
   type volume
   volume fraction .2
end constraint

begin block 1
   material 1
end block
begin block 2
   material 1
end block

begin material 1
   poissons ratio .3
   youngs modulus 1e11
   thermal expansion coefficient 1e-5
   thermal conductivity coefficient 910
   reference temperature 1e-2
end material

begin optimization parameters
   number processors 4
   filter radius scale 2.48
   max iterations 30 
   output frequency 1000 
   algorithm ksal
   discretization density 
   initial density value .2
   al penalty parameter 1
   al penalty scale factor 1.05
   fixed blocks 2
end optimization parameters

begin mesh
   name tm2.exo
end mesh


