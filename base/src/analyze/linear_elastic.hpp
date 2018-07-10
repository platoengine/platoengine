#ifndef ISOTROPICELASTIC
#define ISOTROPICELASTIC

#include "material_container.hpp"
#include "Plato_Parser.hpp"

class IsotropicElastic : public MaterialModel
{
public:
  IsotropicElastic(pugi::xml_node& node);
  ~IsotropicElastic();

  enum Parameters {
    YOUNGS_MODULUS,
    POISSONS_RATIO,
    C11, C12, C13,
         C22, C23,
              C33,
    C44, C55, C66,
    NUMPARAM
  };

  int STRESS;
  int STRAIN_INCREMENT;
  int TOTAL_STRAIN;

  // State variables 
  enum StateVariables { NUM_STATE_VARS=0 };

 protected:

   Intrepid::FieldContainer<double>* C;
};

class IsotropicElastic2D : public IsotropicElastic
{
public:
  IsotropicElastic2D(pugi::xml_node& node);
  ~IsotropicElastic2D();
  bool SetUp(DataContainer*, Tensor& R);
  bool Initialize(int, DataContainer*);
  bool UpdateMaterialState(int dataIndex,
                           DataContainer* dc);
  bool Tangent(int dataIndex, DataContainer* dc,
                  Intrepid::FieldContainer<double>*& C){return false;}
};

class IsotropicElastic3D : public IsotropicElastic
{
public:
  IsotropicElastic3D(pugi::xml_node& node);
  ~IsotropicElastic3D();
  bool SetUp(DataContainer*, Tensor& R);
  bool Initialize(int, DataContainer*);
  bool UpdateMaterialState(int dataIndex,
                           DataContainer* dc);
  bool Tangent(int dataIndex, DataContainer* dc,
                  Intrepid::FieldContainer<double>*& C);
};
MaterialModel* NewIsotropicElastic3D(pugi::xml_node& node);
MaterialModel* NewIsotropicElastic2D(pugi::xml_node& node);


#endif

