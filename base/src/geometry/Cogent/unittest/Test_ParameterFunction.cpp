#include <gtest/gtest.h>

#include "core/Cogent_Types.hpp"
#include "core/Cogent_ParameterFunction.hpp"

/******************************************************************************
*  Test: simpleAddition
*  Purpose:  Check for basic functionality of ParameterFunction class
*******************************************************************************/
TEST(ParameterFunctionTest, simpleAddition)
{
  // define and set independent values
  std::vector<std::string> vars{"x","y"};
  std::vector<RealType> vals{2.0,3.0};

  // define function
  std::string func = "x+y";
  Cogent::ParameterFunction myFunction(vars, func);
  myFunction.update(vals);

  // define expected result
  RealType expectedValue = 5.0;
  
  EXPECT_EQ( myFunction.getValue<double>(), expectedValue );
}


/******************************************************************************
*  Test: inactiveVariable
*  Purpose:  Verify that ParameterFunction class will correctly evaluate 
*  a function even if one independent variable isn't used in the expression.
*******************************************************************************/
TEST(ParameterFunctionTest, inactiveVariable)
{
  // define and set independent values
  std::vector<std::string> vars{"x","y","z"};
  std::vector<RealType> vals{2.0,3.0,4.0};

  // define function
  std::string func = "x+z";
  Cogent::ParameterFunction myFunction(vars, func);
  myFunction.update(vals);

  // define expected result
  RealType expectedValue = 6.0;
  
  EXPECT_EQ( myFunction.getValue<double>(), expectedValue );
}


/******************************************************************************
*  Test: basicDerivative
*  Purpose:  Verify that ParameterFunction class will correctly evaluate 
*  a derivative of a function.
*******************************************************************************/
TEST(ParameterFunctionTest, basicDerivative)
{
  // define and set independent values
  std::vector<std::string> vars{"x","y","z"};
  std::vector<RealType> vals{2.0,3.0,4.0};

  // define function
  std::string func = "x^2+y^(-1)+z^3";
  Cogent::ParameterFunction myFunction(vars, func, 1e-7);
  myFunction.update(vals);

  // define expected result
  std::vector<RealType> expectedValue{4.0, -1.0/9.0, 48.0};

  DFadType result = myFunction.getValue<DFadType>(); 
  
  EXPECT_NEAR( result.dx(0), expectedValue[0], 1e-5 );
  EXPECT_NEAR( result.dx(1), expectedValue[1], 1e-5 );
  EXPECT_NEAR( result.dx(2), expectedValue[2], 1e-5 );
}
