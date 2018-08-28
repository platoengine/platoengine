/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
*/

/*
 * Plato_Test_Geometry_MLS.cpp
 *
 *  Created on: Feb 27, 2018
 */

#include <gtest/gtest.h>
#include "Plato_InputData.hpp"
#include "Plato_Parser.hpp"

namespace PlatoTestInputData
{


  TEST(PlatoTestInputData, UniqueStoreRecall)
  {
    Plato::InputData inputData("Input Data");

    // store and recall some stuff

    // double
    //
    double A_in = 1.0;
    inputData.set("A", A_in);
    auto A_out = inputData.get<decltype(A_in)>("A");
    EXPECT_EQ( A_in, A_out );
  
    // long int
    //
    long int B_in = 1234;
    inputData.set("B", B_in);
    auto B_out = inputData.get<decltype(B_in)>("B");
    EXPECT_EQ( B_in, B_out );

    // string
    //
    std::string C_in = "Cval";
    inputData.set("C", C_in);
    auto C_out = inputData.get<decltype(C_in)>("C");
    EXPECT_EQ( C_in, C_out );
  }

  TEST(PlatoTestInputData, NonUniqueStoreRecall)
  {
    Plato::InputData inputData("Input Data");

    // store and recall some stuff

    // double
    //
    double A_in = 1.0;
    inputData.add("A", A_in);
    auto A_out = inputData.get<decltype(A_in)>("A", 0);
    EXPECT_EQ( A_in, A_out );
    
    // another double, same name
    //
    A_in = 2.0;
    inputData.add("A", A_in);
    A_out = inputData.get<decltype(A_in)>("A", 1);
    EXPECT_EQ( A_in, A_out );

    // different type
    //
    int intA_in = 10;
    inputData.add("Int A", intA_in);
    auto intA_out = inputData.get<decltype(intA_in)>("Int A");
    EXPECT_EQ( intA_in, intA_out );
    
  }

  TEST(PlatoTestInputData, NonUnique_size)
  {
    Plato::InputData inputData("Input Data");

    // add some doubles
    //
    inputData.add<double>("A", 1.0);
    inputData.add<double>("A", 2.0);

    auto size = inputData.size<double>("A");
    EXPECT_EQ(size, (unsigned int)2);

    // add some ints
    //
    inputData.add<int>("Int A", 10);
    inputData.add<int>("Int A", 20);

    size = inputData.size<int>("Int A");
    EXPECT_EQ(size, (unsigned int)2);
  }



  TEST(PlatoTestInputData, NonUnique_getByName)
  {
    Plato::InputData inputData("Input Data");

    // add some doubles
    //
    inputData.add<double>("A", 1.0);
    inputData.add<double>("A", 2.0);

    // add some ints
    //
    inputData.add<int>("Int A", 10);
    inputData.add<int>("Int A", 20);

    auto realVals = inputData.getByName<double>("A");
    EXPECT_EQ(realVals[0], 1.0);
    EXPECT_EQ(realVals[1], 2.0);

    auto intVals = inputData.getByName<int>("Int A");
    EXPECT_EQ(intVals[0], 10);
    EXPECT_EQ(intVals[1], 20);
  }


  TEST(PlatoTestInputData, NonUniqueSubData)
  {

    Plato::InputData inputData("Input Data");

    // add an InputData
    //
    Plato::InputData subData1("SharedData");
    double sd0_A_in = 1.23;
    subData1.set<decltype(sd0_A_in)>("A", sd0_A_in);
    inputData.add<Plato::InputData>("SD", subData1);
  
    // add another InputData
    //
    Plato::InputData subData2("SharedData");
    double sd1_A_in = 4.56;
    subData2.set<decltype(sd1_A_in)>("A", sd1_A_in);
    inputData.add<Plato::InputData>("SD", subData2);
  
    // get values from the sub InputData
    //
    auto sd0 = inputData.get<Plato::InputData>("SD",0);
    EXPECT_EQ(sd0.get<decltype(sd0_A_in)>("A"), sd0_A_in);

    // get values from the sub InputData
    //
    auto sd1 = inputData.get<Plato::InputData>("SD",1);
    EXPECT_EQ(sd1.get<decltype(sd1_A_in)>("A"), sd1_A_in);

    // get the sub InputData by name, set a value, then get the value
    //
    auto sds = inputData.getByName<Plato::InputData>("SD");
    sd0_A_in += 1.0;
    sds[0].set<decltype(sd0_A_in)>("A", sd0_A_in);
    EXPECT_EQ(inputData.get<Plato::InputData>("SD").get<decltype(sd0_A_in)>("A"), sd0_A_in);

  }

  TEST(PlatoTestInputData, NonUniqueSharedData)
  {
    Plato::InputData inputData("Input Data");

    {
      Plato::InputData sd("SharedData");
      sd.set<std::string>("Name","Objective Gradient 1");
      sd.set<std::string>("Type","Scalar");
      sd.set<std::string>("Layout","Nodal Field");
      sd.set<std::string>("OwnerName","Alexa 1");
      sd.set<std::string>("UserName","PlatoMain");
      inputData.add("SharedData",sd);
    }
    {
      Plato::InputData sd("SharedData");
      sd.set<std::string>("Name","Objective Gradient 2");
      sd.set<std::string>("Type","Scalar");
      sd.set<std::string>("Layout","Nodal Field");
      sd.set<std::string>("OwnerName","Alexa 2");
      sd.set<std::string>("UserName","PlatoMain");
      inputData.add("SharedData",sd);
    }
    {
      Plato::InputData sd("SharedData");
      sd.set<std::string>("Name","Topology");
      sd.set<std::string>("Type","Scalar");
      sd.set<std::string>("Layout","Nodal Field");
      sd.set<std::string>("OwnerName","PlatoMain");
      sd.add<std::string>("UserName","PlatoMain");
      sd.add<std::string>("UserName","Alexa 1");
      sd.add<std::string>("UserName","Alexa 2");
      inputData.add("SharedData",sd);
    }
  
    auto sd = inputData.getByName<Plato::InputData>("SharedData");
    EXPECT_EQ(sd[2].get<std::string>("UserName"), "PlatoMain");

  }

  TEST(PlatoTestInputData, ParseSharedData)
  {
    std::stringstream buffer;
    buffer << "<SharedData>"                       << std::endl;
    buffer << "  <Name>Topology</Name>"            << std::endl;
    buffer << "  <Type>Scalar</Type>"              << std::endl;
    buffer << "  <Layout>Nodal Field</Layout>"     << std::endl;
    buffer << "  <OwnerName>PlatoMain</OwnerName>" << std::endl;
    buffer << "  <UserName>PlatoMain</UserName>"   << std::endl;
    buffer << "  <UserName>Alexa 1</UserName>"     << std::endl;
    buffer << "  <UserName>Alexa 2</UserName>"     << std::endl;
    buffer << "</SharedData>"                      << std::endl;

    Plato::Parser* parser = new Plato::PugiParser();
    Plato::InputData inputData = parser->parseString(buffer.str());
    delete parser;

    auto sd = inputData.get<Plato::InputData>("SharedData");
    EXPECT_EQ( sd.get<std::string>("Name"),      "Topology"    );
    EXPECT_EQ( sd.get<std::string>("Type"),      "Scalar"      );
    EXPECT_EQ( sd.get<std::string>("Layout"),    "Nodal Field" );
    EXPECT_EQ( sd.get<std::string>("OwnerName"), "PlatoMain"   );

    auto numUsers = sd.size<std::string>("UserName");
    EXPECT_EQ( numUsers, (unsigned int)3);
    
    EXPECT_EQ( sd.get<std::string>("UserName",0), "PlatoMain" );
    EXPECT_EQ( sd.get<std::string>("UserName",1), "Alexa 1"   );
    EXPECT_EQ( sd.get<std::string>("UserName",2), "Alexa 2"   );

  }

  TEST(PlatoTestInputData, ParseNestedNested)
  {
    std::stringstream buffer;
    buffer << "<Stage>"                  << std::endl;
    buffer << " <Operation>"             << std::endl;
    buffer << "  <Input>"                << std::endl;
    buffer << "   <Name>Topology</Name>" << std::endl;
    buffer << "  </Input>"               << std::endl;
    buffer << " </operation>"            << std::endl;
    buffer << "</Stage>"                 << std::endl;

    Plato::Parser* parser = new Plato::PugiParser();
    Plato::InputData inputData = parser->parseString(buffer.str());
    delete parser;

    auto st = inputData.get<Plato::InputData>("Stage");
    auto op = st.get<Plato::InputData>("Operation");
    auto in = op.get<Plato::InputData>("Input");
    EXPECT_EQ( in.get<std::string>("Name"), "Topology" );
  }

  TEST(PlatoTestInputData, ParseStage)
  {
    std::stringstream buffer;
    buffer << "<Stage>" << std::endl;
    buffer << "  <Name>Set Lower Bounds</Name>" << std::endl;
    buffer << "  <Input>" << std::endl;
    buffer << "    <SharedDataName>Lower Bound Value</SharedDataName>" << std::endl;
    buffer << "  </Input>" << std::endl;
    buffer << "  <Operation>" << std::endl;
    buffer << "    <Name>Calculate Lower Bounds</Name>" << std::endl;
    buffer << "    <PerformerName>PlatoMain</PerformerName>" << std::endl;
    buffer << "    <Input>" << std::endl;
    buffer << "      <ArgumentName>Lower Bound Value</ArgumentName>" << std::endl;
    buffer << "      <SharedDataName>Lower Bound Value</SharedDataName>" << std::endl;
    buffer << "    </Input>" << std::endl;
    buffer << "    <Output>" << std::endl;
    buffer << "      <ArgumentName>Lower Bound Vector</ArgumentName>" << std::endl;
    buffer << "      <SharedDataName>Lower Bound Vector</SharedDataName>" << std::endl;
    buffer << "    </Output>" << std::endl;
    buffer << "  </Operation>" << std::endl;
    buffer << "  <Output>" << std::endl;
    buffer << "    <SharedDataName>Lower Bound Vector</SharedDataName>" << std::endl;
    buffer << "  </Output>" << std::endl;
    buffer << "</Stage>" << std::endl;

    Plato::Parser* parser = new Plato::PugiParser();
    Plato::InputData inputData = parser->parseString(buffer.str());
    delete parser;

    auto st = inputData.get<Plato::InputData>("Stage");
    auto in = st.get<Plato::InputData>("Input");
    EXPECT_EQ( in.get<std::string>("SharedDataName"), "Lower Bound Value" );

    EXPECT_EQ( inputData.get<Plato::InputData>("Stage")
                        .get<Plato::InputData>("Input")
                        .get<std::string>("SharedDataName"), 
                "Lower Bound Value");
  }


  TEST(PlatoTestInputData, ExpandVariable)
  {
    std::stringstream buffer;
    buffer << "<Variable name='Perfs' type='int' from='1' to='4'/>" << std::endl;

    buffer << "<For var='N' in='Perfs'>" << std::endl;
    buffer << "<Performer>" << std::endl;
    buffer << "  <Name>Analyze</Name>" << std::endl;
    buffer << "  <PerformerID>[N]</PerformerID>" << std::endl;
    buffer << "</Performer>" << std::endl;
    buffer << "</For>" << std::endl;

    Plato::Parser* parser = new Plato::PugiParser();
    Plato::InputData inputData = parser->parseString(buffer.str());
    delete parser;


    EXPECT_EQ( inputData.size<Plato::InputData>("Performer"), 4 );

    auto performerSpecs = inputData.getByName<Plato::InputData>("Performer");

    EXPECT_EQ( performerSpecs[0].get<std::string>("Name"), "Analyze" );
    EXPECT_EQ( performerSpecs[1].get<std::string>("Name"), "Analyze" );
    EXPECT_EQ( performerSpecs[2].get<std::string>("Name"), "Analyze" );
    EXPECT_EQ( performerSpecs[3].get<std::string>("Name"), "Analyze" );

    EXPECT_EQ( performerSpecs[0].get<std::string>("PerformerID"), "1" );
    EXPECT_EQ( performerSpecs[1].get<std::string>("PerformerID"), "2" );
    EXPECT_EQ( performerSpecs[2].get<std::string>("PerformerID"), "3" );
    EXPECT_EQ( performerSpecs[3].get<std::string>("PerformerID"), "4" );
  }


  TEST(PlatoTestInputData, ExpandChildVariable)
  {
    std::stringstream buffer;
    buffer << "<Variable name='Apps' type='int' from='1' to='4'/>" << std::endl;

    buffer << "<SharedData>"                              << std::endl;
    buffer << "  <Name>Topology</Name>"                   << std::endl;
    buffer << "  <Type>Scalar</Type>"                     << std::endl;
    buffer << "  <Layout>Nodal Field</Layout>"            << std::endl;
    buffer << "  <OwnerName>PlatoMain</OwnerName>"        << std::endl;
    buffer << "  <UserName>PlatoMain</UserName>"          << std::endl;
    buffer << "  <For var='N' in='Apps'>"                 << std::endl;
    buffer << "  <UserName>LightMP_static_[N]</UserName>" << std::endl;
    buffer << "  </For>"                                  << std::endl;
    buffer << "</SharedData>"                             << std::endl;

    Plato::Parser* parser = new Plato::PugiParser();
    Plato::InputData inputData = parser->parseString(buffer.str());
    delete parser;

    EXPECT_EQ( inputData.get<Plato::InputData>("SharedData").size<std::string>("UserName"), 5 );

    auto userNames = inputData.get<Plato::InputData>("SharedData").getByName<std::string>("UserName");

    EXPECT_EQ( userNames[0], "PlatoMain" );
    EXPECT_EQ( userNames[1], "LightMP_static_1" );
    EXPECT_EQ( userNames[2], "LightMP_static_2" );
  }



  TEST(PlatoTestInputData, ExpandNestedVariable)
  {
    std::stringstream buffer;
    buffer << "<Variable name='Perfs' type='int' from='1' to='4'/>" << std::endl;
    buffer << "<Variable name='Apps'  type='int' from='1' to='3'/>" << std::endl;

    buffer << "<For var='M' in='Apps'>"                               << std::endl;
    buffer << "<Operation>"                                           << std::endl;
    buffer << "  <For var='N' in='Perfs'>"                            << std::endl;
    buffer << "  <Operation>"                                         << std::endl;
    buffer << "    <PerformerName>Performer_[N]</PerformerName>"      << std::endl;
    buffer << "    <Name>Compute</Name>"                              << std::endl;
    buffer << "    <Input>"                                           << std::endl;
    buffer << "      <ArgumentName>Topology</ArgumentName>"           << std::endl;
    buffer << "      <SharedDataName>Topology</SharedDataName>"       << std::endl;
    buffer << "    </Input>"                                          << std::endl;
    buffer << "    <Output>"                                          << std::endl;
    buffer << "      <ArgumentName>Gradient</ArgumentName>"           << std::endl;
    buffer << "      <SharedDataName>Gradient [4*(M-1)+N]</SharedDataName>" << std::endl;
    buffer << "    </Output>"                                         << std::endl;
    buffer << "  </Operation>"                                        << std::endl;
    buffer << "  </For>"                                              << std::endl;
    buffer << "</Operation>"                                          << std::endl;
    buffer << "</For>"                                                << std::endl;

    Plato::Parser* parser = new Plato::PugiParser();
    Plato::InputData inputData = parser->parseString(buffer.str());
    delete parser;


    EXPECT_EQ( inputData.size<Plato::InputData>("Operation"), 3 );

    auto multiOpSpecs = inputData.getByName<Plato::InputData>("Operation");

    auto singleOpSpecs_0 = multiOpSpecs[0].getByName<Plato::InputData>("Operation");
    EXPECT_EQ( singleOpSpecs_0[0].get<std::string>("PerformerName"), "Performer_1" );
    EXPECT_EQ( singleOpSpecs_0[1].get<std::string>("PerformerName"), "Performer_2" );
    EXPECT_EQ( singleOpSpecs_0[2].get<std::string>("PerformerName"), "Performer_3" );
    EXPECT_EQ( singleOpSpecs_0[3].get<std::string>("PerformerName"), "Performer_4" );

    auto singleOpSpecs_1 = multiOpSpecs[1].getByName<Plato::InputData>("Operation");
    EXPECT_EQ( singleOpSpecs_1[0].get<std::string>("PerformerName"), "Performer_1" );
    EXPECT_EQ( singleOpSpecs_1[1].get<std::string>("PerformerName"), "Performer_2" );
    EXPECT_EQ( singleOpSpecs_1[2].get<std::string>("PerformerName"), "Performer_3" );
    EXPECT_EQ( singleOpSpecs_1[3].get<std::string>("PerformerName"), "Performer_4" );

    auto singleOpSpecs_2 = multiOpSpecs[2].getByName<Plato::InputData>("Operation");
    EXPECT_EQ( singleOpSpecs_2[0].get<std::string>("PerformerName"), "Performer_1" );
    EXPECT_EQ( singleOpSpecs_2[1].get<std::string>("PerformerName"), "Performer_2" );
    EXPECT_EQ( singleOpSpecs_2[2].get<std::string>("PerformerName"), "Performer_3" );
    EXPECT_EQ( singleOpSpecs_2[3].get<std::string>("PerformerName"), "Performer_4" );
  }

  TEST(PlatoTestInputData, RealValuedExpression)
  {
    std::stringstream buffer;
    buffer << "<Variable name='Perfs' type='int' from='0' to='4'/>" << std::endl;

    buffer << "<Operation>"                                           << std::endl;
    buffer << "  <For var='N' in='Perfs'>"                            << std::endl;
    buffer << "  <Operation>"                                         << std::endl;
    buffer << "    <PerformerName>Performer_[N]</PerformerName>"      << std::endl;
    buffer << "    <Name>Compute</Name>"                              << std::endl;
    buffer << "    <Parameter>"                                       << std::endl;
    buffer << "      <ArgumentName>Traction X</ArgumentName>"         << std::endl;
    buffer << "      <ArgumentValue>[cos((-10+N*5)*(2.0*3.14159)/360.0)]</ArgumentValue>" << std::endl;
    buffer << "    </Parameter>"                                      << std::endl;
    buffer << "    <Parameter>"                                       << std::endl;
    buffer << "      <ArgumentName>Traction Y</ArgumentName>"         << std::endl;
    buffer << "      <ArgumentValue>[sin((-10+N*5)*(2.0*3.14159)/360.0)]</ArgumentValue>" << std::endl;
    buffer << "    </Parameter>"                                      << std::endl;
    buffer << "    <Input>"                                           << std::endl;
    buffer << "      <ArgumentName>Topology</ArgumentName>"           << std::endl;
    buffer << "      <SharedDataName>Topology</SharedDataName>"       << std::endl;
    buffer << "    </Input>"                                          << std::endl;
    buffer << "    <Output>"                                          << std::endl;
    buffer << "      <ArgumentName>Gradient</ArgumentName>"           << std::endl;
    buffer << "      <SharedDataName>Gradient [N]</SharedDataName>"   << std::endl;
    buffer << "    </Output>"                                         << std::endl;
    buffer << "  </Operation>"                                        << std::endl;
    buffer << "  </For>"                                              << std::endl;
    buffer << "</Operation>"                                          << std::endl;

    Plato::Parser* parser = new Plato::PugiParser();
    Plato::InputData inputData = parser->parseString(buffer.str());
    delete parser;

    auto multiOpSpec = inputData.getByName<Plato::InputData>("Operation");

    auto singleOpSpecs = multiOpSpec[0].getByName<Plato::InputData>("Operation");
    auto parameters = singleOpSpecs[0].getByName<Plato::InputData>("Parameter");
    EXPECT_EQ( parameters[0].get<std::string>("ArgumentValue"), "0.984808" );
    EXPECT_EQ( parameters[1].get<std::string>("ArgumentValue"), "-0.173648" );
  }

} // end PlatoTestInputData namespace
