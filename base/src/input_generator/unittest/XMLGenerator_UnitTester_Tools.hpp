/*
 * XMLGenerator_UnitTester_Tools.hpp
 *
 *  Created on: May 28, 2020
 */

#pragma once

#include <vector>
#include <string>

#include <gtest/gtest.h>

#include "pugixml.hpp"

namespace PlatoTestXMLGenerator
{

/******************************************************************************//**
 * \fn print_children
 * \brief Print children associated with PUGI XML node.
 * \param [in] aParentNode pugi::xml_node
**********************************************************************************/
inline void print_children(const pugi::xml_node& aParentNode)
{
    for(auto& tChild : aParentNode.children())
    {
        std::cout << "name = " << tChild.name() << ", value = " << tChild.child_value() << "\n";
    }
}
// function print_children

/******************************************************************************//**
 * \fn test_children
 * \brief Test children associated with PUGI XML node.
 * \param [in] aKeys       children keys
 * \param [in] aValues     children values
 * \param [in] aParentNode pugi::xml_node
**********************************************************************************/
inline void test_children
(const std::vector<std::string>& aKeys,
 const std::vector<std::string>& aValues,
 const pugi::xml_node& aParentNode)
{
    ASSERT_TRUE(aKeys.size() == aValues.size());
    for(auto& tChild : aParentNode.children())
    {
        auto tItr = std::find(aKeys.begin(), aKeys.end(), tChild.name());
        if(tItr == aKeys.end())
        {
            std::cout << "tItr->c_str() = " << tItr->c_str() << ", tChild.name() = "
                << tChild.name() << "\n";
            ASSERT_TRUE(tItr != aKeys.end());
        }
        ASSERT_STREQ(tItr->c_str(), tChild.name());

        tItr = std::find(aValues.begin(), aValues.end(), tChild.child_value());
        if(tItr == aValues.end())
        {
            std::cout << "tItr->c_str() = " << tItr->c_str() << ", tChild.child_value() = "
                << tChild.child_value() << "\n";
            ASSERT_TRUE(tItr != aValues.end());
        }
        ASSERT_STREQ(tItr->c_str(), tChild.child_value());
    }
}
// function test_children

/******************************************************************************//**
 * \fn test_children
 * \brief Test attributes associated with PUGI XML node.
 * \param [in] aKeys       children keys
 * \param [in] aValues     children values
 * \param [in] aParentNode pugi::xml_node
**********************************************************************************/
inline void test_attributes
(const std::vector<std::string>& aKeys,
 const std::vector<std::string>& aValues,
 const pugi::xml_node& aParentNode)
{
    ASSERT_TRUE(aKeys.size() == aValues.size());
    for(auto& tAttribute : aParentNode.attributes())
    {
        auto tItr = std::find(aKeys.begin(), aKeys.end(), tAttribute.name());
        if(tItr == aKeys.end())
        {
            std::cout << "tItr->c_str() = " << tItr->c_str() << ", tAttribute.name() = "
                << tAttribute.name() << "\n";
            ASSERT_TRUE(tItr != aKeys.end());
        }
        ASSERT_STREQ(tItr->c_str(), tAttribute.name());

        tItr = std::find(aValues.begin(), aValues.end(), tAttribute.value());
        if(tItr == aValues.end())
        {
            std::cout << "tItr->c_str() = " << tItr->c_str() << ", tAttribute.value() = "
                << tAttribute.value() << "\n";
            ASSERT_TRUE(tItr != aValues.end());
        }
        ASSERT_STREQ(tItr->c_str(), tAttribute.value());
    }
}
// function test_attributes

}
// namespace PlatoTestXMLGenerator
