// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include <string>
#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{

class GlobalUtilities
{
public:
    GlobalUtilities();
    virtual ~GlobalUtilities();

    virtual void print(const std::string& message) = 0;
    virtual void fatal_error(const std::string& message) = 0;

    template<typename T>
    void print(const std::vector<T>& v, const bool& end_line)
    {
        const size_t length = v.size();
        std::string print_string = std::string("[") + std::to_string(length) + std::string("]:{");

        for(size_t i = 0u; i < length; i++)
        {
            if(i != 0u)
            {
                print_string += std::string(",");
            }
            print_string += std::to_string(v[i]);
        }
        print_string += std::string("}");

        if(end_line)
        {
            print_string += std::string("\n");
        }

        this->print(print_string);
    }

protected:

};

}

}
