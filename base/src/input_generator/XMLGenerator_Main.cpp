#include "XMLGenerator.hpp"
#include <cstring>

void print_usage()
{
    std::cout << "\n\nUsage: XMLGenerator <plato_input_deck_filename>\n\n";
}

/******************************************************************************/
int main(int argc, char *argv[])
/******************************************************************************/
{
    if(argc == 1 ||
      (argc > 1 && (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--h"))))
    {
        print_usage();
    }
    else
    {
        XMLGenerator generator(argv[1]);
        generator.generate();
    }
}



