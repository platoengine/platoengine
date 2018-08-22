#include "exception_handling.hpp"
#include "communicator.hpp"

ParsingException::ParsingException(string description)
{
  pXcout << "Input error " << endl;
  pXcout << description << endl;
}

IOException::IOException(string file, string description)
{
  pXcout << "!!! Problem reading exodus file " << file << endl;
  pXcout << description << endl;
}

RunTimeError::RunTimeError(string d)
{
  pXcout << "Run time error :" << endl;
  pXcout << d << endl;
}

