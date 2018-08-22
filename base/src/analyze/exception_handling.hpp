#ifndef EXCEPTION_HANDLING
#define EXCEPTION_HANDLING
#include <string>
#include <exception>
#include <vector>
using namespace std;

class ParsingException : public exception
{
  public:
    ParsingException(string d);
};

class IOException : public exception
{
  public:
    IOException(string file, string description);
};

class RunTimeError : public exception
{
  public:
    RunTimeError(string d);
};
#endif
