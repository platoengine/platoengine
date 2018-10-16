#ifndef _COGENT_UTILITIES_H
#define _COGENT_UTILITIES_H

#include <string>

namespace Cogent {

  std::string strint(std::string s, int i, char delim = ' ');

  template<class T>
  constexpr T pow(const T base, const unsigned int p)
  {
    return (p == 0) ? (T)1 : (base * pow(base, p-1));
  }
  
  template<class T>
  constexpr T mono(const T x, const T y, const T z, 
                   const unsigned int px, 
                   const unsigned int py, 
                   const unsigned int pz)
  {
    return pow(x,px)*pow(y,py)*pow(z,pz);
  }
}


#endif
