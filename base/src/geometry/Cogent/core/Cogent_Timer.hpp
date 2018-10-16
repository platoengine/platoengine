#ifndef _COGENT_TIMER_H
#define _COGENT_TIMER_H

#include <boost/timer/timer.hpp>

namespace Cogent {

class Timer : public boost::timer::auto_cpu_timer {
  public:
    Timer(std::string fname) : auto_cpu_timer(6, "%w seconds\n"), fname(fname) {indent += 2;}
    ~Timer() {for(int i=0; i<indent; ++i) std::cout << " ";
              std::cout << fname << ": ";
              indent -= 2;
              if(indent==0) std::cout << std::endl;}
  private:
    std::string fname;
    static int indent;
};

} /** end namespace Cogent */

#endif
