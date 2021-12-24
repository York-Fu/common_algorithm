#include <iostream>
#include <unistd.h>
#include "lpf.h"

int main(int argv, char **argc)
{
  double value = 0;
  LowPassFilter2p lpf2p(1000, 100);

  for (size_t i = 0; i < 20; i++)
  {
    value = 1;
    value = lpf2p.apply(value);
    std::cout << "value(" << i + 1 << "): " << value << "\n";
  }

  return 0;
}
