#include <iostream>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <unistd.h>

// this package
#include <curie_demos/process_mem_usage.h>

const std::size_t ARRAY_SIZE = 12;

class StateType
{
public:
  StateType()
  {
  }
  double *values;
};

int main(int argc, char **argv)
{
  double vm, rss;

  double array_size_b = ARRAY_SIZE * sizeof(double);  // bytes
  double state_size_b = sizeof(StateType);            // bytes

  std::cout << "Size of state: " << state_size_b << " bytes" << std::endl;
  std::cout << "Size of double: " << sizeof(double) << " bytes" << std::endl;
  std::cout << "Size of int: " << sizeof(int) << " bytes" << std::endl;
  std::cout << "Size of short int: " << sizeof(short int) << " bytes" << std::endl;
  std::cout << "Size of short: " << sizeof(short) << " bytes" << std::endl;
  std::cout << "Size of 12 doubles: " << array_size_b << " bytes" << std::endl;
  std::cout << "Total size: " << array_size_b + state_size_b << " bytes" << std::endl;
  return 0;

  // Get computer's base measure of memory usage
  processMemUsage(vm, rss);

  // On first loop, add base memory
  double my_count_mb = vm;

  std::vector<StateType *> states;

  // Load memory
  while (vm < 1000.0)
  {
    states.push_back(new StateType());
    states.back()->values = new double[ARRAY_SIZE];

    // Populate with dummy data
    for (std::size_t i = 0; i < ARRAY_SIZE; ++i)
    {
      states.back()->values[i] = i;
    }

    // My count
    my_count_mb += (array_size_b + state_size_b) / 1048576.0;  // convert byte to megabyte

    // Get computer's measure of memory usage
    processMemUsage(vm, rss);

    // Output
    // std::cout << "VM: " << vm << " MB    |    my_count: " << my_count_mb << " MB" << std::endl;
    std::cout << "VM: " << vm << " MB \t RSS: " << rss << " MB" << std::endl;
  }

  // Unload array
  for (std::size_t i = 0; i < states.size(); ++i)
  {
    // Unload memory
    delete[] states[i]->values;

    // My count
    my_count_mb -= (array_size_b) / 1048576.0;  // convert byte to megabyte

    // Get computer's measure of memory usage
    processMemUsage(vm, rss);

    // Output
    // std::cout << "VM: " << vm << " MB    |    my_count: " << my_count_mb << " MB" << std::endl;
    std::cout << "VM: " << vm << " MB \t RSS: " << rss << " MB" << std::endl;
  }

  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  // usleep(10000000);

  /*
  // Unload state
  for (std::size_t i = 0; i < states.size(); ++i)
  {
    // Unload memory
    delete[] states[i]->values;
    delete states[i];

    // My count
    my_count_mb -= (state_size_b + array_size_b) / 1048576.0; // convert byte to megabyte

    // Get computer's measure of memory usage
    processMemUsage(vm, rss);

    // Output
    std::cout << "VM: " << vm << " MB    |    my_count: " << my_count_mb << " MB" << std::endl;
  }
  */

  while (true)
  {
    processMemUsage(vm, rss);
    std::cout << "VM: " << vm << " MB \t RSS: " << rss << " MB" << std::endl;
    // std::cout << "Memory usage: " << getValue() << " KB" << std::endl;

    usleep(1000000);
  }

  return 0;
}
