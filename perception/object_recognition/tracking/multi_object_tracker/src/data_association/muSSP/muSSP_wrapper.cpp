#include "muSSP_wrapper.h"
#include <array>
#include <cassert>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "muSSP.h"

namespace mussp
{
void MaximizeLinearAssignment(
  const std::vector<std::vector<double>> & cost, std::unordered_map<int, int> * direct_assignment,
  std::unordered_map<int, int> * reverse_assignment)
{
  // Terminate if the graph is empty
  if (cost.size() == 0 || cost.at(0).size() == 0) {
    return;
  }

  // Solve DA by muSSP
  solve_muSSP(cost, direct_assignment, reverse_assignment);

  return;
};
}  // namespace mussp