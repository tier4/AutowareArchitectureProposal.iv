#ifndef MUSSP_H_
#define MUSSP_H_

#include <unordered_map>
#include <vector>

namespace mussp
{
void MaximizeLinearAssignment(
  const std::vector<std::vector<double>> & cost, std::unordered_map<int, int> * direct_assignment,
  std::unordered_map<int, int> * reverse_assignment);
}  // namespace mussp

#endif