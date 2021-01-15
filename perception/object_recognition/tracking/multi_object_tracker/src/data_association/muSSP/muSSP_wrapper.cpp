#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <array>
#include <cassert>
#include "muSSP_wrapper.h"
#include "muSSP.h"

namespace mussp
{
    void MaximizeLinearAssignment(const std::vector<std::vector<double>> &cost,
                                  std::unordered_map<int, int> *direct_assignment,
                                  std::unordered_map<int, int> *reverse_assignment)
    {
        // Terminate if the graph is empty
        if (cost.size() == 0 || cost.at(0).size() == 0)
        {
            return;
        }

        // int n_rows, n_cols;
        // n_rows = cost.size();
        // n_cols = cost[0].size();
        std::cout << "muSSP is called!" << std::endl;

        // //////////////////////////////////
        // // Write graph into tmp file
        // //////////////////////////////////
        // // Open tmp file
        // std::string fpath = "/home/autoware/Desktop/data_association_algorithms/cpp/muSSP/.tmp/input_graph.txt";
        // std::ofstream outfile(fpath, std::ios_base::out);

        // // Write header
        // int n_nodes, n_edges;
        // n_nodes = (n_rows + n_cols + 1) * 2;
        // n_edges = n_rows * n_cols + (n_rows + n_cols) * 3;

        // outfile << "p min " << n_nodes << " " << n_edges << std::endl;
        // // Write arcs
        // //// source (1) -> o_i
        // for (int i = 1; i <= n_rows; i++)
        // {
        //     outfile << "a 1 " << i * 2 << " 0" << std::endl;
        // }
        // for (int j = 1; j <= n_cols; j++)
        // {
        //     outfile << "a 1 " << (j + n_rows) * 2 << " 100" << std::endl;
        // }

        // // o_j -> sink (n_nodes)
        // for (int i = 1; i <= n_rows; i++)
        // {
        //     outfile << "a " << i * 2 + 1 << " " << n_nodes << " 100" << std::endl;
        // }
        // for (int j = 1; j <= n_cols; j++)
        // {
        //     outfile << "a " << (j + n_rows) * 2 + 1 << " " << n_nodes << " 0" << std::endl;
        // }

        // //// o_i -> h_i
        // for (int i = 1; i <= n_rows; i++)
        // {
        //     outfile << "a " << i * 2 << " " << i * 2 + 1 << " 0" << std::endl;
        // }
        // for (int j = 1; j <= n_cols; j++)
        // {
        //     outfile << "a " << (j + n_rows) * 2 << " " << (j + n_rows) * 2 + 1 << " 0" << std::endl;
        // }

        // //// h_i -> o_j
        // for (int i = 1; i <= n_rows; i++)
        // {
        //     for (int j = 1; j <= n_cols; j++)
        //     {
        //         if (cost.at(i - 1).at(j - 1) > 0){
        //             outfile << "a " << i * 2 + 1 << " " << (j + n_rows) * 2 << " " << -cost.at(i - 1).at(j - 1) << std::endl;
        //         }
        //     }
        // }

        //////////////////////////////////
        // Solve by muSSP
        //////////////////////////////////
        // std::unordered_map<int, int> _assignment;
        // std::vector<std::vector<bool>> assignment_mat(n_rows, std::vector<bool>(n_cols, 0));
        solve_muSSP(cost, direct_assignment, reverse_assignment);
        // solve_muSSP(fpath, &assignment_mat);

//////////////////////////////////
// Format output
//////////////////////////////////

// Check the assignment is valid
// #ifndef NDEBUG
//         for (int i = 0; i < n_rows; i++)
//         {
//             int sum = 0;
//             for (int j = 0; j < n_cols; j++)
//             {
//                 sum += assignment_mat.at(i).at(j);
//             }
//             // assert(sum <= 1);
//             if (sum > 1)
//             {
//                 assert(sum <= 1);
//             }
//         }
//         for (int j = 0; j < n_cols; j++)
//         {
//             int sum = 0;
//             for (int i = 0; i < n_rows; i++)
//             {
//                 sum += assignment_mat.at(i).at(j);
//             }
//             assert(sum <= 1);
//         }
// #endif

        // // Format output
        // for (int i = 0; i < n_rows; i++)
        // {
        //     for (int j = 0; j < n_cols; j++)
        //     {
        //         if (assignment_mat.at(i).at(j) == 1)
        //         {
        //             (*direct_assignment)[i] = j;
        //             (*reverse_assignment)[j] = i;
        //         }
        //     }
        // }

        return;
    };
} // namespace mussp