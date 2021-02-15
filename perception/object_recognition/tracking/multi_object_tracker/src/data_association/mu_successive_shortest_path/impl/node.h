/*
 * Copyright 2019 Virginia Tech. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#pragma once
#include <vector>

class Node
{
public:
    Node() = default;
//    int get_id() const;

//    int node_id = 0;
//    double shortest_path;
    std::vector<int> precursor_idx;
    std::vector<int> precursor_edges_idx;
    std::vector<double> precursor_edges_weights;

    std::vector<int> successor_idx;
    std::vector<int> successor_edges_idx;
    std::vector<double> successor_edges_weights;

    double price = 0;

//    bool visited = false;
    //bool in_tree = false;
//    Node *parent_node = nullptr; //parent node in shortest path tree

    void add_precursor(int pre_id, int pre_edge_id, double weight);
    void add_successor(int succ_id, int succ_edge_id, double weight);
//    void delete_precursor(int pre_id);
//    void delete_successor(int succ_id);
};
