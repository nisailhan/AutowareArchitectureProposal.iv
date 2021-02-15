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
#include "node.h"
#include <algorithm>       /* fabs */
//#include<bits/stdc++.h>
#include <map>
#include <limits>
//#include <iostream>

#define MUSSP_FINF 1000000.0 //numeric_limits<double>::max()
#define MUSSP_FINFHALF MUSSP_FINF/2.0

///
/// \brief The Sink class
///
class Sink
{
public:
    // use set to save sink's precursors' distances
    std::multimap<double, int> sink_precursors;
    std::vector<double> sink_precursor_weights;
    double sink_cost_ = 0; // this can be a vector, in our framework, it it a scaler
    double sink_weight_shift = 0;

    Sink() = default;
    Sink(int n, double sink_cost);

    void sink_update_all(std::vector<Node> &V, std::vector<double> &distance2src, int sink_id, int n);

    void sink_update_all_weight(std::vector<Node> &V, std::vector<double> &distance2src, int sink_id, int n);


    void sink_build_precursormap(std::vector<double> &ancestor_ssd, std::vector<int> &ancestor_node_id, std::vector<int> &parent_node_id, int n);


    void sink_update_all_half(std::vector<double> distance2src, int sink_id, int n);
    void sink_update_subgraph(std::vector<int> update_node_id, std::vector<double> distance2src, int sink_id, int n);
};
