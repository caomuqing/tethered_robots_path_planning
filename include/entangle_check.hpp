#ifndef ENTANGLE_CHECK_HPP_
#define ENTANGLE_CHECK_HPP_

#include <iostream>
#include <cstdio>
#include <numeric>      // std::iota
#include <algorithm>    // std::sort, std::stable_sort
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;

bool check3robotEnt(std::vector<Vector2i>& v, Vector2i to_add);

template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v) ;

#endif 