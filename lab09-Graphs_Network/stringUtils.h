#pragma once

/*
  Copyright (c) 2020
  Swarthmore College Computer Science Department, Swarthmore PA
  J. Brody, A. Danner, M. Gagne, L. Meeden, Z. Palmer, A. Soni, M. Wehar
  Distributed as course material for Fall 2020
  CPSC 035: Data Structures and Algorithms
  https://tinyurl.com/yyr8mdoh
*/

// This module contains a helper function to standardize error message
// generation in template classes.

#include <string>

namespace cs35 {
    template <typename T>
    std::string to_string(T t) {
        return std::to_string(t);
    }

    std::string to_string(std::string s);
}
