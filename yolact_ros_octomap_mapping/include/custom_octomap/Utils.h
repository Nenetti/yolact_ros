//
// Created by ubuntu on 2019/09/19.
//

#ifndef OCTOMAP_SERVER_UTILS_H
#define OCTOMAP_SERVER_UTILS_H


#include <cmath>

namespace custom_octomap {

    /// compute log-odds from probability:
    inline float logodds(double probability) {
        return (float) log(probability / (1 - probability));
    }

    /// compute probability from logodds:
    inline double probability(double logodds) {
        return 1. - (1. / (1. + exp(logodds)));
    }
    
}

#endif //OCTOMAP_SERVER_UTILS_H