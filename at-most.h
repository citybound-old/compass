//
// Created by Anselm Eickhoff on 14/02/16.
//

#ifndef COMPASS_AT_MOST_H
#define COMPASS_AT_MOST_H

#include "static-queue/StaticQueue.h"

template <int max_n, typename T>
using AtMost = squeue::StaticQueue<T, max_n>;
#endif //COMPASS_AT_MOST_H
