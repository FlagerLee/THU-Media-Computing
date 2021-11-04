//
// Created by flagerlee on 2021/11/1.
//

#ifndef MEDIAPROC_IMG_COMPOSITING_H
#define MEDIAPROC_IMG_COMPOSITING_H

#include "library.h"

class QuadTree {
public:
    // method
    QuadTree() {
        lt = lb = rt = rb = nullptr;
        is_final = false;
        block_size = std::numeric_limits<int>::max();
    }

    // data
    QuadTree* lt, lb, rt, rb; // left-top, left-bottom, right-top, right-bottom
    bool is_final;
    int block_size;
};

#endif //MEDIAPROC_IMG_COMPOSITING_H
