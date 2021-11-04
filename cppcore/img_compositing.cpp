//
// Created by flagerlee on 2021/11/1.
//

#include "img_compositing.h"

namespace composite {

    Solver::Solver(cv::Mat) {

    }

    const int kNumQuadTreeSize = 16;
    constexpr int kQuadTreeSizes[kNumQuadTreeSize] = {
            (1 << 0) + 1,
            (1 << 1) + 1,
            (1 << 2) + 1,
            (1 << 3) + 1,
            (1 << 4) + 1,
            (1 << 5) + 1,
            (1 << 6) + 1,
            (1 << 7) + 1,
            (1 << 8) + 1,
            (1 << 9) + 1,
            (1 << 10) + 1,
            (1 << 11) + 1,
            (1 << 12) + 1,
            (1 << 13) + 1,
            (1 << 14) + 1,
            (1 << 15) + 1
    };

    void Solver::CreateQuadTree(std::vector<coordinate> seam, cv::Mat img) {
        // process twice.
        // create a regular quadtree for the first time
        // then add constraint "no two nodes that share an edge may differ in tree depth by more than one"

        // create root node
        int max_size = std::max(img.size[0], img.size[1]);

    }

    cv::Mat Solver::VisualizeQuadTree() {
        return cv::Mat();
    }
}