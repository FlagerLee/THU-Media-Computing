//
// Created by flagerlee on 2021/11/1.
//

#ifndef CPPCORE_IMG_COMPOSITING_H
#define CPPCORE_IMG_COMPOSITING_H

#include "library.h"

namespace composite {
    typedef std::pair<int, int> point;

    class QuadTreeNode {
    public:
        // method
        QuadTreeNode(point ltp, int blk_size);
        ~QuadTreeNode();
        bool isLeaf() {
            return lt == nullptr && lb == nullptr && rt == nullptr && rb == nullptr;
        }

        // data
        QuadTreeNode *lt, *lb, *rt, *rb; // left-top, left-bottom, right-top, right-bottom
        point ltp; // left-top point
        int block_size;
    };

    class Solver {
    public:
        // method
        explicit Solver(const cv::Mat&, const std::vector<cv::Point2i>&);

        void CreateQuadTree(const std::vector<cv::Point2i>&);

        cv::Mat VisualizeQuadTree();

        // data
        QuadTreeNode *root;
        cv::Mat src_img;
        cv::Mat sTaTas, sTaTb;
        std::vector<double> x_delta, y_delta;
        int x_dim, y_dim;
    };
}

#endif //CPPCORE_IMG_COMPOSITING_H
