//
// Created by flagerlee on 2021/12/3.
//

#include <cstdio>
#include "library.h"
#include "img_compositing.h"
#include "img_proc.h"

using namespace cv;

int main() {
    Mat bg_img = imread("../../img/sea.jpeg");
    Mat proc_img = imread("../../img/proc.png");
    const int dx = 600, dy = 1200;
    for(int i = 0; i < proc_img.rows; i ++)
        for(int j = 0; j < proc_img.cols; j ++) {
            auto &color = proc_img.at<Vec3b>(i, j);
            if (!color[0] && !color[1] && !color[2]) continue;
            bg_img.at<Vec3b>(dx + i, dy + j) = color;
        }
    FILE* seam_file = fopen("../seam/seam.txt", "r");
    std::vector<Point2i> seam;
    int x, y;
    while(fscanf(seam_file, "%d %d", &y, &x) == 2) {
        seam.emplace_back(Point2i(dx + x, dy + y));
    }
    for(int i = 0; i < bg_img.rows; i ++) seam.emplace_back(Point2i(i, bg_img.cols - 1));
    for(int i = 0; i < bg_img.cols - 1; i ++) seam.emplace_back(Point2i(bg_img.rows - 1, i));
    composite::Solver solver(bg_img, seam);
    Mat quadtree_img = solver.VisualizeQuadTree();

    // Size_<int> dsize = {1500, 1000};
    // resize(quadtree_img, quadtree_img, dsize);
    imshow("window", quadtree_img);
    waitKey(0);
    imwrite("../../img/quatree.png", quadtree_img);
}