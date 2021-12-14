//
// Created by flagerlee on 2021/11/1.
//

#include <cmath>
#include <string>
#include <queue>
#include <unordered_map>
#include <map>
#include <set>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

#define INF 100000007

struct cmp {
    bool operator()(const Point2i &p1, const Point2i &p2) {
        if (p1.x != p2.x) return p1.x < p2.x;
        else return p1.y < p2.y;
    }
};

static Point2i start_point;
static Point2i last_point;
static vector<Point2i> point_list;
static map<Point2i, Vec3b, cmp> color_map;
static bool pressed = false;
static Mat image;

vector<Point2i> connect_points(const Point2i &start, const Point2i &end) {
    vector<Point2i> points;
    const int delta_x = (int) end.x - (int) start.x;
    const int s_delta_x = abs(delta_x);
    const int delta_y = (int) end.y - (int) start.y;
    const int s_delta_y = abs(delta_y);
    if (s_delta_x != 0 || s_delta_y != 0) {
        if (s_delta_x >= s_delta_y) {
            int s_one = delta_x > 0 ? 1 : -1;
            for (int dx = s_one; abs(dx) < s_delta_x; dx += s_one) {
                points.emplace_back(Point2i(
                        start.x + dx,
                        start.y + floor((float) delta_y * (float) dx / (float) delta_x)
                ));
            }
        } else {
            int s_one = delta_y > 0 ? 1 : -1;
            for (int dy = s_one; abs(dy) < s_delta_y; dy += s_one) {
                points.emplace_back(Point2i(
                        start.x + floor((float) delta_x * (float) dy / (float) delta_y),
                        start.y + dy
                ));
            }
        }
    }
    return points;
}

void change_color(const Point2i &point, const Point3i color) {
    auto &pixel = image.at<Vec3b>(point);
    color_map.insert(make_pair(point, pixel));
    pixel = Vec3b(color.x, color.y, color.z);
}

void change_color(const vector<Point2i> &points, const Point3i color) {
    for (Point2i point: points) {
        auto &pixel = image.at<Vec3b>(point);
        color_map.insert(make_pair(point, pixel));
        pixel = Vec3b(color.x, color.y, color.z);
        // image.at<Vec3b>(point)[0] = color.x;
        // image.at<Vec3b>(point)[1] = color.y;
        // image.at<Vec3b>(point)[2] = color.z;
    }
}

void onMouse(int event, int x, int y, int flags, void *param) {
    if (event == EVENT_LBUTTONDOWN) {
        pressed = true;
        start_point = Point2i(x, y);
        last_point = start_point;
        point_list.push_back(start_point);
        change_color(start_point, Point3i(255, 0, 0));
    } else if (event == EVENT_MOUSEMOVE) {
        if (!pressed) return;
        Point2i point(x, y);
        auto points = connect_points(last_point, point);
        point_list.emplace_back(point);
        point_list.insert(point_list.end(), points.begin(), points.end());
        last_point = point;
        change_color(last_point, Point3i(255, 0, 0));
        change_color(points, Point3i(255, 0, 0));
    } else if (event == EVENT_LBUTTONUP) {
        pressed = false;
        Point2i end_point = Point2i(x, y);
        auto points = connect_points(last_point, end_point);
        change_color(points, Point3i(255, 0, 0));
        point_list.insert(point_list.end(), points.begin(), points.end());
        change_color(end_point, Point3i(255, 0, 0));
        point_list.push_back(end_point);
        points = connect_points(end_point, start_point);
        change_color(points, Point3i(255, 0, 0));
        point_list.insert(point_list.end(), points.begin(), points.end());

        sort(point_list.begin(), point_list.end(), [](const Point2i &p1, const Point2i &p2) {
            if (p1.x != p2.x) return p1.x < p2.x;
            else return p1.y < p2.y;
        });
        point_list.erase(unique(point_list.begin(), point_list.end()), point_list.end());
    }
}

void flood_fill(int bound_x, int bound_y) {
    typedef pair<int, int> _point;
    bool vis[bound_x][bound_y];
    for (int i = 0; i < bound_x; i++)
        for (int j = 0; j < bound_y; j++)
            vis[i][j] = false;

    auto cmp = [](const Point2i &p1, const Point2i &p2) {
        if (p1.x != p2.x) return p1.x < p2.x;
        else return p1.y < p2.y;
    };
    set<Point2i, decltype(cmp)> s(cmp);
    s.insert(point_list.begin(), point_list.end());
    queue<_point> q;
    q.push(make_pair(0, 0));
    while (!q.empty()) {
        _point p = q.front();
        q.pop();
        int x = p.first, y = p.second;
        if (vis[x][y]) continue;
        vis[x][y] = true;
        if (s.find(Point2i(x, y)) == s.end()) {
            change_color(Point2i(x, y), Point3i(0, 0, 0));
            if (x > 0 && !vis[x - 1][y]) q.push(make_pair(x - 1, y));
            if (x < bound_x - 1 && !vis[x + 1][y]) q.push(make_pair(x + 1, y));
            if (y > 0 && !vis[x][y - 1]) q.push(make_pair(x, y - 1));
            if (y < bound_y - 1 && !vis[x][y + 1]) q.push(make_pair(x, y + 1));
        }
    }
}

void recover() {
    for (Point2i p: point_list) {
        auto &pixel = image.at<Vec3b>(p);
        pixel = color_map[p];
    }
}

void clear() {
    recover();
    point_list.erase(point_list.begin(), point_list.end());
    color_map.erase(color_map.begin(), color_map.end());
}

void imgproc() {
    image = imread("../../img/boat.jpeg");
    if (image.empty()) {
        perror("could not read image");
        return;
    }
    Size_<int> dsize = {800, 1200};
    resize(image, image, dsize);
    const string title = "image";
    namedWindow(title);
    setMouseCallback(title, onMouse, 0);
    while (true) {
        imshow(title, image);
        int key = waitKey(20);
        if (key == 27) clear();
        if (key == 13) break;
        if (getWindowProperty(title, 0) == -1)
            break;
    }
    FILE* f = fopen("../seam/seam.txt", "w");
    for(Point2i p : point_list) fprintf(f, "%d %d\n", p.x, p.y);
    fclose(f);
    flood_fill(image.cols, image.rows);
    recover();
    imwrite("../../img/proc.png", image);
    imshow(title, image);
    waitKey(0);
}
