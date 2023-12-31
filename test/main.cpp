#include <iostream>
#include <opencv2/flann.hpp>

using namespace std;

struct Test {
    cv::flann::Index *kd;
};

int main(int argc, char **argv) {
    // 用于构造kdtree的点集
    vector<cv::Point2f> features = {{1, 1}, {2, 2},     {3, 3},    {4, 4},
                                    {2, 4}, {122, 199}, {342, 147}};
    cv::Mat source = cv::Mat(features).reshape(1);
    std::cout << source << std::endl;
    source.convertTo(source, CV_32F);
    Test t;

    cv::flann::KDTreeIndexParams indexParams(2);
    cv::flann::Index *kdtree = new cv::flann::Index(source, indexParams);
    t.kd = kdtree;

    // 预设knnSearch所需参数及容器
    int queryNum = 10;                  // 用于设置返回邻近点的个数
    vector<double> vecQuery(2);          // 存放查询点的容器
    vector<int> vecIndex(queryNum);     // 存放返回的点索引
    vector<double> vecDist(queryNum);    // 存放距离
    cv::flann::SearchParams params(32); // 设置knnSearch搜索参数

    // KD树knn查询
    vecQuery = {124, 338};
    // kdtree.knnSearch(vecQuery, vecIndex, vecDist, queryNum, params);
    t.kd->radiusSearch(vecQuery, vecIndex, vecDist, 10, 10, params);

    cout << "vecDist: " << endl;
    for (auto &x : vecDist)
        cout << x << " ";
    cout << endl;

    cout << "vecIndex: " << endl;
    for (auto &x : vecIndex)
        cout << x << " ";

    return 0;
}
