

#ifndef MATHS_H
#define MATHS_H

#include <vector>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <numeric>
#include <algorithm>
#include <random>
#include <thread>
#include <future>
#include <stack>
#include <unordered_set>
#include <map>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <opencv2/core/eigen.hpp>

namespace util {

    struct val {
        int int_part;
        double double_part;
    };

    double stringToDouble(const std::string& str);

    float stringToFloat(const std::string& str);

    int stringToInt(const std::string& str);

    val processValue(double value, int max_val);

        //function for performance check
    class Timer {
        using Clock = std::chrono::high_resolution_clock;
        using TimePoint = std::chrono::time_point<Clock>;
        using Duration = std::chrono::duration<double>;

        std::unordered_map<std::string, TimePoint> start_times;
        std::unordered_map<std::string, Duration> durations;

    public:
        void start(const std::string& name) {
            start_times[name] = Clock::now();
        }

        void stop(const std::string& name) {
            auto end = Clock::now();
            auto it = start_times.find(name);
            if (it != start_times.end()) {
                durations[name] += end - it->second;
                start_times.erase(it);
            }
        }

        double getDuration(const std::string& name) const {
            auto it = durations.find(name);
            if (it != durations.end()) {
                return it->second.count();
            }
            return 0.0;
        }

        void reset(const std::string& name) {
            durations.erase(name);
            start_times.erase(name);
        }
    };

    struct adj_str {
            cv::Mat adj;
            std::vector<double> connectivity;
            int nodes;
        };

    struct translation{
        cv::Matx33f T;
        float xstart;
        float xend;
        float ystart;
        float yend;
    };


    struct size_data{

        cv::Size dims;
        int min_x;
        int min_y;
        int max_x;
        int max_y;

    };

    using thread = std::vector<std::vector<std::vector<int>>>;

    cv::Rect scaleRect(const cv::Rect& r, double xs, double sy);

    std::vector<double> computeRowSumDividedByZeroCount(const cv::Mat& mat);

    struct size_data get_pan_dimension(const std::vector<cv::Point>& top_lefts,const std::vector<cv::Mat>& images);

    std::vector<int> randomN(int n, int m);

    struct translation get_translation(const cv::Size &base, const cv::Mat &attach,const cv::Matx33f &H);

    std::map<int, std::pair<int,double>> path_table(const cv::Mat& adj,const std::vector<std::pair<int, std::vector<int>>> &nodes,int start);

    std::vector<int> dfs(cv::Mat& graph, int source);

    std::vector<std::pair<int, std::vector<int>>> bfs_ordered_with_neighbors(const cv::Mat& adj, int i);

    float focal_from_hom(const std::vector<std::vector< cv::Matx33f >> & H_mat,const cv::Mat &source_adj);

    template <typename T>
    std::vector<std::vector<T>> splitVector(const std::vector<T>& vec, int n) {
            std::vector<std::vector<T>> result;

            if (n <= 0 || vec.empty()) {
                return result;
            }

            if (n > vec.size()) {

                throw std::invalid_argument("n should be less than vector size.");
            }

            int basic_part_size = vec.size() / n;
            int remainder = vec.size() % n;

            int start_index = 0;

            for (int i = 0; i < n; ++i) {
                int current_part_size = basic_part_size + (i < remainder ? 1 : 0);  // Add 1 if i is less than remainder
                std::vector<T> part(vec.begin() + start_index, vec.begin() + start_index + current_part_size);
                result.push_back(part);
                start_index += current_part_size;
            }

            return result;
    }

    std::vector<struct adj_str> extract_adj(const cv::Mat &adj);


}

#endif
