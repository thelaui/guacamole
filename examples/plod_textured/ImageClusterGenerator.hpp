#ifndef IMAGE_CLUSTER_GENERATOR_HPP
#define IMAGE_CLUSTER_GENERATOR_HPP

#include <gua/math.hpp>
#include <gua/utils/Singleton.hpp>

#include "opencv2/opencv.hpp"

class ImageClusterGenerator : public gua::Singleton<ImageClusterGenerator> {

  public:

    void                      init(cv::Mat const& photo,
                                   cv::Mat const& screen_shot,
                                   int cluster_count);

    std::vector<float> const& get_centers() const { return centers_; };
    cv::Mat const&            get_cluster_labels() const { return cluster_labels_; };

    float                     calculate_center_ratio() const;

    friend class gua::Singleton<ImageClusterGenerator>;

  private:

    ImageClusterGenerator();

    std::vector<float> centers_;
    cv::Mat            cluster_labels_;

};

#endif  // IMAGE_CLUSTER_GENERATOR_HPP
