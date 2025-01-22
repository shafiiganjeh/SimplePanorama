

#ifndef GTKOPS_H
#define GTKOPS_H
#include "_gtk_vars.h"
#include "_img_manipulation.h"
#include <filesystem>


namespace gops {
int findStringIndex(const std::vector<std::string>& vec, const std::string& target);

std::tuple<const std::vector<uint8_t>,const int,const int> to_img_buffer(const cv::Mat& img);

GtkImage* cv_image_to_gtk_image(cv::Mat cv_image);

std::string path_from_uri(std::string uri);

std::vector<std::string> get_path_from_Selection_data(struct image_paths* file_pointer, GtkSelectionData* data);

void set_css_style(GtkCssProvider *provider,GtkWidget *widget);

}

#endif
