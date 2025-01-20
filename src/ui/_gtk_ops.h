

#ifndef GTKOPS_H
#define GTKOPS_H
#include "_gtk_vars.h"
#include "_img_manipulation.h"
#include <filesystem>


namespace gops {

gboolean flowbox_item_clicked(GtkWidget* self,GdkEventButton event,gpointer _);

std::tuple<const std::vector<uint8_t>,const int,const int> to_img_buffer(const cv::Mat& img);

GtkImage* cv_image_to_gtk_image(cv::Mat cv_image);

void add_image_to_flowbox(struct flowbox_ &flowbox,GtkImage* image,std::string labelstr,std::string widget_name,int thumbnail_size,int font_size);

int img_path_list_to_flowbox(std::vector<std::string> &path_list,struct image_paths* file_pointer,int thumbnail_size,struct flowbox_ flowbox);

std::string path_from_uri(std::string uri);

std::vector<std::string> get_path_from_Selection_data(struct image_paths* file_pointer, GtkSelectionData* data);

}

#endif
