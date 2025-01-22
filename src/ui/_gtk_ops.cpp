
#include "_gtk_ops.h"

namespace fs = std::filesystem;

namespace gops {

void set_css_style(GtkCssProvider *provider,GtkWidget *widget){

    GtkStyleContext* context = gtk_widget_get_style_context(widget);
    //gtk_style_context_add_provider_for_screen();

}


int findStringIndex(const std::vector<std::string>& vec, const std::string& target) {
    auto it = std::find(vec.begin(), vec.end(), target);
    if (it != vec.end()) {
        return std::distance(vec.begin(), it);
    } else {
        return -1;
    }
}

std::tuple<const std::vector<uint8_t>,const int,const int> to_img_buffer(const cv::Mat& img){
        cv::Mat RGB_conv;
        cv::cvtColor(img, RGB_conv, cv::COLOR_BGR2RGB);


        std::vector<uint8_t> RGB_buffer;

        if (RGB_conv.isContinuous()) {
        RGB_buffer.assign(RGB_conv.data, RGB_conv.data + RGB_conv.total()*RGB_conv.channels());
        }
        else {
        for (int i = 0; i < RGB_conv.rows; ++i) {
            RGB_buffer.insert(RGB_buffer.end(), RGB_conv.ptr<uint8_t>(i), RGB_conv.ptr<uint8_t>(i)+RGB_conv.cols*RGB_conv.channels());
        }}

        return {RGB_buffer,(int) RGB_conv.rows,(int) RGB_conv.cols};
}


GtkImage* cv_image_to_gtk_image(cv::Mat cv_image){

        auto [RGB_buffer,col,row] = to_img_buffer(cv_image);
        GBytes* ad = g_bytes_new (&RGB_buffer[0],col*row*3);
        GdkPixbuf* pixbuff = gdk_pixbuf_new_from_bytes (ad,GDK_COLORSPACE_RGB,FALSE,8,col,row,col*3);
        return GTK_IMAGE(gtk_image_new_from_pixbuf (pixbuff));
}


std::string path_from_uri(std::string uri){
        std::string ipath = uri.substr(uri.find_first_of("//")+2,uri.length());
        if (!ipath.empty() && ipath[ipath.size() - 1] == '\r'){
            ipath.erase(ipath.size() - 1);

        }
        return ipath;
}

std::vector<std::string> get_path_from_Selection_data(struct image_paths* file_pointer, GtkSelectionData* data){

        std::vector<std::string> path_list;

        const unsigned char* seldata = gtk_selection_data_get_data(data);
        std::stringstream stream((char *)seldata);

        std::string line;
        while (std::getline(stream, line)) {

            line = path_from_uri(line);

            if (std::find((file_pointer -> f_list).begin(), (file_pointer -> f_list).end(), line) == (file_pointer -> f_list).end())
                {
                    path_list.push_back(line);
                }
        }
        return path_list;
}
}



