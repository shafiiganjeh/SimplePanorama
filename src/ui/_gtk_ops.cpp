
#include "_gtk_ops.h"

namespace fs = std::filesystem;

namespace gops {

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


gboolean flowbox_item_clicked(GtkWidget* self,GdkEventButton event,gpointer _){

        GtkWidget* fbox_child = gtk_widget_get_parent (self);

        if (gtk_flow_box_child_is_selected (GTK_FLOW_BOX_CHILD(fbox_child))){
            gtk_flow_box_unselect_child(GTK_FLOW_BOX(gtk_widget_get_parent (fbox_child)),GTK_FLOW_BOX_CHILD(fbox_child));
            return TRUE;
        }
        else{
            return FALSE;
        }

}


void add_image_to_flowbox(struct flowbox_ &flowbox,GtkImage* image,std::string labelstr = "",std::string widget_name = "",int thumbnail_size = 250,int font_size = 12){

        PangoAttrList *const Attrs = pango_attr_list_new();
        PangoAttribute *const SizeAttr = pango_attr_size_new(font_size*PANGO_SCALE);
        pango_attr_list_insert(Attrs, SizeAttr);
        int i = flowbox.flowbox_items.size();

        //maximum name length and align
        int align_length = 0;
        if (labelstr.length() > 20){
            labelstr = labelstr.substr(0, 20).append("...");
            align_length = 23;
        }
        else{
            align_length = labelstr.length();
        }
        int label_aligment = (((thumbnail_size - align_length*font_size)/2) > 0) ? ((thumbnail_size - align_length*font_size)/2) : 0;
        label_aligment = label_aligment + align_length*2;
        const char *outfilename_ptr = labelstr.c_str();

        flowbox.flowbox_items.push_back(gtk_fixed_new ());
        flowbox.flowbox_event.push_back(gtk_event_box_new ());

        gtk_widget_set_name (flowbox.flowbox_event[i],widget_name.c_str() );

        flowbox.flowbox_items_imgs.push_back(image);
        flowbox.flowbox_items_label.push_back(GTK_LABEL(gtk_label_new(outfilename_ptr)));
        gtk_label_set_attributes(flowbox.flowbox_items_label[i], Attrs);
        gtk_widget_set_size_request(GTK_WIDGET(flowbox.flowbox_items[i]),thumbnail_size+20,thumbnail_size+20);
        gtk_widget_set_halign(GTK_WIDGET(flowbox.flowbox_items[i]),GTK_ALIGN_CENTER);
        gtk_widget_set_valign(GTK_WIDGET(flowbox.flowbox_items[i]),GTK_ALIGN_CENTER);

        gtk_fixed_put (GTK_FIXED(flowbox.flowbox_items[i]),GTK_WIDGET(flowbox.flowbox_items_imgs[i]),10,0);
        gtk_fixed_put(GTK_FIXED(flowbox.flowbox_items[i]),GTK_WIDGET(flowbox.flowbox_items_label[i]),label_aligment,thumbnail_size+10);

        gtk_container_add (GTK_CONTAINER (flowbox.flowbox_event[i]), flowbox.flowbox_items[i]);

        gtk_flow_box_insert(GTK_FLOW_BOX(flowbox.flowbox_main), flowbox.flowbox_event[i], i);

        gtk_widget_show(GTK_WIDGET(flowbox.flowbox_items_imgs[i]));
        gtk_widget_show(GTK_WIDGET(flowbox.flowbox_items_label[i]));
        gtk_widget_show(GTK_WIDGET(flowbox.flowbox_items[i]));
        gtk_widget_show(GTK_WIDGET(flowbox.flowbox_event[i]));

        gtk_widget_set_sensitive(GTK_WIDGET(flowbox.flowbox_event[i]),TRUE);
        gtk_widget_set_focus_on_click(GTK_WIDGET(flowbox.flowbox_event[i]),TRUE);

        g_signal_connect(flowbox.flowbox_event[i],"button_press_event", G_CALLBACK(flowbox_item_clicked),NULL);
}



int img_path_list_to_flowbox(std::vector<std::string> &path_list,struct image_paths* file_pointer,int thumbnail_size,struct flowbox_ flowbox){
        int missing = 0;

        for (int i = 0; i < path_list.size(); i++) {

            cv::Mat temp = cv::imread(path_list[i], cv::IMREAD_COLOR);

            if (temp.empty()){
                missing++;
                temp.release();
            }
            else{

                cv::Mat temp_res;
                temp_res = imgm::resize_image(temp,thumbnail_size);
                (file_pointer -> img_data).push_back(temp_res);
                (file_pointer -> f_list).push_back(path_list[i]);

                std::string outfilename_str = (fs::path(path_list[i].c_str()).stem()).string();

                add_image_to_flowbox(flowbox,cv_image_to_gtk_image(temp_res),outfilename_str,(file_pointer -> f_list)[i]);
                temp.release();

            }
        }
        return missing;
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
