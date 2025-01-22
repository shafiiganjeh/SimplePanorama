
#include "_flowbox.h"
namespace fs = std::filesystem;


GtkTargetEntry targetentries[] =
{
    { (char*)"text/uri-list", 0, 0}
};

namespace fbox{
/*
void drag_ind_set(GtkWidget *widget,GdkDragContext *context,gpointer user_data){
    GtkWidget* fbox_child = gtk_widget_get_parent (widget);

    DRAG_IND = gtk_flow_box_child_get_index (GTK_FLOW_BOX_CHILD(fbox_child));
    std::cout <<"asd";

}
*/


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

void add_image_to_flowbox(struct flowbox_ *flowbox, GtkImage* image,struct main_window_ *main_window,std::string labelstr,std::string widget_name ,int index ,int thumbnail_size,int font_size){

        PangoAttrList *const Attrs = pango_attr_list_new();
        PangoAttribute *const SizeAttr = pango_attr_size_new(font_size*PANGO_SCALE);
        pango_attr_list_insert(Attrs, SizeAttr);
        int i = flowbox->flowbox_items.size();

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

        (flowbox->flowbox_items).push_back(gtk_fixed_new ());
        (flowbox->flowbox_event).push_back(gtk_event_box_new ());

        gtk_widget_set_name (flowbox->flowbox_event[i],widget_name.c_str() );

        (flowbox->flowbox_items_imgs).push_back(image);
        (flowbox->flowbox_items_label).push_back(GTK_LABEL(gtk_label_new(outfilename_ptr)));
        gtk_label_set_attributes(flowbox->flowbox_items_label[i], Attrs);
        gtk_widget_set_size_request(GTK_WIDGET(flowbox->flowbox_items[i]),thumbnail_size+20,thumbnail_size+20);
        gtk_widget_set_halign(GTK_WIDGET(flowbox->flowbox_items[i]),GTK_ALIGN_CENTER);
        gtk_widget_set_valign(GTK_WIDGET(flowbox->flowbox_items[i]),GTK_ALIGN_CENTER);

        gtk_fixed_put (GTK_FIXED(flowbox->flowbox_items[i]),GTK_WIDGET(flowbox->flowbox_items_imgs[i]),10,0);
        gtk_fixed_put(GTK_FIXED(flowbox->flowbox_items[i]),GTK_WIDGET(flowbox->flowbox_items_label[i]),label_aligment,thumbnail_size+10);

        gtk_container_add (GTK_CONTAINER (flowbox->flowbox_event[i]), flowbox->flowbox_items[i]);
        //std::cout << "index: " << index <<"\n";
        gtk_flow_box_insert(GTK_FLOW_BOX(flowbox->flowbox_main), flowbox->flowbox_event[i], index);

        gtk_widget_show(GTK_WIDGET(flowbox->flowbox_items_imgs[i]));
        gtk_widget_show(GTK_WIDGET(flowbox->flowbox_items_label[i]));
        gtk_widget_show(GTK_WIDGET(flowbox->flowbox_items[i]));
        gtk_widget_show(GTK_WIDGET(flowbox->flowbox_event[i]));

        gtk_widget_set_sensitive(GTK_WIDGET(flowbox->flowbox_event[i]),TRUE);
        gtk_widget_set_focus_on_click(GTK_WIDGET(flowbox->flowbox_event[i]),TRUE);

        gtk_drag_dest_set(GTK_WIDGET(flowbox->flowbox_event[i]), GTK_DEST_DEFAULT_ALL, targetentries, 1, GDK_ACTION_COPY);

        g_signal_connect(flowbox->flowbox_event[i],"button_press_event", G_CALLBACK(flowbox_item_clicked),NULL);
        g_signal_connect(flowbox->flowbox_event[i],"drag_data_received", G_CALLBACK(drag_rec),main_window);
}

int img_path_list_to_flowbox(std::vector<std::string> &path_list,int thumbnail_size,struct main_window_ *main_window,int index){
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
                (main_window -> ipts).img_data.push_back(temp_res);
                (main_window -> ipts).f_list.push_back(path_list[i]);

                std::string outfilename_str = (fs::path(path_list[i].c_str()).stem()).string();

                add_image_to_flowbox(&(main_window->flowbox),gops::cv_image_to_gtk_image(temp_res),main_window ,outfilename_str,path_list[i],index);

                if (index > -1){index = index +1;}

                temp.release();

            }
        }

        return missing;
}


void drag_rec(GtkWidget *widg, GdkDragContext* context, gint x,
                               gint y, GtkSelectionData* data, guint info, guint time,struct main_window_ *main_window)
{
        int thumbnail_size = 250;
        int index = 0;

        GtkWidget* fbox_child = gtk_widget_get_parent (widg);
        index  = gtk_flow_box_child_get_index (GTK_FLOW_BOX_CHILD(fbox_child));

        std::vector<std::string> path_list = gops::get_path_from_Selection_data(&(main_window->ipts),data);

        int missing = img_path_list_to_flowbox(path_list,thumbnail_size,main_window,index);

        if ((main_window->ipts).f_list.size()>0){
            if (gtk_widget_get_visible(GTK_WIDGET(main_window->img_dragdrop))){
                gtk_widget_set_visible(GTK_WIDGET(main_window->img_dragdrop),FALSE);
                gtk_widget_set_sensitive(GTK_WIDGET(main_window->img_dragdrop),FALSE);
        }

            if (!gtk_widget_get_visible(GTK_WIDGET(main_window->flowbox.flowbox_scrolled_window))){
                gtk_widget_set_visible(GTK_WIDGET(main_window->flowbox.flowbox_scrolled_window),TRUE);
                gtk_widget_set_sensitive(GTK_WIDGET(main_window->flowbox.flowbox_scrolled_window),TRUE);
                gtk_widget_set_sensitive(GTK_WIDGET(main_window->menu_bar.bar_edit),TRUE);
        }}
        guint32 time_ = 0;
        gtk_drag_finish (context,
                 TRUE,
                 TRUE,
                 time_);

}


void connect_signals(struct flowbox_ *flowbox,struct main_window_ *main_window){

       g_signal_connect(main_window->flowbox.flowbox_main, "drag_data_received", G_CALLBACK(drag_rec), main_window);

}

void build_flowbox(GtkWidget *add_to,struct flowbox_ *flowbox,struct main_window_ *main_window){
        flowbox->flowbox_main = gtk_flow_box_new();
        gtk_flow_box_set_selection_mode(GTK_FLOW_BOX(flowbox->flowbox_main),GTK_SELECTION_MULTIPLE);
        flowbox->flowbox_scrolled_window = gtk_scrolled_window_new (NULL, NULL);
        gtk_drag_dest_set(GTK_WIDGET(flowbox->flowbox_main), GTK_DEST_DEFAULT_ALL, targetentries, 1, GDK_ACTION_COPY);
        flowbox->flowbox_scrolled_window_viewpoint = gtk_viewport_new (NULL, NULL);

        gtk_container_add(GTK_CONTAINER(flowbox->flowbox_scrolled_window_viewpoint), flowbox->flowbox_main);
        gtk_container_add(GTK_CONTAINER(flowbox->flowbox_scrolled_window), flowbox->flowbox_scrolled_window_viewpoint);
        gtk_box_pack_end (GTK_BOX(add_to), flowbox->flowbox_scrolled_window,TRUE,TRUE,0);

        gtk_widget_set_visible(GTK_WIDGET(flowbox->flowbox_scrolled_window),FALSE);
        gtk_widget_set_sensitive(GTK_WIDGET(flowbox->flowbox_scrolled_window),FALSE);

        connect_signals(flowbox,main_window);
}
}
