
#ifndef GTKVARS_H
#define GTKVARS_H

#include <gtk/gtk.h>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

extern GtkTargetEntry targetentries[];

struct image_paths{
    std::vector<std::string> f_list;
    std::vector<cv::Mat> img_data;
};

struct flowbox_{
    GtkWidget *flowbox_main;
    GtkWidget *flowbox_scrolled_window;
    GtkWidget *flowbox_scrolled_window_viewpoint;

    std::vector<GtkWidget*> flowbox_items;
    std::vector<GtkWidget*> flowbox_event;
    std::vector<GtkLabel*> flowbox_items_label;
    std::vector<GtkImage*> flowbox_items_imgs;

};

struct menu_bar_{

   GtkWidget *menu_box;
   GtkWidget *bar;

   GtkWidget *bar_file;
   GtkWidget *bar_file_submenu;
   GtkWidget *bar_file_open;
   GtkWidget *bar_file_save;
   GtkWidget *bar_file_quit;

   GtkWidget *bar_edit;
   GtkWidget *bar_edit_submenu;
   GtkWidget *bar_edit_create;

   GtkWidget *bar_edit_cut;
   GtkWidget *bar_edit_select;
   GtkWidget *bar_edit_unselect;
   GtkWidget *bar_edit_order;


   GtkWidget *toolbar;
};

struct {
    GtkWidget *window;
    GtkWidget *box;
    GtkWidget *img_dragdrop;
    GtkWidget *img_dragdrop_frame;
    struct flowbox_ flowbox;
    struct menu_bar_ menu_bar;
} gtk_var;


#endif
