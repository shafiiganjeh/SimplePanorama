
#ifndef GTKVARS_H
#define GTKVARS_H

#include <gtk/gtk.h>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "_image.h"
#include "_panorama.h"

extern GtkTargetEntry targetentries[];





struct task{

    guint bar_timer_id;
    std::atomic<bool> cancel = false;
    std::atomic<double> fraction = 0;

};


struct image_paths{
    std::vector<std::string> f_list;
    std::vector<cv::Mat> img_data;
};


struct Drag_Par{
    gdouble start_x;
    gdouble start_y;
    gdouble hadj_value;
    gdouble vadj_value;
    gboolean is_dragging;
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

};

struct toolbar_{
    GtkWidget *toolbar_main;

    GtkToolItem *toolbar_main_cut;
    GtkWidget *toolbar_main_cut_img;

    GtkWidget *toolbar_main_new_img;
    GtkToolItem *toolbar_main_new;

    GtkWidget *toolbar_main_testing_img;
    GtkToolItem *toolbar_main_testing;
    GtkWidget *test_object;
    bool tested = false;


    GtkToolItem *toolbar_main_rewind;
    GtkToolItem *toolbar_main_create;
};


struct toolbar_viewer{

    GtkWidget *toolbar_main;

    GtkToolItem *toolbar_seperator_1;

    GtkToolItem *toolbar_main_save;
    GtkWidget *toolbar_main_save_img;

    GtkToolItem *toolbar_main_crop;
    GtkWidget *toolbar_main_crop_img;

    GtkToolItem *toolbar_main_zin;
    GtkWidget *toolbar_main_zin_img;

    GtkToolItem *toolbar_main_zout;
    GtkWidget *toolbar_main_zout_img;


};


struct viewer_window_{

    GtkWidget *window;
    GtkWidget *viewer_box;
    GtkWidget *viewer_scrolled_window;
    GtkWidget *viewer_scrolled_window_viewpoint;

    struct toolbar_viewer toolbar;

    GtkImage* img_test;
    GtkWidget *image_window_event;
    cv::Mat image;
    cv::Mat image_zoomed;
    std::vector<int> zoom_val;
    int current_zoom;

    cv::Rect image_rect;
    cv::Rect window_rect;

    struct Drag_Par dragging;

    std::unique_ptr<pan::panorama> panorama;

};


struct main_window_{
    GtkWidget *window;
    GtkWidget *box;
    GtkWidget *img_dragdrop;
    GtkWidget *img_dragdrop_frame;
    struct flowbox_ flowbox;
    struct menu_bar_ menu_bar;
    struct toolbar_ toolbar;

    std::map<int, struct viewer_window_> view;
    int view_number = 1;

    struct image_paths ipts;


};




#endif
