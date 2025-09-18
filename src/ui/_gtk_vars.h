
#ifndef GTKVARS_H
#define GTKVARS_H

#include <gtk/gtk.h>
#include <glib-object.h>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <filesystem>

extern GtkTargetEntry targetentries[];
namespace pan { class panorama;
                struct config;}

struct progress_bar_{

    guint bar_timer_id = false;
    std::atomic<bool> finished = false;
    std::atomic<double> fraction = 0;
    std::atomic<bool> thread_save = false;
    GtkWidget *window;
    GtkWidget *pbar_main_vbox;
    GtkWidget *pbar_progress_bar;
    GtkWidget *pbar_button_box;
    GtkWidget *pbar_button_box_cancel;

    struct viewer_window_* view;
    struct main_window_ * main_window;
    bool canceld = false;
    bool error = false;
    const gchar* what_error;
    int test;

    GtkWidget *instance;
    gchar* loading_text;

    void bar_text(const gchar* source) {

        delete[] loading_text;
        loading_text = nullptr;

        if (source) {
            loading_text = new char[strlen(source) + 1];
            strcpy(loading_text, source);
        }
    }
    //manual construction/cleanup since we interface with C

    void init() {
        bar_text("");
    }

    void cleanup() {
        delete[] loading_text;
    }

};


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


struct config_{

    GtkWidget *conf_menu;
    GtkWidget *conf_menu_paned;
    GtkWidget *conf_menu_paned_sidebar;
    GtkWidget *conf_menu_box;
    GtkWidget *conf_menu_cancelok_box;
    GtkWidget *conf_menu_OK;
    GtkWidget *conf_menu_CANCEL;

    GtkWidget *conf_menu_stack;
    //adv
    GtkWidget *conf_menu_stack_advanced_scrolled_window;
    GtkWidget *conf_menu_stack_advanced_viewport;
    GtkWidget *conf_menu_stack_advanced_box;

    GtkWidget *conf_menu_stack_advanced_frame_system;
    GtkWidget *conf_menu_stack_advanced_frame_system_box;//val
    GtkWidget *conf_menu_stack_advanced_frame_system_entry_threads;
    GtkWidget *conf_menu_stack_advanced_frame_system_entry_size;

    GtkWidget *conf_menu_stack_advanced_frame_matching;
    GtkWidget *conf_menu_stack_advanced_frame_matching_box;//val
    GtkWidget *conf_menu_stack_advanced_frame_matching_entry_maximages;
    GtkWidget *conf_menu_stack_advanced_frame_matching_entry_maxkeypoints;
    GtkWidget *conf_menu_stack_advanced_frame_matching_entry_RANSAC;
    GtkWidget *conf_menu_stack_advanced_frame_matching_entry_xmargin;
    GtkWidget *conf_menu_stack_advanced_frame_matching_entry_minoverlap;
    GtkWidget *conf_menu_stack_advanced_frame_matching_entry_overlapinlmatch;
    GtkWidget *conf_menu_stack_advanced_frame_matching_entry_overlapinlkeyp;
    GtkWidget *conf_menu_stack_advanced_frame_matching_entry_conf;

    GtkWidget *conf_menu_stack_advanced_frame_SIFT;
    GtkWidget *conf_menu_stack_advanced_frame_SIFT_box;//val
    GtkWidget *conf_menu_stack_advanced_frame_system_entry_nfeatures;
    GtkWidget *conf_menu_stack_advanced_frame_system_entry_nOctaveLayers;
    GtkWidget *conf_menu_stack_advanced_frame_system_entry_contrastThreshold;
    GtkWidget *conf_menu_stack_advanced_frame_system_entry_edgeThreshold;
    GtkWidget *conf_menu_stack_advanced_frame_system_entry_sigma;

    GtkWidget *conf_menu_stack_advanced_frame_adjustment;
    GtkWidget *conf_menu_stack_advanced_frame_adjustment_box;//val
    GtkWidget *conf_menu_stack_advanced_frame_system_entry_focal;
    GtkWidget *conf_menu_stack_advanced_frame_system_entry_lambda;

    //bas
    GtkWidget *conf_menu_stack_basic_scrolled_window;
    GtkWidget *conf_menu_stack_basic_viewport;
    GtkWidget *conf_menu_stack_basic_box;

    GtkWidget *conf_menu_stack_basic_frame_method;
    GtkWidget *conf_menu_stack_basic_frame_method_combo;
    GtkWidget *conf_menu_stack_basic_frame_method_box;

    GtkWidget *conf_menu_stack_basic_frame_method_switch_cut;
    GtkWidget *conf_menu_stack_basic_frame_method_switch_cut_yesno;
    GtkWidget *conf_menu_stack_basic_frame_method_switch_gain;
    GtkWidget *conf_menu_stack_basic_frame_method_entry_sigma;
    GtkWidget *conf_menu_stack_basic_frame_method_entry_bands;

    std::vector<GtkCellRenderer *> method_combo_renderer;
    pan::config* config_;
    std::filesystem::path _path_conf;


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
    GtkWidget *bar_edit_config;
    GtkWidget *bar_edit_separator;
    GtkWidget *bar_edit_submenu;
    GtkWidget *bar_edit_create;

    GtkWidget *bar_edit_cut;
    GtkWidget *bar_edit_select;
    GtkWidget *bar_edit_unselect;
    GtkWidget *bar_edit_order;

    struct config_ config;
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
    GtkToolItem *toolbar_seperator_0;

    GtkToolItem *toolbar_main_save;
    GtkWidget *toolbar_main_save_img;


    GtkToolItem *toolbar_main_undo_crop;
    GtkWidget *toolbar_main_undo_crop_img;

    GtkToolItem *toolbar_main_redo_crop;
    GtkWidget *toolbar_main_redo_crop_img;

    GtkToolItem *toolbar_main_crop;
    GtkWidget *toolbar_main_crop_img;


    GtkToolItem *toolbar_main_zin;
    GtkWidget *toolbar_main_zin_img;

    GtkToolItem *toolbar_main_zout;
    GtkWidget *toolbar_main_zout_img;

    std::vector<cv::Rect> crop_vect;

};


struct Drag_Par{
    gdouble start_x,start_y;
    gdouble hadj_value,vadj_value;
    gdouble dx, dy;
    gdouble offset_x = 0;//xy offset due to resizing
    gdouble offset_y = 0;
    int rel_x = 0; //xy relative to window
    int rel_y = 0;
    int abs_x = 0;//xy change when cut
    int abs_y = 0;
    cv::Rect save;

    gboolean is_dragging = FALSE;
    gboolean is_drawing = FALSE;
    gboolean is_config = FALSE;
    gboolean add_offset = FALSE;
    cv::Rect final_drawing;
};


struct viewer_window_{

    int windows_idx;

    GtkWidget *window;
    gint w_x,w_y;
    GtkWidget *viewer_box;
    GtkWidget *viewer_scrolled_window;
    GtkWidget *viewer_scrolled_window_viewpoint;
    GtkWidget *viewer_scrolled_window_viewpoint_overlay;
    GtkWidget *viewer_scrolled_window_viewpoint_drawing;

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
    struct progress_bar_* progress_bar;
    std::shared_ptr<pan::panorama> panorama_;

    cv::Rect crop_preview;
    std::vector<cv::Rect> crop_vec;
    int ret_counter;

};


struct main_window_{
    GtkWidget *window;
    pan::config* config_;
    GtkAccelGroup *window_accel_group;
    GtkWidget *box;
    GtkWidget *img_dragdrop;
    GtkWidget *img_dragdrop_frame;
    struct flowbox_ flowbox;
    struct menu_bar_ menu_bar;
    struct toolbar_ toolbar;

    std::map<int, struct viewer_window_> view;
    int view_number = 1;

    struct image_paths ipts;
    std::filesystem::path _path_ex;
    std::filesystem::path _path_css;
    std::filesystem::path _path_conf;

};




#endif
