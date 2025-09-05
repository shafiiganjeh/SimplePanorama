
#include "_config.h"

namespace conf{


    void conf_blending_settings(struct config_* config_window){

        config_window->conf_menu_stack_basic_scrolled_window = gtk_scrolled_window_new(NULL, NULL);
        gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(config_window->conf_menu_stack_basic_scrolled_window),
                                    GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
        config_window->conf_menu_stack_basic_viewport = gtk_viewport_new(NULL, NULL);
        gtk_container_add(GTK_CONTAINER(config_window->conf_menu_stack_basic_scrolled_window),
                        config_window->conf_menu_stack_basic_viewport);
        config_window->conf_menu_stack_basic_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
        gtk_container_add(GTK_CONTAINER(config_window->conf_menu_stack_basic_viewport),
                        config_window->conf_menu_stack_basic_box);
        gtk_stack_add_titled(GTK_STACK(config_window->conf_menu_stack),
                        config_window->conf_menu_stack_basic_scrolled_window,
                        "blending_", "Blending");

        config_window->conf_menu_stack_basic_box_method = gtk_frame_new ("Blending Method");
        gtk_box_pack_start (GTK_BOX(config_window->conf_menu_stack_basic_box),config_window->conf_menu_stack_basic_box_method,FALSE,FALSE,5);

        config_window->conf_menu_stack_basic_box_method_choose = gtk_combo_box_new ();



    }

    void conf_advanced_settings(struct config_* config_window){

        config_window->conf_menu_stack_advanced_scrolled_window = gtk_scrolled_window_new(NULL, NULL);
        gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(config_window->conf_menu_stack_advanced_scrolled_window),
                                    GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
        config_window->conf_menu_stack_advanced_viewport = gtk_viewport_new(NULL, NULL);
        gtk_container_add(GTK_CONTAINER(config_window->conf_menu_stack_advanced_scrolled_window),
                        config_window->conf_menu_stack_advanced_viewport);
        config_window->conf_menu_stack_advanced_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
        gtk_container_add(GTK_CONTAINER(config_window->conf_menu_stack_advanced_viewport),
                        config_window->conf_menu_stack_advanced_box);
        gtk_stack_add_titled(GTK_STACK(config_window->conf_menu_stack),
                        config_window->conf_menu_stack_advanced_scrolled_window,
                        "advanced_settings", "Advanced Settings");

    }



    void open_conf_window(struct menu_bar_ *add_to,struct config_* config_window,struct main_window_ *main_window){

    config_window->conf_menu = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_default_size(GTK_WINDOW(config_window->conf_menu), 800, 640);
    gtk_window_set_title(GTK_WINDOW(config_window->conf_menu), "Settings");

    config_window->conf_menu_paned = gtk_paned_new(GTK_ORIENTATION_HORIZONTAL);
    gtk_paned_set_wide_handle(GTK_PANED(config_window->conf_menu_paned), true);
    gtk_container_add(GTK_CONTAINER(config_window->conf_menu), config_window->conf_menu_paned);

    // sidebar
    config_window->conf_menu_paned_sidebar_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    config_window->conf_menu_paned_sidebar = gtk_stack_switcher_new();
    gtk_orientable_set_orientation(GTK_ORIENTABLE(config_window->conf_menu_paned_sidebar), GTK_ORIENTATION_VERTICAL);
    gtk_box_pack_start(GTK_BOX(config_window->conf_menu_paned_sidebar_box), config_window->conf_menu_paned_sidebar, FALSE, FALSE, 0);
    gtk_paned_add1(GTK_PANED(config_window->conf_menu_paned), config_window->conf_menu_paned_sidebar_box);



    config_window->conf_menu_stack = gtk_stack_new();
    gtk_stack_set_transition_type(GTK_STACK(config_window->conf_menu_stack), GTK_STACK_TRANSITION_TYPE_SLIDE_LEFT_RIGHT);
    gtk_paned_add2(GTK_PANED(config_window->conf_menu_paned), config_window->conf_menu_stack);
    gtk_stack_switcher_set_stack(GTK_STACK_SWITCHER(config_window->conf_menu_paned_sidebar), GTK_STACK(config_window->conf_menu_stack));



    conf_blending_settings(config_window);
    conf_advanced_settings(config_window);
    gtk_widget_show_all(config_window->conf_menu);

    }

}

