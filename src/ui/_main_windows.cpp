#include "_main_windows.h"

const gchar* ret_path8(std::filesystem::path& path);

void window_quit(GtkWidget *widget, gpointer data)
{
        gtk_main_quit();
}

void connect_signals(struct main_window_ *main_window){

        g_signal_connect(main_window->window, "destroy", G_CALLBACK(window_quit), NULL);
        g_signal_connect(main_window->img_dragdrop, "drag_data_received", G_CALLBACK(fbox::drag_rec), main_window);

}

void build_window(int argc, char** argv,struct main_window_ *main_window,struct pan::config* conf){
        gtk_init(&argc, &argv);


        #ifdef __linux__


        GtkCssProvider *provider = gtk_css_provider_new();
        const gchar* css_path = ret_path8(main_window->_path_css);
        gtk_css_provider_load_from_path(provider,css_path,NULL);
        g_free((gpointer)css_path);
        gtk_style_context_add_provider_for_screen(gdk_screen_get_default(),GTK_STYLE_PROVIDER(provider),GTK_STYLE_PROVIDER_PRIORITY_USER);

        #endif


        main_window->config_ = conf;
        main_window->window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
        main_window->window_accel_group = gtk_accel_group_new();
        gtk_window_add_accel_group(GTK_WINDOW(main_window->window), main_window->window_accel_group);

        gtk_window_set_default_size(GTK_WINDOW(main_window->window), 200, 200);

        main_window->box = gtk_box_new (GTK_ORIENTATION_VERTICAL,0);
        gtk_box_set_baseline_position (GTK_BOX(main_window->box),GTK_BASELINE_POSITION_CENTER);
        gtk_box_set_homogeneous (GTK_BOX(main_window->box),FALSE);
        gtk_container_add (GTK_CONTAINER(main_window->window),main_window->box);
        std::filesystem::path drag_path = main_window->_path_ex;
        drag_path /= "res";drag_path /= "drag.png";
        const gchar* drg_path = ret_path8(drag_path);
        main_window->img_dragdrop = gtk_image_new_from_file (drg_path);
        g_free((gpointer)drg_path);
        gtk_box_set_center_widget (GTK_BOX(main_window->box),main_window->img_dragdrop);
        gtk_drag_dest_set(GTK_WIDGET(main_window->img_dragdrop), GTK_DEST_DEFAULT_ALL, targetentries, 1, GDK_ACTION_COPY);

        cmenu::create_menu_bar(main_window->box,&(main_window->menu_bar),main_window);
        fbox::build_flowbox(main_window->box,&(main_window->flowbox),main_window);
        gtk_widget_show_all(main_window->window);
        connect_signals(main_window);

        gtk_widget_hide (GTK_WIDGET(main_window->flowbox.flowbox_scrolled_window));


}

#ifdef __linux__

const gchar* ret_path8(std::filesystem::path& path){

    return path.c_str();

}

#elif defined(_WIN32)

const gchar* ret_path8(std::filesystem::path& path){

    const std::wstring wpath = path.wstring();
    return g_utf16_to_utf8((const gunichar2*)wpath.c_str(), -1, NULL, NULL, NULL);

}
#endif
