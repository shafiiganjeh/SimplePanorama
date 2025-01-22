
#include "_main_windows.h"

void window_quit(GtkWidget *widget, gpointer data)
{
        gtk_main_quit();
}

void connect_signals(struct main_window_ *main_window){

        g_signal_connect(main_window->window, "destroy", G_CALLBACK(window_quit), NULL);
        g_signal_connect(main_window->img_dragdrop, "drag_data_received", G_CALLBACK(fbox::drag_rec), main_window);

}

void build_window(int argc, char** argv,struct main_window_ *main_window){
        gtk_init(&argc, &argv);

        GtkCssProvider *provider = gtk_css_provider_new();
        gtk_css_provider_load_from_path(provider,"/home/sd_bert/projects/Panorama/res/gtk.css",NULL);


        gtk_style_context_add_provider_for_screen(gdk_screen_get_default(),GTK_STYLE_PROVIDER(provider),GTK_STYLE_PROVIDER_PRIORITY_USER);

        main_window->window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

        gtk_window_set_default_size(GTK_WINDOW(main_window->window), 200, 200);

        main_window->box = gtk_box_new (GTK_ORIENTATION_VERTICAL,0);
        gtk_box_set_baseline_position (GTK_BOX(main_window->box),GTK_BASELINE_POSITION_CENTER);
        gtk_box_set_homogeneous (GTK_BOX(main_window->box),FALSE);
        gtk_container_add (GTK_CONTAINER(main_window->window),main_window->box);
        main_window->img_dragdrop = gtk_image_new_from_file ("drag.jpg");
        gtk_box_set_center_widget (GTK_BOX(main_window->box),main_window->img_dragdrop);
        gtk_drag_dest_set(GTK_WIDGET(main_window->img_dragdrop), GTK_DEST_DEFAULT_ALL, targetentries, 1, GDK_ACTION_COPY);

        cmenu::create_menu_bar(main_window->box,&(main_window->menu_bar),main_window);

        fbox::build_flowbox(main_window->box,&(main_window->flowbox),main_window);

        gtk_widget_show_all(main_window->window);
        gtk_widget_hide (GTK_WIDGET(main_window->flowbox.flowbox_scrolled_window));

        connect_signals(main_window);
}
