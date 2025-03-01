#include "_viewer_toolbar.h"

namespace imgvt{


    gboolean zin(GtkWidget* self,GdkEventButton *event,struct viewer_window_ *main_window){

        if(main_window->current_zoom < main_window->zoom_val.size() - 1){

            main_window->current_zoom++;
            main_window->image_zoomed = imgm::resizeKeepAspectRatio(main_window->image, main_window->zoom_val[main_window->current_zoom]);

            gtk_widget_destroy (GTK_WIDGET(main_window->img_test));
            main_window->img_test = gops::cv_image_to_gtk_image(main_window->image_zoomed);
            gtk_container_add (GTK_CONTAINER(main_window->viewer_scrolled_window_viewpoint),GTK_WIDGET(main_window->img_test));
            gtk_widget_show_all(main_window->window);

        }


        return FALSE;

}


    gboolean zout(GtkWidget* self,GdkEventButton *event,struct viewer_window_ *main_window){

        if(main_window->current_zoom > 0){

            main_window->current_zoom--;
            main_window->image_zoomed = imgm::resizeKeepAspectRatio(main_window->image, main_window->zoom_val[main_window->current_zoom]);

            gtk_widget_destroy (GTK_WIDGET(main_window->img_test));
            main_window->img_test = gops::cv_image_to_gtk_image(main_window->image_zoomed);
            gtk_container_add (GTK_CONTAINER(main_window->viewer_scrolled_window_viewpoint),GTK_WIDGET(main_window->img_test));
            gtk_widget_show_all(main_window->window);

        }


        return FALSE;

}


    void connect_signals(struct viewer_window_ *main_window){

        g_signal_connect(main_window->toolbar.toolbar_main_zin, "button-release-event", G_CALLBACK(zin), main_window);
        g_signal_connect(main_window->toolbar.toolbar_main_zout, "button-release-event", G_CALLBACK(zout), main_window);


}


    void create_toolbar(GtkWidget *add_to,struct toolbar_viewer *toolbar,struct viewer_window_ *main_window){

        toolbar->toolbar_main = gtk_toolbar_new ();

        toolbar->toolbar_main_save_img = gtk_image_new_from_icon_name ("document-save",GTK_ICON_SIZE_SMALL_TOOLBAR );
        toolbar->toolbar_main_save = gtk_tool_button_new (toolbar->toolbar_main_save_img,"save");
        gtk_toolbar_insert(GTK_TOOLBAR(toolbar->toolbar_main),toolbar->toolbar_main_save,-1);
        gtk_widget_set_tooltip_text (GTK_WIDGET(toolbar->toolbar_main_save),"Save Panorama");

        toolbar->toolbar_main_zin_img = gtk_image_new_from_icon_name ("zoom-in",GTK_ICON_SIZE_SMALL_TOOLBAR );
        toolbar->toolbar_main_zin = gtk_tool_button_new (toolbar->toolbar_main_zin_img,"Zoom In");
        gtk_toolbar_insert(GTK_TOOLBAR(toolbar->toolbar_main),toolbar->toolbar_main_zin,-1);
        gtk_widget_set_tooltip_text (GTK_WIDGET(toolbar->toolbar_main_zin),"Zoom In");

        toolbar->toolbar_main_zout_img = gtk_image_new_from_icon_name ("zoom-out",GTK_ICON_SIZE_SMALL_TOOLBAR );
        toolbar->toolbar_main_zout = gtk_tool_button_new (toolbar->toolbar_main_zout_img,"Zoom Out");
        gtk_toolbar_insert(GTK_TOOLBAR(toolbar->toolbar_main),toolbar->toolbar_main_zout,-1);
        gtk_widget_set_tooltip_text (GTK_WIDGET(toolbar->toolbar_main_zout),"Zoom Out");


        if(main_window->current_zoom == -1){

            gtk_widget_set_sensitive(GTK_WIDGET(toolbar->toolbar_main_zin),FALSE);
            gtk_widget_set_sensitive(GTK_WIDGET(toolbar->toolbar_main_zout),FALSE);

        }


        gtk_box_pack_end (GTK_BOX(add_to),toolbar->toolbar_main,FALSE,FALSE,0);



        connect_signals(main_window);

    }


}



