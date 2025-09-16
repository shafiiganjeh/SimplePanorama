
#include "_toolbar.h"

namespace tbar{

void cut_operation(GtkFlowBoxChild *child,struct main_window_ *main_window){

        GList *children = gtk_container_get_children(GTK_CONTAINER(child));
        gpointer widget = g_list_nth_data (children,0);

        int ind = gops::findStringIndex((main_window->ipts.f_list),gtk_widget_get_name (GTK_WIDGET( widget)));
        if (ind >= 0){
              gtk_widget_destroy(GTK_WIDGET(child));
              main_window->ipts.f_list.erase(main_window->ipts.f_list.begin() + ind);
              main_window->ipts.img_data.erase(main_window->ipts.img_data.begin() + ind);
              gtk_widget_show_all(main_window->flowbox.flowbox_main);
        }
        else{
               std::cout<< gtk_widget_get_name (GTK_WIDGET( widget))<<" not found";
        }

        g_list_free(g_steal_pointer (&children));
}

gboolean cut(GtkWidget* self,GdkEventButton *event,struct main_window_ *main_window){

        GList* s_list = gtk_flow_box_get_selected_children(GTK_FLOW_BOX( main_window->flowbox.flowbox_main));

        g_list_foreach (s_list,(GFunc)cut_operation,main_window);

        g_list_free(g_steal_pointer (&s_list));

        return FALSE;

}


gboolean new_pan(GtkWidget* self,GdkEventButton *event,struct main_window_ *main_window){


        GList* s_list = gtk_flow_box_get_selected_children(GTK_FLOW_BOX( main_window->flowbox.flowbox_main));
        guint size =g_list_length (s_list);

        if(size < 2){

                gchar message[] = "Select at least two images";
                dbox(message);
                g_list_free(g_steal_pointer (&s_list));

                return FALSE;
        }

        g_list_free(g_steal_pointer (&s_list));

        imgv::create_viewer(main_window,self,event);

        return FALSE;
}


gboolean _test_(GtkWidget* self,GdkEventButton *event,struct main_window_ *main_window){
        if(main_window->toolbar.tested == false){

        // Create the custom dialog window
        GtkWidget *dialog = gtk_window_new(GTK_WINDOW_TOPLEVEL);
        gtk_window_set_title(GTK_WINDOW(dialog), "Progress");
        gtk_window_set_default_size(GTK_WINDOW(dialog), 300, 100);
        gtk_window_set_resizable(GTK_WINDOW(dialog), FALSE);
        gtk_window_set_position(GTK_WINDOW(dialog), GTK_WIN_POS_CENTER_ON_PARENT);
        gtk_window_set_transient_for(GTK_WINDOW(dialog), GTK_WINDOW(main_window->window));

        // Create a main vertical box to hold all content
        GtkWidget *main_vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
        gtk_container_add(GTK_CONTAINER(dialog), main_vbox);
        gtk_widget_set_margin_top(main_vbox, 5);
        gtk_widget_set_margin_bottom(main_vbox, 5);
        gtk_widget_set_margin_start(main_vbox, 5);
        gtk_widget_set_margin_end(main_vbox, 5);

        // Create and add the progress bar
        GtkWidget *progress_bar = gtk_progress_bar_new();
        gtk_progress_bar_set_show_text(GTK_PROGRESS_BAR(progress_bar), TRUE);

        const gchar* frst = "text ";
        const gchar* scnd = "%d%%";
        gchar comb[strlen(frst) + strlen(scnd) + 1];

        strcpy(comb, frst);
        strcat(comb, scnd);

        gchar *text = g_strdup_printf(comb, 12);
        gtk_progress_bar_set_text(GTK_PROGRESS_BAR(progress_bar), text);
        gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(progress_bar), .5);
        gtk_box_pack_start(GTK_BOX(main_vbox), progress_bar, TRUE, TRUE, 0);

        // Create a horizontal box to center the OK button
        GtkWidget *button_hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
        gtk_box_pack_start(GTK_BOX(main_vbox), button_hbox, TRUE, TRUE, 10);

        // Create the OK button
        GtkWidget *ok_button = gtk_button_new_with_label("CANCEL");
        gtk_box_pack_start(GTK_BOX(button_hbox), ok_button, TRUE, FALSE, 0);

        // Connect the button click to close the dialog
        g_signal_connect_swapped(ok_button, "clicked", G_CALLBACK(gtk_widget_destroy), dialog);


                // Show all widgets
                gtk_widget_show_all(dialog);
        }

        return FALSE;
}


void connect_signals(struct main_window_ *main_window){

        g_signal_connect(main_window->toolbar.toolbar_main_cut, "button-release-event", G_CALLBACK(cut), main_window);
        g_signal_connect(main_window->toolbar.toolbar_main_new, "button-release-event", G_CALLBACK(new_pan), main_window);
        //g_signal_connect(main_window->toolbar.toolbar_main_testing, "button-release-event", G_CALLBACK(_test_), main_window);

}


void create_toolbar(GtkWidget *add_to,struct toolbar_ *toolbar,struct main_window_ *main_window){

        toolbar->toolbar_main = gtk_toolbar_new ();

        toolbar->toolbar_main_cut_img = gtk_image_new_from_icon_name ("edit-cut-symbolic",GTK_ICON_SIZE_SMALL_TOOLBAR );
        toolbar->toolbar_main_cut = gtk_tool_button_new (toolbar->toolbar_main_cut_img,"Cut");
        gtk_toolbar_insert(GTK_TOOLBAR(toolbar->toolbar_main),toolbar->toolbar_main_cut,0);
        gtk_widget_set_tooltip_text (GTK_WIDGET(toolbar->toolbar_main_cut),"Cut selection");

        toolbar->toolbar_main_new_img = gtk_image_new_from_icon_name ("document-new-symbolic",GTK_ICON_SIZE_SMALL_TOOLBAR );
        toolbar->toolbar_main_new = gtk_tool_button_new (toolbar->toolbar_main_new_img,"New");
        gtk_toolbar_insert(GTK_TOOLBAR(toolbar->toolbar_main),toolbar->toolbar_main_new,1);
        gtk_widget_set_tooltip_text (GTK_WIDGET(toolbar->toolbar_main_new),"Panorama from selection");

/*
        toolbar->toolbar_main_testing_img = gtk_image_new_from_icon_name ("system-run",GTK_ICON_SIZE_SMALL_TOOLBAR );
        toolbar->toolbar_main_testing = gtk_tool_button_new (toolbar->toolbar_main_testing_img,"Test");
        gtk_toolbar_insert(GTK_TOOLBAR(toolbar->toolbar_main),toolbar->toolbar_main_testing,2);
        gtk_widget_set_tooltip_text (GTK_WIDGET(toolbar->toolbar_main_testing),"This is a Test");
*/

        gtk_box_pack_end (GTK_BOX(add_to),toolbar->toolbar_main,FALSE,FALSE,0);


        connect_signals(main_window);
}

}

