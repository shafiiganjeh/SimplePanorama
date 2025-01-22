
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

void connect_signals(struct main_window_ *main_window){

        g_signal_connect(main_window->toolbar.toolbar_main_cut, "button-release-event", G_CALLBACK(cut), main_window);

}


void create_toolbar(GtkWidget *add_to,struct toolbar_ *toolbar,struct main_window_ *main_window){

        toolbar->toolbar_main = gtk_toolbar_new ();

        toolbar->toolbar_main_cut_img = gtk_image_new_from_icon_name ("edit-cut",GTK_ICON_SIZE_SMALL_TOOLBAR );
        toolbar->toolbar_main_cut = gtk_tool_button_new (toolbar->toolbar_main_cut_img,"Cut");
        gtk_toolbar_insert(GTK_TOOLBAR(toolbar->toolbar_main),toolbar->toolbar_main_cut,0);
        gtk_widget_set_tooltip_text (GTK_WIDGET(toolbar->toolbar_main_cut),"Cut selection");

        gtk_box_pack_end (GTK_BOX(add_to),toolbar->toolbar_main,FALSE,FALSE,0);


        connect_signals(main_window);
}

}

