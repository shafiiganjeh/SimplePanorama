
#include "_create_menu.h"

namespace fs = std::filesystem;

namespace cmenu{

void window_quit(GtkWidget *widget,struct main_window_ *main_window)
{
        gtk_main_quit();
}


gboolean open_folder(GtkMenuItem* *widget,struct main_window_ *main_window)
{
        int thumbnail_size = 250;

        GtkFileChooserNative *native;
        GtkFileChooserAction action = GTK_FILE_CHOOSER_ACTION_SELECT_FOLDER;
        gint res;

        native = gtk_file_chooser_native_new ("Open File",GTK_WINDOW(main_window->window),action,"_Open","_Cancel");

        res = gtk_native_dialog_run (GTK_NATIVE_DIALOG (native));
        if (res == GTK_RESPONSE_ACCEPT)
        {
            char *path;
            GtkFileChooser *chooser = GTK_FILE_CHOOSER (native);
            path = gtk_file_chooser_get_filename (chooser);
            std::string str(path);

            std::vector<std::string> path_list;

            for (const auto & entry : fs::directory_iterator(path)){
                std::string temp_string = entry.path().string();

                if (std::find((main_window->ipts.f_list).begin(), (main_window->ipts.f_list).end(), temp_string) == (main_window->ipts.f_list).end())
                    {
                        path_list.push_back(temp_string);
                    }

            }


            int missing = fbox::img_path_list_to_flowbox(path_list,thumbnail_size,main_window);

        }

            if ((main_window->ipts.f_list).size()>0){
                if (gtk_widget_get_visible(GTK_WIDGET(main_window->img_dragdrop))){
                    gtk_widget_set_visible(GTK_WIDGET(main_window->img_dragdrop),FALSE);
                    gtk_widget_set_sensitive(GTK_WIDGET(main_window->img_dragdrop),FALSE);
            }

                if (!gtk_widget_get_visible(GTK_WIDGET(main_window->flowbox.flowbox_scrolled_window))){
                    gtk_widget_set_visible(GTK_WIDGET(main_window->flowbox.flowbox_scrolled_window),TRUE);
                    gtk_widget_set_sensitive(GTK_WIDGET(main_window->flowbox.flowbox_scrolled_window),TRUE);
                    gtk_widget_set_sensitive(GTK_WIDGET(main_window->menu_bar.bar_edit),TRUE);
            }}

        g_object_unref (native);
        return FALSE;
}


gboolean unselect_all(GtkMenuItem* *widget,struct main_window_ *main_window){
        gtk_flow_box_unselect_all (GTK_FLOW_BOX(main_window->flowbox.flowbox_main));
        return FALSE;

}

gboolean select_all(GtkMenuItem* *widget,struct main_window_ *main_window){
        gtk_flow_box_select_all (GTK_FLOW_BOX(main_window->flowbox.flowbox_main));
        return FALSE;

}


gboolean cut(GtkMenuItem* *widget,struct main_window_ *main_window){
        GList* s_list = gtk_flow_box_get_selected_children(GTK_FLOW_BOX( main_window->flowbox.flowbox_main));

        g_list_foreach (s_list,(GFunc)gops::cut_operation,main_window);

        g_list_free(g_steal_pointer (&s_list));
        return FALSE;

}

gboolean open_config(GtkMenuItem* *widget,struct main_window_ *main_window){

        conf::open_conf_window(&main_window->menu_bar,&(main_window->menu_bar.config),main_window);

        return FALSE;
}


void connect_signals(struct main_window_ *main_window){

        g_signal_connect(main_window->menu_bar.bar_file_quit, "activate", G_CALLBACK(window_quit), main_window);
        g_signal_connect(main_window->menu_bar.bar_file_open, "activate", G_CALLBACK(open_folder), main_window);
        g_signal_connect(main_window->menu_bar.bar_edit_unselect, "activate", G_CALLBACK(unselect_all), main_window);
        g_signal_connect(main_window->menu_bar.bar_edit_select, "activate", G_CALLBACK(select_all), main_window);
        g_signal_connect(main_window->menu_bar.bar_edit_cut, "activate", G_CALLBACK(cut), main_window);
        g_signal_connect(main_window->menu_bar.bar_edit_config, "activate", G_CALLBACK(open_config), main_window);
}


void create_menu_bar(GtkWidget *add_to,struct menu_bar_ *menu_bar,struct main_window_ *main_window){


        menu_bar->menu_box = gtk_box_new (GTK_ORIENTATION_HORIZONTAL,0);
        menu_bar->bar = gtk_menu_bar_new();
        //menu_bar->toolbar = gtk_toolbar_new();

        menu_bar->bar_file = gtk_menu_item_new_with_label ("File");
        gtk_container_add(GTK_CONTAINER(menu_bar->bar),menu_bar->bar_file);
        menu_bar->bar_file_submenu = gtk_menu_new();
        gtk_menu_item_set_submenu(GTK_MENU_ITEM(menu_bar->bar_file),menu_bar->bar_file_submenu);


        menu_bar->bar_file_open = gtk_menu_item_new_with_label ("Open Folder");
        gtk_menu_shell_append(GTK_MENU_SHELL(menu_bar->bar_file_submenu),menu_bar->bar_file_open);

        menu_bar->bar_file_save = gtk_menu_item_new_with_label ("Save Panorama");
        gtk_widget_set_sensitive(GTK_WIDGET(menu_bar->bar_file_save),FALSE);
        gtk_menu_shell_append(GTK_MENU_SHELL(menu_bar->bar_file_submenu),menu_bar->bar_file_save);

        GtkWidget *seperator = gtk_separator_menu_item_new ();
        gtk_menu_shell_append(GTK_MENU_SHELL(menu_bar->bar_file_submenu),seperator);

        menu_bar->bar_file_quit = gtk_menu_item_new_with_label ("Quit");
        gtk_menu_shell_append(GTK_MENU_SHELL(menu_bar->bar_file_submenu),menu_bar->bar_file_quit);
        gtk_widget_add_accelerator(menu_bar->bar_file_quit, "activate", main_window->window_accel_group,
                        GDK_KEY_q,
                        GDK_CONTROL_MASK,
                        GTK_ACCEL_VISIBLE);


        menu_bar->bar_edit = gtk_menu_item_new_with_label ("Edit");
        gtk_container_add(GTK_CONTAINER(menu_bar->bar),menu_bar->bar_edit);
        menu_bar->bar_edit_submenu = gtk_menu_new();
        gtk_menu_item_set_submenu(GTK_MENU_ITEM(menu_bar->bar_edit),menu_bar->bar_edit_submenu);
        //gtk_widget_set_sensitive(GTK_WIDGET(menu_bar->bar_edit),FALSE);


        menu_bar->bar_edit_create = gtk_menu_item_new_with_label ("Panorama From Selection");
        gtk_menu_shell_append(GTK_MENU_SHELL(menu_bar->bar_edit_submenu),menu_bar->bar_edit_create);

        menu_bar->bar_edit_cut = gtk_menu_item_new_with_label ("Cut Selection");
        gtk_menu_shell_append(GTK_MENU_SHELL(menu_bar->bar_edit_submenu),menu_bar->bar_edit_cut);
        gtk_widget_add_accelerator(menu_bar->bar_edit_cut, "activate", main_window->window_accel_group,
                        GDK_KEY_x,
                        GDK_CONTROL_MASK,
                        GTK_ACCEL_VISIBLE);


        menu_bar->bar_edit_select = gtk_menu_item_new_with_label ("Select All");
        gtk_menu_shell_append(GTK_MENU_SHELL(menu_bar->bar_edit_submenu),menu_bar->bar_edit_select);
        gtk_widget_add_accelerator(menu_bar->bar_edit_select, "activate", main_window->window_accel_group,
                        GDK_KEY_a,
                        GDK_CONTROL_MASK,
                        GTK_ACCEL_VISIBLE);

        menu_bar->bar_edit_unselect = gtk_menu_item_new_with_label ("Unselect All");
        gtk_menu_shell_append(GTK_MENU_SHELL(menu_bar->bar_edit_submenu),menu_bar->bar_edit_unselect);
        gtk_widget_add_accelerator(menu_bar->bar_edit_unselect, "activate", main_window->window_accel_group,
                        GDK_KEY_a,
                        static_cast<GdkModifierType>(GDK_CONTROL_MASK | GDK_SHIFT_MASK),
                        GTK_ACCEL_VISIBLE);

        menu_bar->bar_edit_separator = gtk_separator_menu_item_new ();
        gtk_menu_shell_append(GTK_MENU_SHELL(menu_bar->bar_edit_submenu),menu_bar->bar_edit_separator);

        menu_bar->bar_edit_config = gtk_menu_item_new_with_label ("Config");
        gtk_menu_shell_append(GTK_MENU_SHELL(menu_bar->bar_edit_submenu),menu_bar->bar_edit_config);


        gtk_box_pack_start (GTK_BOX(menu_bar->menu_box),menu_bar->bar,TRUE,TRUE,0);
        //gtk_box_pack_end (GTK_BOX(menu_bar->menu_box),menu_bar->toolbar,FALSE,FALSE,0);
        gtk_container_add (GTK_CONTAINER(add_to),menu_bar->menu_box);

        tbar::create_toolbar(menu_bar->menu_box,&(main_window->toolbar),main_window);

        connect_signals(main_window);
}

}
