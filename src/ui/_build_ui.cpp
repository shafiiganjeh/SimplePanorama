
#include "_build_ui.h"
namespace fs = std::filesystem;

GtkTargetEntry targetentries[] =
{
    { (char*)"text/uri-list", 0, 0}
};

namespace ui {

void create_menu_bar(){

        gtk_var.menu_bar.menu_box = gtk_box_new (GTK_ORIENTATION_HORIZONTAL,0);
        gtk_var.menu_bar.bar = gtk_menu_bar_new();
        gtk_var.menu_bar.toolbar = gtk_toolbar_new();

        gtk_var.menu_bar.bar_file = gtk_menu_item_new_with_label ("File");
        gtk_container_add(GTK_CONTAINER(gtk_var.menu_bar.bar),gtk_var.menu_bar.bar_file);
        gtk_var.menu_bar.bar_file_submenu = gtk_menu_new();
        gtk_menu_item_set_submenu(GTK_MENU_ITEM(gtk_var.menu_bar.bar_file),gtk_var.menu_bar.bar_file_submenu);


        gtk_var.menu_bar.bar_file_open = gtk_menu_item_new_with_label ("Open Folder");
        gtk_menu_shell_append(GTK_MENU_SHELL(gtk_var.menu_bar.bar_file_submenu),gtk_var.menu_bar.bar_file_open);

        gtk_var.menu_bar.bar_file_save = gtk_menu_item_new_with_label ("Save Panorama");
        gtk_widget_set_sensitive(GTK_WIDGET(gtk_var.menu_bar.bar_file_save),FALSE);
        gtk_menu_shell_append(GTK_MENU_SHELL(gtk_var.menu_bar.bar_file_submenu),gtk_var.menu_bar.bar_file_save);

        GtkWidget *seperator = gtk_separator_menu_item_new ();
        gtk_menu_shell_append(GTK_MENU_SHELL(gtk_var.menu_bar.bar_file_submenu),seperator);

        gtk_var.menu_bar.bar_file_quit = gtk_menu_item_new_with_label ("Quit");
        gtk_menu_shell_append(GTK_MENU_SHELL(gtk_var.menu_bar.bar_file_submenu),gtk_var.menu_bar.bar_file_quit);


        gtk_var.menu_bar.bar_edit = gtk_menu_item_new_with_label ("Edit");
        gtk_container_add(GTK_CONTAINER(gtk_var.menu_bar.bar),gtk_var.menu_bar.bar_edit);
        gtk_var.menu_bar.bar_edit_submenu = gtk_menu_new();
        gtk_menu_item_set_submenu(GTK_MENU_ITEM(gtk_var.menu_bar.bar_edit),gtk_var.menu_bar.bar_edit_submenu);
        gtk_widget_set_sensitive(GTK_WIDGET(gtk_var.menu_bar.bar_edit),FALSE);


        gtk_var.menu_bar.bar_edit_create = gtk_menu_item_new_with_label ("Create From Selection");
        gtk_menu_shell_append(GTK_MENU_SHELL(gtk_var.menu_bar.bar_edit_submenu),gtk_var.menu_bar.bar_edit_create);

        gtk_var.menu_bar.bar_edit_cut = gtk_menu_item_new_with_label ("Cut Selection");
        gtk_menu_shell_append(GTK_MENU_SHELL(gtk_var.menu_bar.bar_edit_submenu),gtk_var.menu_bar.bar_edit_cut);

        gtk_var.menu_bar.bar_edit_select = gtk_menu_item_new_with_label ("Select All");
        gtk_menu_shell_append(GTK_MENU_SHELL(gtk_var.menu_bar.bar_edit_submenu),gtk_var.menu_bar.bar_edit_select);

        gtk_var.menu_bar.bar_edit_unselect = gtk_menu_item_new_with_label ("Unselect All");
        gtk_menu_shell_append(GTK_MENU_SHELL(gtk_var.menu_bar.bar_edit_submenu),gtk_var.menu_bar.bar_edit_unselect);

        gtk_var.menu_bar.bar_edit_order = gtk_menu_item_new_with_label ("Order Selection");
        gtk_menu_shell_append(GTK_MENU_SHELL(gtk_var.menu_bar.bar_edit_submenu),gtk_var.menu_bar.bar_edit_order);


        gtk_box_pack_start (GTK_BOX(gtk_var.menu_bar.menu_box),gtk_var.menu_bar.bar,TRUE,TRUE,0);
        gtk_box_pack_end (GTK_BOX(gtk_var.menu_bar.menu_box),gtk_var.menu_bar.toolbar,FALSE,FALSE,0);
        gtk_container_add (GTK_CONTAINER(gtk_var.box),gtk_var.menu_bar.menu_box);

}

void build_flowbox(){
        gtk_var.flowbox.flowbox_main = gtk_flow_box_new();
        gtk_flow_box_set_selection_mode(GTK_FLOW_BOX(gtk_var.flowbox.flowbox_main),GTK_SELECTION_MULTIPLE);
        gtk_var.flowbox.flowbox_scrolled_window = gtk_scrolled_window_new (NULL, NULL);
        gtk_drag_dest_set(GTK_WIDGET(gtk_var.flowbox.flowbox_main), GTK_DEST_DEFAULT_ALL, targetentries, 1, GDK_ACTION_COPY);
        gtk_var.flowbox.flowbox_scrolled_window_viewpoint = gtk_viewport_new (NULL, NULL);

        gtk_container_add(GTK_CONTAINER(gtk_var.flowbox.flowbox_scrolled_window_viewpoint), gtk_var.flowbox.flowbox_main);
        gtk_container_add(GTK_CONTAINER(gtk_var.flowbox.flowbox_scrolled_window), gtk_var.flowbox.flowbox_scrolled_window_viewpoint);
        gtk_box_pack_end (GTK_BOX(gtk_var.box), gtk_var.flowbox.flowbox_scrolled_window,TRUE,TRUE,0);

        gtk_widget_set_visible(GTK_WIDGET(gtk_var.flowbox.flowbox_scrolled_window),FALSE);
        gtk_widget_set_sensitive(GTK_WIDGET(gtk_var.flowbox.flowbox_scrolled_window),FALSE);
}

void build_window(int argc, char** argv){
        gtk_init(&argc, &argv);
        gtk_var.window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
        gtk_window_set_default_size(GTK_WINDOW(gtk_var.window), 200, 200);

        gtk_var.box = gtk_box_new (GTK_ORIENTATION_VERTICAL,0);
        gtk_box_set_baseline_position (GTK_BOX(gtk_var.box),GTK_BASELINE_POSITION_CENTER);
        gtk_box_set_homogeneous (GTK_BOX(gtk_var.box),FALSE);
        gtk_container_add (GTK_CONTAINER(gtk_var.window),gtk_var.box);
        gtk_var.img_dragdrop = gtk_image_new_from_file ("drag.jpg");
        gtk_box_set_center_widget (GTK_BOX(gtk_var.box),gtk_var.img_dragdrop);
        gtk_drag_dest_set(GTK_WIDGET(gtk_var.img_dragdrop), GTK_DEST_DEFAULT_ALL, targetentries, 1, GDK_ACTION_COPY);

        create_menu_bar();
        build_flowbox();

        gtk_widget_show_all(gtk_var.window);
        gtk_widget_hide (GTK_WIDGET(gtk_var.flowbox.flowbox_scrolled_window));
}

void drag_rec(GtkWidget *widg, GdkDragContext* context, gint x,
                               gint y, GtkSelectionData* data, guint info, guint time,struct image_paths* file_pointer)
{
        int thumbnail_size = 250;

        std::vector<std::string> path_list = gops::get_path_from_Selection_data(file_pointer,data);

        int missing = gops::img_path_list_to_flowbox(path_list,file_pointer,thumbnail_size,gtk_var.flowbox);

        if ((file_pointer->f_list).size()>0){
            if (gtk_widget_get_visible(GTK_WIDGET(gtk_var.img_dragdrop))){
                gtk_widget_set_visible(GTK_WIDGET(gtk_var.img_dragdrop),FALSE);
                gtk_widget_set_sensitive(GTK_WIDGET(gtk_var.img_dragdrop),FALSE);
        }

            if (!gtk_widget_get_visible(GTK_WIDGET(gtk_var.flowbox.flowbox_scrolled_window))){
                gtk_widget_set_visible(GTK_WIDGET(gtk_var.flowbox.flowbox_scrolled_window),TRUE);
                gtk_widget_set_sensitive(GTK_WIDGET(gtk_var.flowbox.flowbox_scrolled_window),TRUE);
                gtk_widget_set_sensitive(GTK_WIDGET(gtk_var.menu_bar.bar_edit),TRUE);
        }}

}

void destroy(GtkWidget *widget, gpointer data)
{
        gtk_main_quit();
}

gboolean open_folder(GtkMenuItem* *widget,struct image_paths* file_pointer)
{
        int thumbnail_size = 250;

        GtkFileChooserNative *native;
        GtkFileChooserAction action = GTK_FILE_CHOOSER_ACTION_SELECT_FOLDER;
        gint res;

        native = gtk_file_chooser_native_new ("Open File",GTK_WINDOW(gtk_var.window),action,"_Open","_Cancel");

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

                if (std::find((file_pointer -> f_list).begin(), (file_pointer -> f_list).end(), temp_string) == (file_pointer -> f_list).end())
                    {
                        path_list.push_back(temp_string);
                    }

            }

            int missing = gops::img_path_list_to_flowbox(path_list,file_pointer,thumbnail_size,gtk_var.flowbox);

        }

            if ((file_pointer->f_list).size()>0){
                if (gtk_widget_get_visible(GTK_WIDGET(gtk_var.img_dragdrop))){
                    gtk_widget_set_visible(GTK_WIDGET(gtk_var.img_dragdrop),FALSE);
                    gtk_widget_set_sensitive(GTK_WIDGET(gtk_var.img_dragdrop),FALSE);
            }

                if (!gtk_widget_get_visible(GTK_WIDGET(gtk_var.flowbox.flowbox_scrolled_window))){
                    gtk_widget_set_visible(GTK_WIDGET(gtk_var.flowbox.flowbox_scrolled_window),TRUE);
                    gtk_widget_set_sensitive(GTK_WIDGET(gtk_var.flowbox.flowbox_scrolled_window),TRUE);
                    gtk_widget_set_sensitive(GTK_WIDGET(gtk_var.menu_bar.bar_edit),TRUE);
            }}

        g_object_unref (native);
        return TRUE;
}

gboolean unselect_all(GtkMenuItem* *widget,struct image_paths* file_pointer){
        gtk_flow_box_unselect_all (GTK_FLOW_BOX(gtk_var.flowbox.flowbox_main));
        return FALSE;

}

gboolean select_all(GtkMenuItem* *widget,struct image_paths* file_pointer){
        gtk_flow_box_select_all (GTK_FLOW_BOX(gtk_var.flowbox.flowbox_main));
        return FALSE;

}

void connect_signals(struct image_paths* ipts){

        g_signal_connect(gtk_var.window, "destroy", G_CALLBACK(destroy), NULL);

        g_signal_connect(gtk_var.img_dragdrop, "drag_data_received", G_CALLBACK(drag_rec), ipts);
        g_signal_connect(gtk_var.flowbox.flowbox_main, "drag_data_received", G_CALLBACK(drag_rec), &ipts);


        g_signal_connect(gtk_var.menu_bar.bar_file_quit, "button_press_event", G_CALLBACK(destroy), NULL);
        g_signal_connect(gtk_var.menu_bar.bar_file_open, "activate", G_CALLBACK(open_folder), &ipts);
        g_signal_connect(gtk_var.menu_bar.bar_edit_unselect, "activate", G_CALLBACK(unselect_all), NULL);
        g_signal_connect(gtk_var.menu_bar.bar_edit_select, "activate", G_CALLBACK(select_all), NULL);



}

}

