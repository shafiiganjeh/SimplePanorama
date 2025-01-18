
#include "Pcv2.h"

namespace ui {

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



void build_window(int argc, char** argv)
{
        gtk_init(&argc, &argv);
        gtk_var.window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
        gtk_window_set_default_size(GTK_WINDOW(gtk_var.window), 200, 200);

        gtk_var.box = gtk_box_new (GTK_ORIENTATION_VERTICAL,0);
        gtk_box_set_baseline_position (GTK_BOX(gtk_var.box),GTK_BASELINE_POSITION_CENTER);
        gtk_box_set_homogeneous (GTK_BOX(gtk_var.box),FALSE);
        gtk_container_add (GTK_CONTAINER(gtk_var.window),gtk_var.box);
        gtk_var.img_dragdrop = gtk_image_new_from_file ("/home/sd_bert/Desktop/cvproject/flowbox/build/image.jpg");
        gtk_box_set_center_widget (GTK_BOX(gtk_var.box),gtk_var.img_dragdrop);
        gtk_drag_dest_set(GTK_WIDGET(gtk_var.img_dragdrop), GTK_DEST_DEFAULT_ALL, targetentries, 1, GDK_ACTION_COPY);

        create_menu_bar();
        build_flowbox();

        gtk_widget_show_all(gtk_var.window);
        gtk_widget_hide (GTK_WIDGET(gtk_var.flowbox.flowbox_scrolled_window));
}
}
