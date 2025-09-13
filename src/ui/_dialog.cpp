
#include "_dialog.h"

static void close_dialog(GtkWidget *widget, gpointer data) {
    gtk_widget_destroy(GTK_WIDGET(data));
}

void dbox(gchar *message) {
    GtkWidget *dialog, *content_area, *label, *button;

    dialog = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(dialog), "Warning!");
    gtk_window_set_modal(GTK_WINDOW(dialog), TRUE);
    gtk_window_set_position(GTK_WINDOW(dialog), GTK_WIN_POS_CENTER);
    gtk_container_set_border_width(GTK_CONTAINER(dialog), 10);
    gtk_window_set_default_size(GTK_WINDOW(dialog), 300, 90);
    gtk_window_set_resizable(GTK_WINDOW(dialog), FALSE);

    content_area = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_container_add(GTK_CONTAINER(dialog), content_area);

    label = gtk_label_new(message);
    gtk_label_set_line_wrap(GTK_LABEL(label), TRUE);
    gtk_label_set_max_width_chars(GTK_LABEL(label), 40);
    gtk_box_pack_start(GTK_BOX(content_area), label, TRUE, TRUE, 0);

    button = gtk_button_new_with_label("OK");
    gtk_widget_set_size_request(button, 130, 20);
    gtk_widget_set_margin_start(button, 85);
    gtk_widget_set_margin_end(button, 85);
    gtk_box_pack_start(GTK_BOX(content_area), button, FALSE, FALSE, 0);


    g_signal_connect(button, "clicked", G_CALLBACK(close_dialog), dialog);


    gtk_widget_show_all(dialog);
}
