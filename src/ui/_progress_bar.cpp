
#include "_progress_bar.h"

namespace pbar{



    gboolean update_progress(gpointer data) {

        progress_bar_ *progress_bar = static_cast<progress_bar_*>(data);
        if(progress_bar->cancel) {return G_SOURCE_REMOVE;}

        gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(progress_bar->pbar_progress_bar), .5);

        const gchar* number = "%d%%";
        gchar* temp = new gchar[strlen(number) + strlen(progress_bar->loading_text) + 1];;
        strcpy(temp, progress_bar->loading_text);
        strcat(temp, number);

        gchar *text = g_strdup_printf(temp, (int)(progress_bar->fraction * 100));
        gtk_progress_bar_set_text(GTK_PROGRESS_BAR(progress_bar->pbar_progress_bar), text);

        g_free(text);
        delete[] temp;

        if (progress_bar->fraction >= 1.0) {

            gtk_widget_destroy(progress_bar->window);

            return G_SOURCE_REMOVE;
        }
        return G_SOURCE_CONTINUE;
    }


    void window_closed(GtkWidget *widget, progress_bar_ *progress_bar){

        progress_bar->cancel = true;

    }


    void connect_signals(progress_bar_ *progress_bar){

        g_signal_connect(progress_bar->window, "destroy", G_CALLBACK(window_closed), progress_bar);
        g_signal_connect(progress_bar->pbar_button_box_cancel, "clicked", G_CALLBACK(gtk_window_close), GTK_WINDOW(progress_bar->window));

    }


    void open_progress_bar(GtkWidget *add_to,progress_bar_ *progress_bar,struct main_window_ *main_window){


        progress_bar->window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
        gtk_window_set_title(GTK_WINDOW(progress_bar->window), "Working...");
        gtk_window_set_default_size(GTK_WINDOW(progress_bar->window), 300, 100);
        gtk_window_set_resizable(GTK_WINDOW(progress_bar->window), FALSE);
        gtk_window_set_position(GTK_WINDOW(progress_bar->window), GTK_WIN_POS_CENTER_ON_PARENT);
        gtk_window_set_transient_for(GTK_WINDOW(progress_bar->window), GTK_WINDOW(main_window->window));

        progress_bar->pbar_main_vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
        gtk_container_add(GTK_CONTAINER(progress_bar->window), progress_bar->pbar_main_vbox);
        gtk_widget_set_margin_top(progress_bar->pbar_main_vbox, 5);
        gtk_widget_set_margin_bottom(progress_bar->pbar_main_vbox, 5);
        gtk_widget_set_margin_start(progress_bar->pbar_main_vbox, 5);
        gtk_widget_set_margin_end(progress_bar->pbar_main_vbox, 5);

        progress_bar->pbar_progress_bar = gtk_progress_bar_new();
        gtk_progress_bar_set_show_text(GTK_PROGRESS_BAR(progress_bar->pbar_progress_bar), TRUE);
        gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(progress_bar->pbar_progress_bar), 0);
        gtk_box_pack_start(GTK_BOX(progress_bar->pbar_main_vbox), progress_bar->pbar_progress_bar, TRUE, TRUE, 0);

        progress_bar->pbar_button_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
        gtk_box_pack_start(GTK_BOX(progress_bar->pbar_main_vbox), progress_bar->pbar_button_box, FALSE, FALSE, 10);

        progress_bar->pbar_button_box_cancel = gtk_button_new_with_label("CANCEL");
        gtk_box_pack_end(GTK_BOX(progress_bar->pbar_button_box), progress_bar->pbar_button_box_cancel, TRUE, FALSE, 0);

        progress_bar->bar_text("sample text: ");

        connect_signals(progress_bar);
        gtk_widget_show_all(progress_bar->window);

    }

}
