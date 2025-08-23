
#ifndef PBAR_H
#define PBAR_H
#include "_gtk_vars.h"

namespace pbar{


struct progress_bar_{

    guint bar_timer_id = false;
    std::atomic<bool> cancel = false;
    std::atomic<double> fraction = .12;
    GtkWidget *window;
    GtkWidget *pbar_main_vbox;
    GtkWidget *pbar_progress_bar;
    GtkWidget *pbar_button_box;
    GtkWidget *pbar_button_box_cancel;

    gchar* loading_text;

    void bar_text(const gchar* source) {

        delete[] loading_text;
        loading_text = nullptr;

        if (source) {
            loading_text = new char[strlen(source) + 1];
            strcpy(loading_text, source);
        }
    }

    ~progress_bar_() {
        delete[] loading_text;
    }

};


gboolean update_progress(gpointer data);

void open_progress_bar(GtkWidget *add_to,struct progress_bar_ *progress_bar,struct main_window_ *main_window);


}
#endif

