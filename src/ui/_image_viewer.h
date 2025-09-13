
#ifndef VIEW_H
#define VIEW_H
#include "_panorama.h"
#include "_gtk_vars.h"
#include "_gtk_ops.h"
#include "_viewer_toolbar.h"
#include "_img_manipulation.h"

namespace imgv{

struct widget_and_Id{

    int id;
    struct main_window_ *window;

};

void window_quit(GtkWidget *widget, gpointer data);

gboolean update_progress(gpointer data);


gboolean show_image(struct progress_bar_ *progress_bar);


void create_viewer(struct main_window_ *main_window,GtkWidget* self,GdkEventButton *event);

void open_progress_bar(GtkWidget *add_to,struct progress_bar_ *progress_bar,struct main_window_ *main_window);

gboolean on_scroll(GtkWidget *widget,GdkEventScroll event,struct viewer_window_ *viewer_window);

void connect_signals(struct main_window_ *main_window,int id);




}

#endif

