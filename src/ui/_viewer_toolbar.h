
#ifndef VIEWT_H
#define VIEWT_H

#include "_panorama.h"
#include "_gtk_vars.h"
#include "_gtk_ops.h"


namespace imgvt{


    void create_toolbar(GtkWidget *add_to,struct toolbar_viewer *toolbar,struct viewer_window_ *main_window);

    gboolean save_img(GtkWidget* self,GdkEventButton *event,struct viewer_window_ *main_window);

    gboolean on_draw(GtkWidget *widget, cairo_t *cr, gpointer data);

    void get_offset(struct viewer_window_ *viewer_window);

    void refresh_window(struct viewer_window_ *main_window);

}

#endif

