
#ifndef VIEW_H
#define VIEW_H
#include "_gtk_vars.h"
#include "_gtk_ops.h"
#include "_viewer_toolbar.h"
#include "_img_manipulation.h"


namespace imgv{

struct widget_and_Id{

    int id;
    struct main_window_ *window;

};


void create_viewer(struct main_window_ *main_window,GtkWidget* self,GdkEventButton *event);


}

#endif

