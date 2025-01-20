
#ifndef BUILDUI_H
#define BUILDUI_H

#include "_gtk_vars.h"
#include "_gtk_ops.h"

namespace ui {


void build_flowbox();

void create_menu_bar();

void  build_window(int argc, char** argv);

void drag_rec(GtkWidget *widg, GdkDragContext* context, gint x,
                               gint y, GtkSelectionData* data, guint info, guint time,struct image_paths* file_pointer);

void connect_signals(struct image_paths* ipts);

static void destroy(GtkWidget *widget, gpointer data);

gboolean open_folder(GtkMenuItem* *widget,struct image_paths* file_pointer);

}
#endif
