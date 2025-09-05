
#ifndef MWIND_H
#define MWIND_H
#include "_gtk_vars.h"
#include "_flowbox.h"
#include "_create_menu.h"
#include "_toolbar.h"
#include "_image_viewer.h"
#include "_panorama.h"

void build_window(int argc, char** argv,struct main_window_ *main_window,struct pan::config* conf);

static void window_quit(GtkWidget *widget, gpointer data);

void connect_signals(struct main_window_ *main_window);

#endif
