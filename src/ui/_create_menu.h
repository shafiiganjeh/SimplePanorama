#ifndef CMENU_H
#define CMENU_H
#include "_gtk_vars.h"
#include "_flowbox.h"
#include "_toolbar.h"

namespace cmenu{

void create_menu_bar(GtkWidget *add_to,struct menu_bar_ *menu_bar,struct main_window_ *main_window);

void connect_signals(struct main_window_ *main_window);

}
#endif

