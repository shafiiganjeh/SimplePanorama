
#ifndef CONF_H
#define CONF_H
#include "_gtk_vars.h"
#include "_panorama.h"
#include "_config_parser.h"
#include "_util.h"

namespace conf{

    void open_conf_window(struct menu_bar_ *add_to,struct config_* config_window,struct main_window_ *main_window);

    gboolean new_pan(GtkWidget* self,GdkEventButton *event,struct main_window_ *main_window);

    void insert_text_event_float(GtkEditable *editable, const gchar *text, gint length, gint *position, gpointer data);

    void insert_text_event_int(GtkEditable *editable, const gchar *text, gint length, gint *position, gpointer data);

    void entry_settings(GtkWidget *row,GtkWidget *hbox,GtkWidget *label,GtkWidget * insert_to,GtkWidget * entry_address,const gchar* label_text);

}
#endif


