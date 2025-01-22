#ifndef FBOX_H
#define FBOX_H
#include "_gtk_vars.h"
#include "_gtk_ops.h"
#include <typeinfo>


namespace fbox{

void build_flowbox(GtkWidget *add_to,struct flowbox_ *flowbox,struct main_window_ *main_window);

void connect_signals(struct flowbox_ *flowbox,struct main_window_ *main_window);

int img_path_list_to_flowbox(std::vector<std::string> &path_list,int thumbnail_size,struct main_window_ *main_window,int index = 0);

void add_image_to_flowbox(struct flowbox_ *flowbox, GtkImage* image,struct main_window_ *main_window,std::string labelstr = "",std::string widget_name = "",int index = 0,int thumbnail_size = 250,int font_size = 12);

gboolean flowbox_item_clicked(GtkWidget* self,GdkEventButton *event,gpointer _);

void drag_rec(GtkWidget *widg, GdkDragContext* context, gint x,
                               gint y, GtkSelectionData* data, guint info, guint time,struct main_window_ *main_window);

}
#endif
