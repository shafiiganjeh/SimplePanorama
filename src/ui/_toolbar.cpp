
#include "_toolbar.h"

namespace tbar{

void create_toolbar(GtkWidget *add_to,struct toolbar_ *toolbar,struct main_window_ *main_window){

        toolbar->toolbar_main = gtk_toolbar_new ();

        toolbar->toolbar_main_cut_img = gtk_image_new_from_icon_name ("edit-cut",GTK_ICON_SIZE_SMALL_TOOLBAR );
        toolbar->toolbar_main_cut = gtk_tool_button_new (toolbar->toolbar_main_cut_img,"Cut");
        gtk_toolbar_insert(GTK_TOOLBAR(toolbar->toolbar_main),toolbar->toolbar_main_cut,0);
        gtk_widget_set_tooltip_text (GTK_WIDGET(toolbar->toolbar_main_cut),"Cut selection");

        gtk_box_pack_end (GTK_BOX(add_to),toolbar->toolbar_main,FALSE,FALSE,0);
}


}

