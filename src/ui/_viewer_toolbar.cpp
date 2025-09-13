#include "_viewer_toolbar.h"

namespace imgvt{

    void refresh_window(struct viewer_window_ *main_window){

            main_window->dragging.is_config = TRUE;
            main_window->dragging.final_drawing = cv::Rect(0,0,0,0);
            gdk_threads_add_idle((GSourceFunc)gtk_widget_queue_draw,(void*)main_window->viewer_scrolled_window_viewpoint_drawing);

            main_window->image_zoomed = imgm::resizeKeepAspectRatio(main_window->image, main_window->zoom_val[main_window->current_zoom],&(main_window->crop_preview));
            gtk_widget_destroy (GTK_WIDGET(main_window->img_test));
            main_window->img_test = gops::cv_image_to_gtk_image(main_window->image_zoomed);
            gtk_container_add(GTK_CONTAINER(main_window->image_window_event), GTK_WIDGET(main_window->img_test));
            gtk_widget_show_all(main_window->window);
            get_offset(main_window);

    }

    void get_offset(struct viewer_window_ *viewer_window){

        GtkAllocation viewport_alloc;
        gtk_widget_get_allocation(viewer_window->viewer_scrolled_window, &viewport_alloc);
        gint viewport_w = viewport_alloc.width;
        gint viewport_h = viewport_alloc.height;

        int image_w = viewer_window->image_zoomed.cols;
        int image_h = viewer_window->image_zoomed.rows;

        if(viewport_w >= image_w){
            viewer_window->dragging.offset_x = ((double)viewport_w - (double)image_w)/2;
            viewer_window->dragging.add_offset = TRUE;
        }else{
            viewer_window->dragging.offset_x = 0;
        }

        if(viewport_h >= image_h){
            viewer_window->dragging.offset_y = ((double)viewport_h - (double)image_h)/2;
            viewer_window->dragging.add_offset = TRUE;
        }else{
            viewer_window->dragging.offset_y = 0;
        }
    }


    void draw_rect(gdouble x,gdouble y, gdouble dx, gdouble dy,cairo_t *cr){

            cairo_set_operator(cr, CAIRO_OPERATOR_OVER);


            cairo_set_source_rgba (cr, 0, 0, 0, .5);
            cairo_set_line_width(cr, 7);
            cairo_rectangle(cr,x-5,y-5,dx+10,dy+10);
            cairo_stroke(cr);
            cairo_rectangle(cr,x+5,y+5,dx-10,dy-10);
            cairo_stroke(cr);

            cairo_set_line_width(cr, 3);
            cairo_set_source_rgba (cr, 1, 0, 0, 1);
            cairo_rectangle(cr,x,y,dx,dy);
            cairo_stroke(cr);

            //cairo_rectangle(cr,x-3,y-3,dx+3,dy+3);
            //cairo_stroke(cr);

    }

    gboolean on_draw(GtkWidget *widget, cairo_t *cr, gpointer data) {
        Drag_Par *draw_data = (Drag_Par *)data;

        //std::cout <<"drawing " <<draw_data->final_drawing<<"\n";
        if (draw_data->is_config) {

            draw_rect(draw_data->final_drawing.x + draw_data->offset_x,
                    draw_data->final_drawing.y + draw_data->offset_y,
                    draw_data->final_drawing.width,
                    draw_data->final_drawing.height,cr);

        }else{

            draw_rect(draw_data->start_x -draw_data->rel_x,
                    draw_data->start_y -draw_data->rel_y,
                    draw_data->dx,
                    draw_data->dy,cr);

        }

        return FALSE;
    }


    gboolean zin(GtkWidget* self,GdkEventButton *event,struct viewer_window_ *main_window){

        if(main_window->current_zoom < main_window->zoom_val.size() - 1){

            int image_w = main_window->image_zoomed.cols;
            int image_h = main_window->image_zoomed.rows;

            main_window->current_zoom++;
            main_window->image_zoomed = imgm::resizeKeepAspectRatio(main_window->image, main_window->zoom_val[main_window->current_zoom],&(main_window->crop_preview));

            gtk_widget_destroy (GTK_WIDGET(main_window->img_test));
            main_window->img_test = gops::cv_image_to_gtk_image(main_window->image_zoomed);
            gtk_container_add(GTK_CONTAINER(main_window->image_window_event), GTK_WIDGET(main_window->img_test));

            main_window->dragging.is_config = TRUE;

            int image_w_norm = main_window->image_zoomed.cols;
            int image_h_norm = main_window->image_zoomed.rows;

            double ratio = ((double)image_w_norm/(double)image_w + (double)image_h_norm/(double)image_h)/2;

            cv::Rect window = main_window->dragging.final_drawing;
            window = util::scaleRect(window,ratio,ratio);
            window.x = main_window->dragging.final_drawing.x * ratio;
            window.y = main_window->dragging.final_drawing.y * ratio;
            main_window->dragging.final_drawing = window;

            gtk_widget_show_all(main_window->window);
            get_offset(main_window);

            gdk_threads_add_idle((GSourceFunc)gtk_widget_queue_draw,(void*)main_window->viewer_scrolled_window_viewpoint_drawing);
            //main_window->dragging.is_config = FALSE;

        }

        return FALSE;

    }


    gboolean zout(GtkWidget* self,GdkEventButton *event,struct viewer_window_ *main_window){

        if(main_window->current_zoom > 0){

            int image_w = main_window->image_zoomed.cols;
            int image_h = main_window->image_zoomed.rows;

            main_window->current_zoom--;
            main_window->image_zoomed = imgm::resizeKeepAspectRatio(main_window->image, main_window->zoom_val[main_window->current_zoom],&(main_window->crop_preview));

            gtk_widget_destroy (GTK_WIDGET(main_window->img_test));
            main_window->img_test = gops::cv_image_to_gtk_image(main_window->image_zoomed);
            gtk_container_add(GTK_CONTAINER(main_window->image_window_event), GTK_WIDGET(main_window->img_test));

            main_window->dragging.is_config = TRUE;

            int image_w_norm = main_window->image_zoomed.cols;
            int image_h_norm = main_window->image_zoomed.rows;

            double ratio = ((double)image_w_norm/(double)image_w + (double)image_h_norm/(double)image_h)/2;

            cv::Rect window = main_window->dragging.final_drawing;
            window = util::scaleRect(window,ratio,ratio);
            window.x = main_window->dragging.final_drawing.x * ratio;
            window.y = main_window->dragging.final_drawing.y * ratio;
            main_window->dragging.final_drawing = window;

            gtk_widget_show_all(main_window->window);
            get_offset(main_window);

            gdk_threads_add_idle((GSourceFunc)gtk_widget_queue_draw,(void*)main_window->viewer_scrolled_window_viewpoint_drawing);

        }


        return FALSE;

    }


    gboolean return_rect(GtkWidget* self,GdkEventButton *event,struct viewer_window_ *main_window){

        double ratio = (double)(main_window->image.cols) / (double)(main_window->zoom_val[main_window->current_zoom]);

        cv::Rect image(0,0,main_window->image.cols,main_window->image.rows);
        cv::Rect window;

        if(main_window->dragging.final_drawing.width < 0){
            window.width = 0-(main_window->dragging.final_drawing.width);
            window.x = main_window->dragging.final_drawing.x + main_window->dragging.final_drawing.width;

        }else{

            window.x = main_window->dragging.final_drawing.x;
            window.width =  main_window->dragging.final_drawing.width;

        }

        if(main_window->dragging.final_drawing.height < 0){
            window.height = 0-(main_window->dragging.final_drawing.height);
            window.y = main_window->dragging.final_drawing.y + main_window->dragging.final_drawing.height;
        }else{

            window.y = main_window->dragging.final_drawing.y;
            window.height =  main_window->dragging.final_drawing.height;

        }

        //std::cout<<"window.x: "<<window.x<<"\n";

        window = util::scaleRect(window,ratio,ratio); // rescale rect from zoomed to prev
        window.x = main_window->crop_preview.x + window.x;
        window.y = main_window->crop_preview.y + window.y;
        cv::Rect ROI = image bitand window;
        ROI = ROI bitand main_window->crop_preview;

        if(ROI.area()>0){

            main_window->crop_preview = ROI;

        }else{

            return FALSE;

        }

        main_window->crop_vec.resize(main_window->ret_counter + 1);
        main_window->crop_vec.push_back(main_window->crop_preview);
        main_window->ret_counter++;

        refresh_window(main_window);

        return FALSE;

    }


    gboolean save_img(GtkWidget* self,GdkEventButton *event,struct viewer_window_ *main_window){

        cv::Mat test = main_window->panorama_->get_panorama(main_window->crop_preview);

        GtkFileChooserNative *native;
        GtkFileChooserAction action = GTK_FILE_CHOOSER_ACTION_SAVE;
        GtkFileChooser *chooser;
        GtkFileFilter *filter;
        gint res;

        filter = gtk_file_filter_new();
        gtk_file_filter_add_pattern(filter, "*.png");

        native = gtk_file_chooser_native_new ("Save File",GTK_WINDOW(main_window->window),action,"_Save","_Cancel");
        chooser = GTK_FILE_CHOOSER (native);
        gtk_file_chooser_set_do_overwrite_confirmation (chooser, TRUE);
        gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(chooser), filter);
        gtk_file_chooser_set_current_name(GTK_FILE_CHOOSER(chooser), "new.png");

        res = gtk_native_dialog_run (GTK_NATIVE_DIALOG (native));
        if (res == GTK_RESPONSE_ACCEPT){

            gchar* username = gtk_file_chooser_get_filename (chooser);
            //gchar* cname = gtk_file_chooser_get_current_name (chooser);

            gchar* location = gtk_file_chooser_get_filename (chooser);
            cv::imwrite(username, test);

        }

        g_object_unref (native);


        return FALSE;
    }


    gboolean redo_crop(GtkWidget* self,GdkEventButton *event,struct viewer_window_ *main_window){

        if(main_window->crop_vec.size() - 1 > main_window->ret_counter){

            main_window->ret_counter++;
            main_window->crop_preview = main_window->crop_vec[main_window->ret_counter];
            refresh_window(main_window);

        }


        return FALSE;
    }


    gboolean undo_crop(GtkWidget* self,GdkEventButton *event,struct viewer_window_ *main_window){

        if(main_window->ret_counter > 0){

            main_window->ret_counter--;
            main_window->crop_preview = main_window->crop_vec[main_window->ret_counter];
            refresh_window(main_window);

        }

        return FALSE;
    }


    void connect_signals(struct viewer_window_ *main_window){

        g_signal_connect(main_window->toolbar.toolbar_main_zin, "button-release-event", G_CALLBACK(zin), main_window);
        g_signal_connect(main_window->toolbar.toolbar_main_zout, "button-release-event", G_CALLBACK(zout), main_window);

        g_signal_connect(main_window->toolbar.toolbar_main_crop, "button-release-event", G_CALLBACK(return_rect), main_window);
        g_signal_connect(main_window->toolbar.toolbar_main_redo_crop, "button-release-event", G_CALLBACK(redo_crop), main_window);
        g_signal_connect(main_window->toolbar.toolbar_main_undo_crop, "button-release-event", G_CALLBACK(undo_crop), main_window);

        g_signal_connect(main_window->toolbar.toolbar_main_save, "button-release-event", G_CALLBACK(save_img), main_window);

}

    void create_toolbar(GtkWidget *add_to,struct toolbar_viewer *toolbar,struct viewer_window_ *main_window){

        toolbar->toolbar_main = gtk_toolbar_new();

        toolbar->toolbar_main_save_img = gtk_image_new_from_icon_name ("document-save-symbolic",GTK_ICON_SIZE_SMALL_TOOLBAR );
        toolbar->toolbar_main_save = gtk_tool_button_new (toolbar->toolbar_main_save_img,"save");
        gtk_toolbar_insert(GTK_TOOLBAR(toolbar->toolbar_main),toolbar->toolbar_main_save,-1);
        gtk_widget_set_tooltip_text (GTK_WIDGET(toolbar->toolbar_main_save),"Save Panorama");

        toolbar->toolbar_seperator_0 = gtk_separator_tool_item_new();
        gtk_toolbar_insert(GTK_TOOLBAR(toolbar->toolbar_main),toolbar->toolbar_seperator_0,-1);

        toolbar->toolbar_main_crop_img = gtk_image_new_from_icon_name("zoom-fit-best-symbolic",GTK_ICON_SIZE_SMALL_TOOLBAR );
        toolbar->toolbar_main_crop = gtk_tool_button_new (toolbar->toolbar_main_crop_img,"crop");
        gtk_toolbar_insert(GTK_TOOLBAR(toolbar->toolbar_main),toolbar->toolbar_main_crop,-1);
        gtk_widget_set_tooltip_text (GTK_WIDGET(toolbar->toolbar_main_crop),"Crop to Selection");

        toolbar->toolbar_main_undo_crop_img = gtk_image_new_from_icon_name("go-previous-symbolic",GTK_ICON_SIZE_SMALL_TOOLBAR );
        toolbar->toolbar_main_undo_crop = gtk_tool_button_new (toolbar->toolbar_main_undo_crop_img,"undo");
        gtk_toolbar_insert(GTK_TOOLBAR(toolbar->toolbar_main),toolbar->toolbar_main_undo_crop,-1);
        gtk_widget_set_tooltip_text (GTK_WIDGET(toolbar->toolbar_main_undo_crop),"Undo Crop");

        toolbar->toolbar_main_redo_crop_img = gtk_image_new_from_icon_name("go-next-symbolic",GTK_ICON_SIZE_SMALL_TOOLBAR );
        toolbar->toolbar_main_redo_crop = gtk_tool_button_new (toolbar->toolbar_main_redo_crop_img,"redo");
        gtk_toolbar_insert(GTK_TOOLBAR(toolbar->toolbar_main),toolbar->toolbar_main_redo_crop,-1);
        gtk_widget_set_tooltip_text (GTK_WIDGET(toolbar->toolbar_main_redo_crop),"Redo Crop");

        toolbar->toolbar_seperator_1 = gtk_separator_tool_item_new();
        gtk_toolbar_insert(GTK_TOOLBAR(toolbar->toolbar_main),toolbar->toolbar_seperator_1,-1);

        toolbar->toolbar_main_zin_img = gtk_image_new_from_icon_name ("zoom-in-symbolic",GTK_ICON_SIZE_SMALL_TOOLBAR );
        toolbar->toolbar_main_zin = gtk_tool_button_new (toolbar->toolbar_main_zin_img,"Zoom In");
        gtk_toolbar_insert(GTK_TOOLBAR(toolbar->toolbar_main),toolbar->toolbar_main_zin,-1);
        gtk_widget_set_tooltip_text (GTK_WIDGET(toolbar->toolbar_main_zin),"Zoom In");

        toolbar->toolbar_main_zout_img = gtk_image_new_from_icon_name ("zoom-out-symbolic",GTK_ICON_SIZE_SMALL_TOOLBAR );
        toolbar->toolbar_main_zout = gtk_tool_button_new (toolbar->toolbar_main_zout_img,"Zoom Out");
        gtk_toolbar_insert(GTK_TOOLBAR(toolbar->toolbar_main),toolbar->toolbar_main_zout,-1);
        gtk_widget_set_tooltip_text (GTK_WIDGET(toolbar->toolbar_main_zout),"Zoom Out");


        if(main_window->current_zoom == -1){

            gtk_widget_set_sensitive(GTK_WIDGET(toolbar->toolbar_main_zin),FALSE);
            gtk_widget_set_sensitive(GTK_WIDGET(toolbar->toolbar_main_zout),FALSE);

        }


        gtk_box_pack_end (GTK_BOX(add_to),toolbar->toolbar_main,FALSE,FALSE,0);

        connect_signals(main_window);

    }


}



