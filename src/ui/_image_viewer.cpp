#include "_image_viewer.h"

namespace imgv{

    const gchar* ret_path8(std::filesystem::path& path);

    gboolean update_progress(gpointer data) {

        progress_bar_ *progress_bar = static_cast<progress_bar_*>(data);
        if(progress_bar->finished) {

            return G_SOURCE_REMOVE;

        }

        double frac = progress_bar->fraction.load(std::memory_order_relaxed);

        gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(progress_bar->pbar_progress_bar), frac);

        const gchar* number = "%d%%";
        gchar* temp = new gchar[strlen(number) + strlen(progress_bar->loading_text) + 1];
        strcpy(temp, progress_bar->loading_text);
        strcat(temp, number);


        gchar *text = g_strdup_printf(temp, (int)(frac * 100));
        gtk_progress_bar_set_text(GTK_PROGRESS_BAR(progress_bar->pbar_progress_bar), text);

        g_free(text);
        delete[] temp;

        if (frac >= 1.0) {

            progress_bar->finished = true;
            gtk_widget_destroy(progress_bar->window);

            return G_SOURCE_REMOVE;
        }
        gtk_widget_show_all(progress_bar->window);

        return G_SOURCE_CONTINUE;
    }


    void bar_closed(GtkWidget *widget, progress_bar_ *progress_bar){

        if(progress_bar->finished){
            if(progress_bar->error == false){
                show_image(progress_bar);
            }else{
                dbox(progress_bar->what_error);
            }

            //progress_bar->thread_save.wait(false, std::memory_order_acquire);
            gtk_widget_set_sensitive(GTK_WIDGET(progress_bar->main_window->toolbar.toolbar_main_new),true);
            progress_bar->cleanup();
            g_free(progress_bar);
            return;

        }

        progress_bar->finished = true;
        progress_bar->view->panorama_->cancel();
        gtk_widget_set_sensitive(GTK_WIDGET(progress_bar->main_window->toolbar.toolbar_main_new),true);
        progress_bar->cleanup();
        //g_free(progress_bar);

    }


    void connect_signals_bar(progress_bar_ *progress_bar){

        g_signal_connect(progress_bar->window, "destroy", G_CALLBACK(bar_closed), progress_bar);
        g_signal_connect(progress_bar->pbar_button_box_cancel, "clicked", G_CALLBACK(gtk_window_close), GTK_WINDOW(progress_bar->window));

    }


    void open_progress_bar(GtkWidget *add_to,struct progress_bar_ *progress_bar,struct main_window_ *main_window){

        progress_bar->window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
        gtk_window_set_title(GTK_WINDOW(progress_bar->window), "Working...");
        gtk_window_set_default_size(GTK_WINDOW(progress_bar->window), 300, 100);
        gtk_window_set_resizable(GTK_WINDOW(progress_bar->window), FALSE);
        gtk_window_set_position(GTK_WINDOW(progress_bar->window), GTK_WIN_POS_CENTER_ON_PARENT);
        gtk_window_set_transient_for(GTK_WINDOW(progress_bar->window), GTK_WINDOW(add_to));

        progress_bar->pbar_main_vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
        gtk_container_add(GTK_CONTAINER(progress_bar->window), progress_bar->pbar_main_vbox);
        gtk_widget_set_margin_top(progress_bar->pbar_main_vbox, 5);
        gtk_widget_set_margin_bottom(progress_bar->pbar_main_vbox, 5);
        gtk_widget_set_margin_start(progress_bar->pbar_main_vbox, 5);
        gtk_widget_set_margin_end(progress_bar->pbar_main_vbox, 5);

        progress_bar->pbar_progress_bar = gtk_progress_bar_new();
        gtk_progress_bar_set_show_text(GTK_PROGRESS_BAR(progress_bar->pbar_progress_bar), TRUE);
        gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(progress_bar->pbar_progress_bar), 0);
        gtk_box_pack_start(GTK_BOX(progress_bar->pbar_main_vbox), progress_bar->pbar_progress_bar, TRUE, TRUE, 0);

        progress_bar->pbar_button_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
        gtk_box_pack_start(GTK_BOX(progress_bar->pbar_main_vbox), progress_bar->pbar_button_box, FALSE, FALSE, 10);

        progress_bar->pbar_button_box_cancel = gtk_button_new_with_label("CANCEL");
        gtk_box_pack_end(GTK_BOX(progress_bar->pbar_button_box), progress_bar->pbar_button_box_cancel, TRUE, FALSE, 0);

        progress_bar->bar_text(" ");

        connect_signals_bar(progress_bar);
        gtk_widget_show_all(progress_bar->window);

}


gboolean on_mouse_press(GtkWidget *widget, GdkEventButton *event, struct viewer_window_ *viewer_window) {

    viewer_window->dragging.start_x = event->x_root;
    viewer_window->dragging.start_y = event->y_root;
    GdkDisplay *display;
    GdkCursor *cursor;

    GtkAdjustment *hadj = gtk_scrolled_window_get_hadjustment(GTK_SCROLLED_WINDOW(widget));
    GtkAdjustment *vadj = gtk_scrolled_window_get_vadjustment(GTK_SCROLLED_WINDOW(widget));

    viewer_window->dragging.hadj_value = gtk_adjustment_get_value(hadj);
    viewer_window->dragging.vadj_value = gtk_adjustment_get_value(vadj);


    GdkWindow *gdk_window = gtk_widget_get_window(GTK_WIDGET(viewer_window->img_test));
    gdk_window_get_origin(gdk_window, &viewer_window->dragging.rel_x, &viewer_window->dragging.rel_y);


    if (event->button == GDK_BUTTON_PRIMARY) {

        viewer_window->dragging.is_dragging = TRUE;
        viewer_window->dragging.is_drawing = FALSE;
        viewer_window->dragging.is_config = TRUE;

        display = gtk_widget_get_display(widget);
        cursor = gdk_cursor_new_from_name(display, "grabbing");
        gdk_window_set_cursor(gtk_widget_get_window(widget), cursor);

        g_object_unref(cursor);
        gtk_widget_queue_draw(viewer_window->viewer_scrolled_window_viewpoint_drawing);
        //gdk_threads_add_idle((GSourceFunc)gtk_widget_queue_draw,(void*)viewer_window->viewer_scrolled_window_viewpoint_drawing);

        return TRUE;
    }else if(event->button == GDK_BUTTON_SECONDARY){

        viewer_window->dragging.is_dragging = FALSE;
        viewer_window->dragging.is_drawing = TRUE;
        viewer_window->dragging.is_config = FALSE;

        display = gtk_widget_get_display(widget);
        cursor = gdk_cursor_new_from_name(display,"cell");
        gdk_window_set_cursor(gtk_widget_get_window(widget), cursor);


        return TRUE;
    }

    return FALSE;

}


gboolean on_mouse_motion(GtkWidget *widget, GdkEventMotion *event, struct viewer_window_ *viewer_window) {

    if (viewer_window->dragging.is_dragging) {

        viewer_window->dragging.dx = event->x_root - viewer_window->dragging.start_x;
        viewer_window->dragging.dy = event->y_root - viewer_window->dragging.start_y;

        GtkAdjustment *hadj = gtk_scrolled_window_get_hadjustment(GTK_SCROLLED_WINDOW(widget));
        GtkAdjustment *vadj = gtk_scrolled_window_get_vadjustment(GTK_SCROLLED_WINDOW(widget));
        gtk_adjustment_set_value(hadj, viewer_window->dragging.hadj_value - viewer_window->dragging.dx);
        gtk_adjustment_set_value(vadj, viewer_window->dragging.vadj_value - viewer_window->dragging.dy);

    }else if(viewer_window->dragging.is_drawing){

        viewer_window->dragging.dx = event->x_root - viewer_window->dragging.start_x;
        viewer_window->dragging.dy = event->y_root - viewer_window->dragging.start_y;

        viewer_window->dragging.add_offset = FALSE;
        gtk_widget_queue_draw(viewer_window->viewer_scrolled_window_viewpoint_drawing);
        //gdk_threads_add_idle((GSourceFunc)gtk_widget_queue_draw,(void*)viewer_window->viewer_scrolled_window_viewpoint_drawing);

    }

    return FALSE;
}


gboolean on_mouse_release(GtkWidget *widget, GdkEventButton *event, struct viewer_window_ *viewer_window) {

    if (event->button == GDK_BUTTON_PRIMARY) {

        viewer_window->dragging.is_dragging = FALSE;
        viewer_window->dragging.is_config = TRUE;
        viewer_window->dragging.is_drawing = FALSE;
        gdk_window_set_cursor(gtk_widget_get_window(widget), NULL);
        //gdk_threads_add_idle((GSourceFunc)gtk_widget_queue_draw,(void*)viewer_window->viewer_scrolled_window_viewpoint_drawing);

        return TRUE;
    }else if(event->button == GDK_BUTTON_SECONDARY){

        //std::cout<<"sec \n";
        viewer_window->dragging.is_config = FALSE;
        gdk_window_set_cursor(gtk_widget_get_window(widget), NULL);

        if(viewer_window->dragging.is_drawing){
            viewer_window->dragging.is_drawing = FALSE;
            viewer_window->dragging.final_drawing =
            cv::Rect((int)viewer_window->dragging.start_x - viewer_window->dragging.rel_x - (int)viewer_window->dragging.offset_x,
                    (int)viewer_window->dragging.start_y - viewer_window->dragging.rel_y - (int)viewer_window->dragging.offset_y,
                    (int)viewer_window->dragging.dx,
                    (int)viewer_window->dragging.dy);
        }


        return TRUE;
    }

    return FALSE;
}


void window_quit(GtkWidget *widget, gpointer data){

    struct widget_and_Id* wind_id;
    wind_id = (struct widget_and_Id*)data;
    wind_id->window->view[wind_id->id].panorama_.reset();
    wind_id->window->view.erase(wind_id->id);

    delete wind_id;

}


gboolean on_resize(GtkWidget *widget,GdkRectangle *event, struct viewer_window_ *viewer_window){

    viewer_window->dragging.is_config = TRUE;
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


    viewer_window->dragging.is_drawing = FALSE;
    if((not(event->width == viewer_window->w_x)) or (not(event->height == viewer_window->w_y))){

        viewer_window->dragging.is_config = TRUE;
        gtk_widget_queue_draw(viewer_window->viewer_scrolled_window_viewpoint_drawing);
        //gdk_threads_add_idle((GSourceFunc)gtk_widget_queue_draw,(void*)viewer_window->viewer_scrolled_window_viewpoint_drawing);

    }

    viewer_window->w_x = event->width;
    viewer_window->w_y = event->height;

    return FALSE;
}


gboolean on_scroll(GtkWidget *widget,GdkEventScroll event, struct viewer_window_ *viewer_window){

    viewer_window->dragging.is_drawing = FALSE;
    viewer_window->dragging.is_config = TRUE;
    gtk_widget_queue_draw(viewer_window->viewer_scrolled_window_viewpoint_drawing);
    //gdk_threads_add_idle((GSourceFunc)gtk_widget_queue_draw,(void*)viewer_window->viewer_scrolled_window_viewpoint_drawing);

    return FALSE;
}



gboolean window_maximized(GtkWidget *widget, GdkEventWindowState *event, struct viewer_window_ *viewer_window){

    GtkAllocation viewport_alloc;
    gtk_widget_get_allocation(viewer_window->viewer_scrolled_window, &viewport_alloc);
    gint viewport_w;
    gint viewport_h;


    viewer_window->dragging.is_config = FALSE;

    if (event->changed_mask & GDK_WINDOW_STATE_MAXIMIZED) {

        if (event->new_window_state & GDK_WINDOW_STATE_MAXIMIZED) {

            gtk_widget_get_allocation(viewer_window->viewer_scrolled_window, &viewport_alloc);
            viewport_w = viewport_alloc.width;
            viewport_h = viewport_alloc.height;

            viewer_window->dragging.rel_x = -46;
            viewer_window->dragging.rel_y = -46;
            std::cout<<"max x: "<<viewport_w<<"\n";
            std::cout<<"max y: "<<viewport_h<<"\n";

        }else{

            gtk_widget_get_allocation(viewer_window->viewer_scrolled_window, &viewport_alloc);
            viewport_w = viewport_alloc.width;
            viewport_h = viewport_alloc.height;

            GdkWindow *gdk_window = gtk_widget_get_window(GTK_WIDGET(viewer_window->img_test));
            gdk_window_get_origin(gdk_window, &viewer_window->dragging.rel_x, &viewer_window->dragging.rel_y);
            std::cout<<"min x: "<<viewport_w<<"\n";
            std::cout<<"min y: "<<viewport_h<<"\n";

        }
    }

    return FALSE;
}


void connect_signals(struct main_window_ *main_window,int id){

        struct widget_and_Id* wind_id;
        wind_id = new struct widget_and_Id;
        wind_id->id = id;
        wind_id->window = main_window;
        gpointer test = wind_id;

        g_signal_connect(main_window->view[id].window, "destroy", G_CALLBACK(window_quit),test);

        g_signal_connect(main_window->view[id].viewer_scrolled_window, "button-press-event", G_CALLBACK(on_mouse_press), &main_window->view[id]);

        g_signal_connect(main_window->view[id].viewer_scrolled_window, "motion-notify-event", G_CALLBACK(on_mouse_motion), &main_window->view[id]);

        g_signal_connect(main_window->view[id].viewer_scrolled_window, "button-release-event", G_CALLBACK(on_mouse_release), &main_window->view[id]);

        g_signal_connect(main_window->view[id].window, "size-allocate", G_CALLBACK(on_resize), &main_window->view[id]);

        GtkAdjustment *hadj = gtk_scrolled_window_get_hadjustment(GTK_SCROLLED_WINDOW(main_window->view[id].viewer_scrolled_window));
        GtkAdjustment *vadj = gtk_scrolled_window_get_vadjustment(GTK_SCROLLED_WINDOW(main_window->view[id].viewer_scrolled_window));

        #ifdef __linux__

        g_signal_connect(hadj, "value-changed", G_CALLBACK(on_scroll), &main_window->view[id]);
        g_signal_connect(vadj, "value-changed", G_CALLBACK(on_scroll), &main_window->view[id]);

        #endif

        //g_signal_connect(main_window->view[id].window, "window-state-event", G_CALLBACK(window_maximized), &main_window->view[id]);

        g_signal_connect(main_window->view[id].viewer_scrolled_window_viewpoint_drawing, "draw", G_CALLBACK(imgvt::on_draw), &(main_window->view[id].dragging));

}


void get_files(GtkFlowBoxChild *child,struct widget_and_Id *for_list){

    GList *children = gtk_container_get_children(GTK_CONTAINER(child));
    gpointer widget = g_list_nth_data(children,0);

    int ind = gops::findStringIndex((for_list->window->ipts.f_list),gtk_widget_get_name(GTK_WIDGET( widget)));

    if (ind >= 0){

        for_list->window->view[for_list->id].panorama_->add_images(for_list->window->ipts.f_list[ind]);

    }
    else{
        std::cout<< gtk_widget_get_name (GTK_WIDGET( widget))<<" not found";
    }

    g_list_free(g_steal_pointer (&children));
}


void on_drawing_area_realize(GtkWidget *widget, gpointer data) {
    GdkWindow *gdk_window = gtk_widget_get_window(widget);
    if (gdk_window) {
        gdk_window_set_pass_through(gdk_window, TRUE);
    }
}


gboolean show_image(struct progress_bar_ *bar) {

    struct viewer_window_ *progress_bar_viewer = bar->view;

    gtk_window_set_default_size(GTK_WINDOW(progress_bar_viewer->window), 800, 600);

    progress_bar_viewer->viewer_box = gtk_box_new (GTK_ORIENTATION_VERTICAL,0);
    gtk_box_set_homogeneous (GTK_BOX(progress_bar_viewer->viewer_box),FALSE);
    gtk_container_add (GTK_CONTAINER(progress_bar_viewer->window),progress_bar_viewer->viewer_box);

    progress_bar_viewer->viewer_scrolled_window = gtk_scrolled_window_new (NULL, NULL);
    progress_bar_viewer->viewer_scrolled_window_viewpoint = gtk_viewport_new (NULL, NULL);
    gtk_container_add(GTK_CONTAINER(progress_bar_viewer->viewer_scrolled_window), progress_bar_viewer->viewer_scrolled_window_viewpoint);
    gtk_box_pack_end(GTK_BOX(progress_bar_viewer->viewer_box),progress_bar_viewer->viewer_scrolled_window,TRUE,TRUE,0);

    //progress_bar_viewer->image = cv::imread("/home/sd_bert/Pictures/1725014833368114.png", cv::IMREAD_COLOR);
    progress_bar_viewer->image = progress_bar_viewer->panorama_->get_preview();
    int image_w = progress_bar_viewer->image.cols;
    int image_h = progress_bar_viewer->image.rows;

    cv::Rect imagerect(0,0,image_w,image_h);
    progress_bar_viewer->crop_preview = imagerect;
    std::vector<cv::Rect>().swap(progress_bar_viewer->crop_vec);
    progress_bar_viewer->crop_vec.push_back(imagerect);
    progress_bar_viewer->ret_counter = 0;


    if ( 200 < (progress_bar_viewer->image.cols - 800)){

        float z_num = (progress_bar_viewer->image.cols - 800)/200;
        if (z_num < 3){

            progress_bar_viewer->zoom_val.resize(2);
            progress_bar_viewer->zoom_val[0] = 800;
            progress_bar_viewer->zoom_val[1] = progress_bar_viewer->image.cols;

        }else{

            int zooms = (int)z_num;
            progress_bar_viewer->zoom_val.resize(zooms+1);
            progress_bar_viewer->zoom_val[0] = 800;
            progress_bar_viewer->zoom_val[zooms] = progress_bar_viewer->image.cols;
            for(int i = 1; i < zooms;i++){

                progress_bar_viewer->zoom_val[i] = 800 + 200*i;
            }

        }

    }

    if (1 > progress_bar_viewer->zoom_val.size()){

        progress_bar_viewer->image_zoomed = progress_bar_viewer->image;
        progress_bar_viewer->current_zoom = -1;

    }else{

        progress_bar_viewer->image_zoomed = imgm::resizeKeepAspectRatio(progress_bar_viewer->image, progress_bar_viewer->zoom_val[1]);
        progress_bar_viewer->current_zoom = 1;

    }

    progress_bar_viewer->img_test = gops::cv_image_to_gtk_image(progress_bar_viewer->image_zoomed);

    progress_bar_viewer->viewer_scrolled_window_viewpoint_overlay = gtk_overlay_new();

    progress_bar_viewer->image_window_event = gtk_event_box_new();
    gtk_container_add(GTK_CONTAINER(progress_bar_viewer->image_window_event), GTK_WIDGET(progress_bar_viewer->img_test));
    gtk_widget_add_events(progress_bar_viewer->image_window_event, GDK_BUTTON_PRESS_MASK bitor GDK_BUTTON_RELEASE_MASK bitor GDK_POINTER_MOTION_MASK);

    gtk_container_add(GTK_CONTAINER(progress_bar_viewer->viewer_scrolled_window_viewpoint_overlay),progress_bar_viewer->image_window_event);

    progress_bar_viewer->viewer_scrolled_window_viewpoint_drawing = gtk_drawing_area_new();

    gtk_overlay_add_overlay(GTK_OVERLAY(progress_bar_viewer->viewer_scrolled_window_viewpoint_overlay),progress_bar_viewer->viewer_scrolled_window_viewpoint_drawing);
    gtk_widget_set_app_paintable(progress_bar_viewer->viewer_scrolled_window_viewpoint_drawing, TRUE);
    gtk_widget_set_sensitive(progress_bar_viewer->viewer_scrolled_window_viewpoint_drawing, TRUE);
    gtk_overlay_set_overlay_pass_through(GTK_OVERLAY(progress_bar_viewer->viewer_scrolled_window_viewpoint_overlay), progress_bar_viewer->viewer_scrolled_window_viewpoint_drawing, TRUE);

    g_signal_connect(progress_bar_viewer->viewer_scrolled_window_viewpoint_drawing, "realize", G_CALLBACK(on_drawing_area_realize), NULL);

    gtk_container_add(GTK_CONTAINER(progress_bar_viewer->viewer_scrolled_window_viewpoint),progress_bar_viewer->viewer_scrolled_window_viewpoint_overlay);

    imgvt::create_toolbar(progress_bar_viewer->viewer_box,&(progress_bar_viewer->toolbar),progress_bar_viewer);

    gtk_widget_show_all(progress_bar_viewer->window);
    connect_signals(bar->main_window,progress_bar_viewer->windows_idx);

    return true;

}


void create_viewer(struct main_window_ *main_window,GtkWidget* self,GdkEventButton *event){

    //ensure only 1 window of the same type is created
    int test = main_window->view_number;
    main_window->view_number++;
    if (main_window->view.count(test)){return;}

    gtk_widget_set_sensitive(GTK_WIDGET(main_window->toolbar.toolbar_main_new),false);

    struct viewer_window_ viewer;
    viewer.window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    main_window->view[test] = std::move(viewer);
    main_window->view[test].windows_idx = test;
    std::vector<std::string> files;
    //pan::panorama pan(files);

    GList* s_list = gtk_flow_box_get_selected_children(GTK_FLOW_BOX( main_window->flowbox.flowbox_main));

    struct widget_and_Id for_list;
    for_list.id = test;
    for_list.window = main_window;

    main_window->view[test].progress_bar = g_new0(progress_bar_, 1);
    main_window->view[test].progress_bar->init();
    main_window->view[test].progress_bar->view = &(main_window->view[test]);
    main_window->view[test].progress_bar->main_window = main_window;
    main_window->view[test].progress_bar->test = test;

    //show_image(main_window->view[test].progress_bar);

    main_window->view[test].panorama_ = std::make_shared<pan::panorama>(files,main_window->view[test].progress_bar);

    g_list_foreach(s_list,(GFunc)get_files,&for_list); //deletes files

    open_progress_bar(main_window->window,main_window->view[test].progress_bar,main_window);

    g_timeout_add(100,update_progress, main_window->view[test].progress_bar);

    auto get_pan = [main_window,test](pan::config* conf) {
        try {

            main_window->view[test].panorama_->stitch_panorama(conf);
            cv::Mat prev = main_window->view[test].panorama_->get_preview();

        }catch(const std::runtime_error& e){

            main_window->view[test].progress_bar->error = true;
            const char* message = e.what();
            int length = std::strlen(message);
            main_window->view[test].progress_bar->what_error = g_utf8_make_valid(message,length);

        }catch (...) {
            main_window->view[test].progress_bar->error = true;
            main_window->view[test].progress_bar->what_error = "Unknown ERROR!";
        }

        main_window->view[test].progress_bar->fraction = 1;

    };

    auto thread = std::thread(get_pan,main_window->config_);
    thread.detach();

}

}


