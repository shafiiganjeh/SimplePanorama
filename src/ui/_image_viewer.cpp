#include "_image_viewer.h"
#include "_progress_bar.h"

namespace imgv{


gboolean on_mouse_press(GtkWidget *widget, GdkEventButton *event, struct viewer_window_ *viewer_window) {
    if (event->button == GDK_BUTTON_PRIMARY) {

        GtkAdjustment *hadj = gtk_scrolled_window_get_hadjustment(GTK_SCROLLED_WINDOW(widget));
        GtkAdjustment *vadj = gtk_scrolled_window_get_vadjustment(GTK_SCROLLED_WINDOW(widget));

        viewer_window->dragging.start_x = event->x_root;
        viewer_window->dragging.start_y = event->y_root;
        viewer_window->dragging.hadj_value = gtk_adjustment_get_value(hadj);
        viewer_window->dragging.vadj_value = gtk_adjustment_get_value(vadj);
        viewer_window->dragging.is_dragging = TRUE;

        GdkDisplay *display = gtk_widget_get_display(widget);
        GdkCursor *cursor = gdk_cursor_new_from_name(display, "grabbing");
        gdk_window_set_cursor(gtk_widget_get_window(widget), cursor);
        g_object_unref(cursor);

        return TRUE;
    }

    return FALSE;

}


gboolean on_mouse_motion(GtkWidget *widget, GdkEventMotion *event, struct viewer_window_ *viewer_window) {

    if (viewer_window->dragging.is_dragging) {

        GtkAdjustment *hadj = gtk_scrolled_window_get_hadjustment(GTK_SCROLLED_WINDOW(widget));
        GtkAdjustment *vadj = gtk_scrolled_window_get_vadjustment(GTK_SCROLLED_WINDOW(widget));

        gdouble dx = event->x_root - viewer_window->dragging.start_x;
        gdouble dy = event->y_root - viewer_window->dragging.start_y;

        gtk_adjustment_set_value(hadj, viewer_window->dragging.hadj_value - dx);
        gtk_adjustment_set_value(vadj, viewer_window->dragging.vadj_value - dy);

    }

    return FALSE;
}


gboolean on_mouse_release(GtkWidget *widget, GdkEventButton *event, struct viewer_window_ *viewer_window) {
    if (event->button == GDK_BUTTON_PRIMARY) {

        viewer_window->dragging.is_dragging = FALSE;
        gdk_window_set_cursor(gtk_widget_get_window(widget), NULL);

        return TRUE;
    }
    return FALSE;
}


void window_quit(GtkWidget *widget, gpointer data){

    struct widget_and_Id* wind_id;
    wind_id = (struct widget_and_Id*)data;
    wind_id->window->view.erase(wind_id->id);

    delete wind_id;
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

}


void get_files(GtkFlowBoxChild *child,struct widget_and_Id *for_list){

    GList *children = gtk_container_get_children(GTK_CONTAINER(child));
    gpointer widget = g_list_nth_data(children,0);

    int ind = gops::findStringIndex((for_list->window->ipts.f_list),gtk_widget_get_name(GTK_WIDGET( widget)));

    if (ind >= 0){

        for_list->window->view[for_list->id].panorama->add_images(for_list->window->ipts.f_list[ind]);

    }
    else{
            std::cout<< gtk_widget_get_name (GTK_WIDGET( widget))<<" not found";
    }

    g_list_free(g_steal_pointer (&children));
}


void create_viewer(struct main_window_ *main_window,GtkWidget* self,GdkEventButton *event){

    //ensure only 1 window of the same type is created
    int test = main_window->view_number;
    main_window->view_number++;
    if (main_window->view.count(test)){return;}

    struct viewer_window_ viewer;
    viewer.window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    main_window->view[test] = std::move(viewer);
    std::vector<std::string> files;
    //pan::panorama pan(files);
    auto ptr = std::make_unique<pan::panorama>(files);
    main_window->view[test].panorama = std::move(ptr);

    GList* s_list = gtk_flow_box_get_selected_children(GTK_FLOW_BOX( main_window->flowbox.flowbox_main));
    struct widget_and_Id for_list;
    for_list.id = test;
    for_list.window = main_window;
    g_list_foreach(s_list,(GFunc)get_files,&for_list);
    struct pan::config _CONF_;

    pbar::progress_bar_ *progress_bar = g_new0(pbar::progress_bar_, 1);


    pbar::open_progress_bar(main_window->window,progress_bar,main_window);

    progress_bar->bar_timer_id = g_timeout_add(200, pbar::update_progress, progress_bar);


    main_window->view[test].panorama->stitch_panorama(8,_CONF_);
/*
    auto handle = std::async(std::launch::async, [&ptr](){
            return ptr->foo(); // Ofcourse make foo public in your snippet
    });


    std::thread stitch_thread = std::thread(&pan::panorama::stitch_panorama,&main_window->view[test].panorama, 8,_CONF_);


    if (stitch_thread.joinable()) {
        stitch_thread.join();
    }
*/
    //adding box
    gtk_window_set_default_size(GTK_WINDOW(main_window->view[test].window), 800, 600);
    main_window->view[test].viewer_box = gtk_box_new (GTK_ORIENTATION_VERTICAL,0);
    gtk_box_set_homogeneous (GTK_BOX(main_window->view[test].viewer_box),FALSE);
    gtk_container_add (GTK_CONTAINER(main_window->view[test].window),main_window->view[test].viewer_box);

    main_window->view[test].viewer_scrolled_window = gtk_scrolled_window_new (NULL, NULL);
    main_window->view[test].viewer_scrolled_window_viewpoint = gtk_viewport_new (NULL, NULL);
    gtk_container_add(GTK_CONTAINER(main_window->view[test].viewer_scrolled_window), main_window->view[test].viewer_scrolled_window_viewpoint);
    gtk_box_pack_end(GTK_BOX(main_window->view[test].viewer_box),main_window->view[test].viewer_scrolled_window,TRUE,TRUE,0);

    main_window->view[test].image = main_window->view[test].panorama->get_preview();
    main_window->view[test].panorama.reset();
    //main_window->view[test].image = cv::imread("/home/sd_bert/Pictures/1725014833368114.png", cv::IMREAD_COLOR);

    if ( 200 < (main_window->view[test].image.cols - 800)){

        float z_num = (main_window->view[test].image.cols - 800)/200;
        if (z_num < 3){

            main_window->view[test].zoom_val.resize(2);
            main_window->view[test].zoom_val[0] = 800;
            main_window->view[test].zoom_val[1] = main_window->view[test].image.cols;

        }else{

            int zooms = (int)z_num;
            main_window->view[test].zoom_val.resize(zooms+1);
            main_window->view[test].zoom_val[0] = 800;
            main_window->view[test].zoom_val[zooms] = main_window->view[test].image.cols;
            for(int i = 1; i < zooms;i++){

                main_window->view[test].zoom_val[i] = 800 + 200*i;
            }

        }

    }

    if (1 > main_window->view[test].zoom_val.size()){

        main_window->view[test].image_zoomed = main_window->view[test].image;
        main_window->view[test].current_zoom = -1;

    }else{

        main_window->view[test].image_zoomed = imgm::resizeKeepAspectRatio(main_window->view[test].image, main_window->view[test].zoom_val[1]);
        main_window->view[test].current_zoom = 1;

    }

    main_window->view[test].img_test = gops::cv_image_to_gtk_image(main_window->view[test].image_zoomed);

    main_window->view[test].image_window_event = gtk_event_box_new();
    gtk_container_add(GTK_CONTAINER(main_window->view[test].image_window_event), GTK_WIDGET(main_window->view[test].img_test));

    gtk_container_add(GTK_CONTAINER(main_window->view[test].viewer_scrolled_window_viewpoint),main_window->view[test].image_window_event);

    gtk_widget_add_events(main_window->view[test].image_window_event, GDK_BUTTON_PRESS_MASK bitor GDK_BUTTON_RELEASE_MASK bitor GDK_POINTER_MOTION_MASK);

    imgvt::create_toolbar(main_window->view[test].viewer_box,&(main_window->view[test].toolbar),&(main_window->view[test]));

    gtk_widget_show_all(main_window->view[test].window);
    connect_signals(main_window,test);

}


}


