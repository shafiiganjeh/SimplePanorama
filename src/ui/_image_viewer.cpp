#include "_image_viewer.h"

namespace imgv{

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

}


void get_files(GtkFlowBoxChild *child,struct main_window_ *main_window){

        GList *children = gtk_container_get_children(GTK_CONTAINER(child));
        gpointer widget = g_list_nth_data (children,0);

        int ind = gops::findStringIndex((main_window->ipts.f_list),gtk_widget_get_name (GTK_WIDGET( widget)));

        if (ind >= 0){
              gtk_widget_destroy(GTK_WIDGET(child));
              main_window->ipts.f_list.erase(main_window->ipts.f_list.begin() + ind);
              main_window->ipts.img_data.erase(main_window->ipts.img_data.begin() + ind);
              gtk_widget_show_all(main_window->flowbox.flowbox_main);
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

    GList* s_list = gtk_flow_box_get_selected_children(GTK_FLOW_BOX( main_window->flowbox.flowbox_main));

    g_list_foreach(s_list,(GFunc)get_files,main_window);


    struct viewer_window_ viewer;
    viewer.window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    main_window->view[test] = viewer;

    //adding box
    gtk_window_set_default_size(GTK_WINDOW(main_window->view[test].window), 800, 600);
    main_window->view[test].viewer_box = gtk_box_new (GTK_ORIENTATION_VERTICAL,0);
    gtk_box_set_homogeneous (GTK_BOX(main_window->view[test].viewer_box),FALSE);
    gtk_container_add (GTK_CONTAINER(main_window->view[test].window),main_window->view[test].viewer_box);

    //adding scrolled viewpoint at the end of box
//for later
    //main_window->view[test].image = cv::imread("/home/sd_bert/Pictures/1732466448802106.jpg", cv::IMREAD_COLOR);
    //GtkWidget *drawing_area = gtk_drawing_area_new ();
    //gtk_widget_set_size_request (drawing_area, main_window->view[test].image.cols, main_window->view[test].image.cols.rows);

    main_window->view[test].viewer_scrolled_window = gtk_scrolled_window_new (NULL, NULL);
    main_window->view[test].viewer_scrolled_window_viewpoint = gtk_viewport_new (NULL, NULL);
    gtk_container_add(GTK_CONTAINER(main_window->view[test].viewer_scrolled_window), main_window->view[test].viewer_scrolled_window_viewpoint);
    gtk_box_pack_end (GTK_BOX(main_window->view[test].viewer_box),main_window->view[test].viewer_scrolled_window,TRUE,TRUE,0);

    main_window->view[test].image = cv::imread("/home/sd_bert/Pictures/1725014833368114.png", cv::IMREAD_COLOR);

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

    gtk_container_add (GTK_CONTAINER(main_window->view[test].viewer_scrolled_window_viewpoint),GTK_WIDGET(main_window->view[test].img_test));

//for later
    //gtk_container_add (GTK_CONTAINER(main_window->view[test].viewer_scrolled_window_viewpoint),GTK_WIDGET(drawing_area));

    imgvt::create_toolbar(main_window->view[test].viewer_box,&(main_window->view[test].toolbar),&(main_window->view[test]));

    gtk_widget_show_all(main_window->view[test].window);
    connect_signals(main_window,test);

}


}


