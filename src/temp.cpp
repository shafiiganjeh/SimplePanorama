    gboolean return_rect(GtkWidget* self,GdkEventButton *event,struct viewer_window_ *main_window){

        int image_w = main_window->image_zoomed.cols;
        int image_h = main_window->image_zoomed.rows;
        int image_w_norm = main_window->crop_preview.width;
        int image_h_norm = main_window->crop_preview.height;

        double ratio = ((double)image_w_norm/(double)image_w + (double)image_h_norm/(double)image_h)/2;


        cv::Rect image(0,0,image_w,image_h);
        cv::Rect window = main_window->dragging.final_drawing;
        cv::Rect ROI = image bitand window;

        ROI = util::scaleRect(ROI,ratio,ratio); // rescale rect from zoomed to prev

        cv::imshow("Image Display", main_window->image(ROI));
        cv::waitKey(0);

/*
        main_window->crop_preview = ROI;
        main_window->image_zoomed = imgm::resizeKeepAspectRatio((main_window->image)(main_window->crop_preview), main_window->zoom_val[main_window->current_zoom]);
        gtk_widget_destroy (GTK_WIDGET(main_window->img_test));
        main_window->img_test = gops::cv_image_to_gtk_image(main_window->image_zoomed);
        gtk_container_add(GTK_CONTAINER(main_window->image_window_event), GTK_WIDGET(main_window->img_test));
        gtk_widget_show_all(main_window->window);
*/


        //cv::Mat test = main_window->panorama_->get_panorama(ROI);
        //cv::imshow("Image Display", test);
        //cv::waitKey(0);

        return TRUE;

    }



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
