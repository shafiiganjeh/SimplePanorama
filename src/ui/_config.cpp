
#include "_config.h"

namespace conf{

    void entry_settings(GtkWidget *row,GtkWidget *hbox,GtkWidget *label,GtkWidget * insert_to,GtkWidget * entry_address){

        row = gtk_list_box_row_new();
        gtk_widget_set_margin_top(row, 3);
        gtk_widget_set_margin_bottom(row, 3);
        hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 10);
        gtk_container_set_border_width(GTK_CONTAINER(hbox), 5);
        gtk_entry_set_input_purpose (GTK_ENTRY(entry_address),GTK_INPUT_PURPOSE_NUMBER);
        gtk_label_set_xalign(GTK_LABEL(label), 0.0);
        gtk_box_pack_start(GTK_BOX(hbox), label, TRUE, TRUE, 0);
        gtk_box_pack_start(GTK_BOX(hbox), entry_address, FALSE, FALSE, 0);
        gtk_container_add(GTK_CONTAINER(row), hbox);
        gtk_list_box_insert(GTK_LIST_BOX(insert_to), row, -1);

    }


    gboolean cancel(GtkWidget* self,GdkEventButton *event,struct config_* config_window){

        gtk_window_close (GTK_WINDOW(config_window->conf_menu));
        return FALSE;

    }

    std::string e2str(const gchar* entry){
        std::string e2str(entry);
        std::replace( e2str.begin(), e2str.end(), ',', '.');
        //std::cout <<"\n"<<e2str<<"\n";
        return e2str;
    }

    gboolean OK(GtkWidget* self,GdkEventButton *event,struct config_* config_window){

        GtkTreeIter iter;
        if(gtk_combo_box_get_active_iter(GTK_COMBO_BOX(config_window->conf_menu_stack_basic_frame_method_combo), &iter)){

            gchar *entry;
            GtkTreeModel* model = gtk_combo_box_get_model(GTK_COMBO_BOX(config_window->conf_menu_stack_basic_frame_method_combo));
            gtk_tree_model_get(model, &iter,0,&entry,-1);
            std::string str(entry);
            config_window->config_->blend = static_cast<pan::Blending>(pan::StringToBlending(str));
            g_free(entry);

        }
        const gchar* entry;


        config_window->config_->cut = gtk_switch_get_state(GTK_SWITCH(config_window->conf_menu_stack_basic_frame_method_switch_cut));

        config_window->config_->cut_seams = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(config_window->conf_menu_stack_basic_frame_method_switch_cut_yesno));

        config_window->config_->gain_compensation = gtk_switch_get_state(GTK_SWITCH(config_window->conf_menu_stack_basic_frame_method_switch_gain));

        entry = gtk_entry_get_text(GTK_ENTRY(config_window->conf_menu_stack_basic_frame_method_entry_sigma));
        config_window->config_->sigma_blend = util::stringToInt(e2str(entry));
        entry = gtk_entry_get_text(GTK_ENTRY(config_window->conf_menu_stack_basic_frame_method_entry_bands));
        config_window->config_->bands = util::stringToInt(e2str(entry));

        entry = gtk_entry_get_text(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_threads));
        config_window->config_->threads = util::stringToInt(e2str(entry));
        entry = gtk_entry_get_text(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_size));
        config_window->config_->init_size = util::stringToInt(e2str(entry));
        entry = gtk_entry_get_text(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_maximages));
        config_window->config_->max_images_per_match = util::stringToInt(e2str(entry));
        entry = gtk_entry_get_text(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_maxkeypoints));
        config_window->config_->max_keypoints = util::stringToInt(e2str(entry));
        entry = gtk_entry_get_text(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_RANSAC));
        config_window->config_->RANSAC_iterations = util::stringToInt(e2str(entry));
        entry = gtk_entry_get_text(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_xmargin));
        config_window->config_->x_margin = util::stringToInt(e2str(entry));
        entry = gtk_entry_get_text(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_minoverlap));
        config_window->config_->min_overlap = util::stringToFloat(e2str(entry));
        entry = gtk_entry_get_text(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_overlapinlmatch));
        config_window->config_->overlap_inl_match = util::stringToFloat(e2str(entry));
        entry = gtk_entry_get_text(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_overlapinlkeyp));
        config_window->config_->overlap_inl_keyp = util::stringToFloat(e2str(entry));
        entry = gtk_entry_get_text(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_conf));
        config_window->config_->conf = util::stringToFloat(e2str(entry));


        entry = gtk_entry_get_text(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_nfeatures));
        config_window->config_->nfeatures = util::stringToInt(e2str(entry));
        entry = gtk_entry_get_text(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_nOctaveLayers));
        config_window->config_->nOctaveLayers = util::stringToInt(e2str(entry));
        entry = gtk_entry_get_text(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_contrastThreshold));
        config_window->config_->contrastThreshold = util::stringToDouble(e2str(entry));
        entry = gtk_entry_get_text(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_edgeThreshold));

        config_window->config_->edgeThreshold = util::stringToDouble(e2str(entry));
        entry = gtk_entry_get_text(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_sigma));
        config_window->config_->sigma_sift = util::stringToDouble(e2str(entry));

        entry = gtk_entry_get_text(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_focal));
        config_window->config_->focal = util::stringToFloat(e2str(entry));
        entry = gtk_entry_get_text(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_lambda));
        config_window->config_->lambda = util::stringToFloat(e2str(entry));

        conf::ConfigParser test(config_window->config_);
        test.write_cfg(config_window->_path_conf);
        return FALSE;

    }


    void insert_text_event_float(GtkEditable *editable, const gchar *text, gint length, gint *position, gpointer data){
        int i;
        for (i = 0; i < length; i++) {
            if((std::strcmp(&text[i], ".") == 0) or (std::strcmp(&text[i], ",") == 0)){
                return;
            }else if (!isdigit(text[i])) {
                g_signal_stop_emission_by_name(G_OBJECT(editable), "insert-text");
                return;
            }
        }
    }


    void insert_text_event_int(GtkEditable *editable, const gchar *text, gint length, gint *position, gpointer data){
        int i;
        for (i = 0; i < length; i++) {
            if (!isdigit(text[i])) {
                g_signal_stop_emission_by_name(G_OBJECT(editable), "insert-text");
                return;
            }
        }
    }

    void on_combo_box_changed(GtkComboBox* combo_box, struct config_* config_window) {
        GtkTreeIter iter;
        if(gtk_combo_box_get_active_iter(combo_box, &iter)){

            gchar *entry;
            GtkTreeModel* model = gtk_combo_box_get_model(combo_box);

            gtk_tree_model_get(model, &iter,0,&entry,-1);

            GtkListBoxRow* lst3 = gtk_list_box_get_row_at_index (GTK_LIST_BOX(config_window->conf_menu_stack_basic_frame_method_box),3);
            GtkListBoxRow* lst4 = gtk_list_box_get_row_at_index (GTK_LIST_BOX(config_window->conf_menu_stack_basic_frame_method_box),4);

            if((std::strcmp(entry, "MULTI_BLEND") == 0)){

                gtk_widget_show (GTK_WIDGET(lst3));
                gtk_widget_show (GTK_WIDGET(lst4));

            }else{

                gtk_widget_hide (GTK_WIDGET(lst3));
                gtk_widget_hide (GTK_WIDGET(lst4));

            }

            g_free(entry);

        }

    }

    void conf_blending_settings(struct config_* config_window,struct main_window_ *main_window){

        config_window->config_ = main_window->config_;
        config_window->conf_menu_stack_basic_scrolled_window = gtk_scrolled_window_new(NULL, NULL);
        gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(config_window->conf_menu_stack_basic_scrolled_window),
                                    GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
        config_window->conf_menu_stack_basic_viewport = gtk_viewport_new(NULL, NULL);
        gtk_container_add(GTK_CONTAINER(config_window->conf_menu_stack_basic_scrolled_window),
                        config_window->conf_menu_stack_basic_viewport);
        config_window->conf_menu_stack_basic_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
        gtk_container_add(GTK_CONTAINER(config_window->conf_menu_stack_basic_viewport),
                        config_window->conf_menu_stack_basic_box);
        gtk_stack_add_titled(GTK_STACK(config_window->conf_menu_stack),
                        config_window->conf_menu_stack_basic_scrolled_window,
                        "blending_", "Blending");

        config_window->conf_menu_stack_basic_frame_method = gtk_frame_new ("Blending Method");

        gtk_box_pack_start (GTK_BOX(config_window->conf_menu_stack_basic_box),config_window->conf_menu_stack_basic_frame_method,FALSE,FALSE,5);

        GtkListStore *store_list;
        GtkTreeIter method_iter;
        store_list = gtk_list_store_new(1, G_TYPE_STRING);

        std::vector<GtkCellRenderer *>().swap(config_window->method_combo_renderer);
        for(int i = 0 ; i < pan::_enum_sizeoff_;i++){
            gtk_list_store_append(store_list, &method_iter);
            gtk_list_store_set(store_list, &method_iter,0,pan::BlendingToString(i),-1);
            GtkCellRenderer* r = gtk_cell_renderer_text_new();
            config_window->method_combo_renderer.push_back(r);
        }

        config_window->conf_menu_stack_basic_frame_method_combo = gtk_combo_box_new_with_model(GTK_TREE_MODEL(store_list));

        for(int i = 0 ; i < pan::_enum_sizeoff_;i++){
            gtk_cell_layout_pack_start(GTK_CELL_LAYOUT(config_window->conf_menu_stack_basic_frame_method_combo), config_window->method_combo_renderer[i], TRUE);
        }

        gtk_cell_layout_set_attributes(GTK_CELL_LAYOUT(config_window->conf_menu_stack_basic_frame_method_combo), config_window->method_combo_renderer[0],
                                  "text", 0, NULL);

        config_window->conf_menu_stack_basic_frame_method_box = gtk_list_box_new ();
        gtk_list_box_set_selection_mode(GTK_LIST_BOX(config_window->conf_menu_stack_basic_frame_method_box), GTK_SELECTION_NONE);
        GtkWidget *row;
        GtkWidget *hbox;
        GtkWidget *label;
        std::string varAsString;
        struct util::val ret;

        row = gtk_list_box_row_new();
        gtk_widget_set_margin_top(row, 15);
        gtk_widget_set_margin_bottom(row, 5);
        gtk_container_add(GTK_CONTAINER(row), config_window->conf_menu_stack_basic_frame_method_combo);
        gtk_list_box_insert(GTK_LIST_BOX(config_window->conf_menu_stack_basic_frame_method_box), row, -1);


        row = gtk_list_box_row_new();
        gtk_widget_set_margin_top(row, 3);
        gtk_widget_set_margin_bottom(row, 3);
        hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 10);
        gtk_container_set_border_width(GTK_CONTAINER(hbox), 5);

        label = gtk_label_new("Cut Image seams");
        gtk_label_set_xalign(GTK_LABEL(label), 0.0);
        config_window->conf_menu_stack_basic_frame_method_switch_cut_yesno = gtk_check_button_new ();
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(config_window->conf_menu_stack_basic_frame_method_switch_cut_yesno),main_window->config_->cut_seams);
        gtk_box_pack_start(GTK_BOX(hbox), label, FALSE, FALSE, 0);
        gtk_box_pack_start(GTK_BOX(hbox), config_window->conf_menu_stack_basic_frame_method_switch_cut_yesno, FALSE, FALSE, 0);

        config_window->conf_menu_stack_basic_frame_method_switch_cut = gtk_switch_new ();
        gtk_switch_set_state (GTK_SWITCH(config_window->conf_menu_stack_basic_frame_method_switch_cut),main_window->config_->cut);
        gtk_widget_set_margin_end(config_window->conf_menu_stack_basic_frame_method_switch_cut,30);
        label = gtk_label_new("Find Image seams using content aware cutting");
        gtk_label_set_xalign(GTK_LABEL(label), .8);

        gtk_box_pack_start(GTK_BOX(hbox), label, TRUE, TRUE, 0);
        gtk_box_pack_start(GTK_BOX(hbox), config_window->conf_menu_stack_basic_frame_method_switch_cut, FALSE, FALSE, 0);

        gtk_container_add(GTK_CONTAINER(row), hbox);
        gtk_list_box_insert(GTK_LIST_BOX(config_window->conf_menu_stack_basic_frame_method_box), row, -1);
        g_signal_connect(config_window->conf_menu_stack_basic_frame_method_combo, "changed", G_CALLBACK(on_combo_box_changed), config_window);


        row = gtk_list_box_row_new();
        gtk_widget_set_margin_top(row, 3);
        gtk_widget_set_margin_bottom(row, 3);
        hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 10);
        gtk_container_set_border_width(GTK_CONTAINER(hbox), 5);
        config_window->conf_menu_stack_basic_frame_method_switch_gain = gtk_switch_new ();
        gtk_switch_set_state (GTK_SWITCH(config_window->conf_menu_stack_basic_frame_method_switch_gain),main_window->config_->gain_compensation);
        gtk_widget_set_margin_end(config_window->conf_menu_stack_basic_frame_method_switch_gain,30);
        label = gtk_label_new("Equalize Image contrast");
        gtk_label_set_xalign(GTK_LABEL(label), 0.0);
        gtk_box_pack_start(GTK_BOX(hbox), label, TRUE, TRUE, 0);
        gtk_box_pack_start(GTK_BOX(hbox), config_window->conf_menu_stack_basic_frame_method_switch_gain, FALSE, FALSE, 0);
        gtk_container_add(GTK_CONTAINER(row), hbox);
        gtk_list_box_insert(GTK_LIST_BOX(config_window->conf_menu_stack_basic_frame_method_box), row, -1);


        config_window->conf_menu_stack_basic_frame_method_entry_sigma = gtk_entry_new ();
        ret = util::processValue(main_window->config_->sigma_blend, 100);
        varAsString = std::to_string(ret.double_part);
        gtk_entry_set_text (GTK_ENTRY(config_window->conf_menu_stack_basic_frame_method_entry_sigma),varAsString.c_str());
        gtk_entry_set_input_purpose (GTK_ENTRY(config_window->conf_menu_stack_basic_frame_method_entry_sigma),GTK_INPUT_PURPOSE_NUMBER);
        gtk_entry_set_max_length(GTK_ENTRY(config_window->conf_menu_stack_basic_frame_method_entry_sigma),8);
        label = gtk_label_new("Sigma value (higher values blend more but can be blurry)");
        entry_settings(row,hbox,label,config_window->conf_menu_stack_basic_frame_method_box,config_window->conf_menu_stack_basic_frame_method_entry_sigma);
        g_signal_connect(G_OBJECT(config_window->conf_menu_stack_basic_frame_method_entry_sigma), "insert-text", G_CALLBACK(insert_text_event_float), NULL);


        config_window->conf_menu_stack_basic_frame_method_entry_bands = gtk_entry_new ();
        ret = util::processValue(main_window->config_->bands, 9);
        varAsString = std::to_string(ret.int_part);
        gtk_entry_set_text (GTK_ENTRY(config_window->conf_menu_stack_basic_frame_method_entry_bands),varAsString.c_str());
        gtk_entry_set_input_purpose (GTK_ENTRY(config_window->conf_menu_stack_basic_frame_method_entry_bands),GTK_INPUT_PURPOSE_NUMBER);
        gtk_entry_set_max_length(GTK_ENTRY(config_window->conf_menu_stack_basic_frame_method_entry_bands),1);
        label = gtk_label_new("Number of Bands");
        entry_settings(row,hbox,label,config_window->conf_menu_stack_basic_frame_method_box,config_window->conf_menu_stack_basic_frame_method_entry_bands);
        g_signal_connect(G_OBJECT(config_window->conf_menu_stack_basic_frame_method_entry_bands), "insert-text", G_CALLBACK(insert_text_event_int), NULL);


        gtk_container_add(GTK_CONTAINER(config_window->conf_menu_stack_basic_frame_method),config_window->conf_menu_stack_basic_frame_method_box);
        gtk_combo_box_set_active(GTK_COMBO_BOX(config_window->conf_menu_stack_basic_frame_method_combo), main_window->config_->blend);

    }

    void conf_advanced_settings(struct config_* config_window,struct main_window_ *main_window){

        config_window->conf_menu_stack_advanced_scrolled_window = gtk_scrolled_window_new(NULL, NULL);
        gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(config_window->conf_menu_stack_advanced_scrolled_window),GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
        config_window->conf_menu_stack_advanced_viewport = gtk_viewport_new(NULL, NULL);
        gtk_container_add(GTK_CONTAINER(config_window->conf_menu_stack_advanced_scrolled_window),config_window->conf_menu_stack_advanced_viewport);
        config_window->conf_menu_stack_advanced_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
        gtk_container_add(GTK_CONTAINER(config_window->conf_menu_stack_advanced_viewport),config_window->conf_menu_stack_advanced_box);
        gtk_stack_add_titled(GTK_STACK(config_window->conf_menu_stack),config_window->conf_menu_stack_advanced_scrolled_window,"advanced_settings", "Advanced Settings");

        GtkWidget *row;
        GtkWidget *hbox;
        GtkWidget *label;
        std::string varAsString;
        struct util::val ret;

        //system.............................
        config_window->conf_menu_stack_advanced_frame_system = gtk_frame_new ("System");
        gtk_box_pack_start (GTK_BOX(config_window->conf_menu_stack_advanced_box),config_window->conf_menu_stack_advanced_frame_system,FALSE,FALSE,5);
        config_window->conf_menu_stack_advanced_frame_system_box = gtk_list_box_new ();
        gtk_list_box_set_selection_mode(GTK_LIST_BOX(config_window->conf_menu_stack_advanced_frame_system_box), GTK_SELECTION_NONE);

        config_window->conf_menu_stack_advanced_frame_system_entry_threads = gtk_entry_new ();
        ret = util::processValue(main_window->config_->threads, 64);
        varAsString = std::to_string(ret.int_part);
        gtk_entry_set_text (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_threads),varAsString.c_str());
        gtk_entry_set_input_purpose (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_threads),GTK_INPUT_PURPOSE_NUMBER);
        gtk_entry_set_max_length(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_threads),2);
        label = gtk_label_new("Max number of Threads)");
        entry_settings(row,hbox,label,config_window->conf_menu_stack_advanced_frame_system_box,config_window->conf_menu_stack_advanced_frame_system_entry_threads);
        g_signal_connect(G_OBJECT(config_window->conf_menu_stack_advanced_frame_system_entry_threads), "insert-text", G_CALLBACK(insert_text_event_int), NULL);


        config_window->conf_menu_stack_advanced_frame_system_entry_size = gtk_entry_new ();
        ret = util::processValue(main_window->config_->init_size, 8000);
        varAsString = std::to_string(ret.int_part);
        gtk_entry_set_text (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_size),varAsString.c_str());
        gtk_entry_set_input_purpose (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_size),GTK_INPUT_PURPOSE_NUMBER);
        gtk_entry_set_max_length(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_size),4);
        label = gtk_label_new("Max resize resolution");
        entry_settings(row,hbox,label,config_window->conf_menu_stack_advanced_frame_system_box,config_window->conf_menu_stack_advanced_frame_system_entry_size);
        g_signal_connect(G_OBJECT(config_window->conf_menu_stack_advanced_frame_system_entry_size), "insert-text", G_CALLBACK(insert_text_event_int), NULL);


        gtk_container_add(GTK_CONTAINER(config_window->conf_menu_stack_advanced_frame_system),config_window->conf_menu_stack_advanced_frame_system_box);


        //matching.............................
        config_window->conf_menu_stack_advanced_frame_matching = gtk_frame_new ("Matching");
        gtk_box_pack_start (GTK_BOX(config_window->conf_menu_stack_advanced_box),config_window->conf_menu_stack_advanced_frame_matching,FALSE,FALSE,5);
        config_window->conf_menu_stack_advanced_frame_matching_box = gtk_list_box_new ();
        gtk_list_box_set_selection_mode(GTK_LIST_BOX(config_window->conf_menu_stack_advanced_frame_matching_box), GTK_SELECTION_NONE);

        config_window->conf_menu_stack_advanced_frame_matching_entry_maximages = gtk_entry_new ();
        ret = util::processValue(main_window->config_->max_images_per_match, 1000);
        varAsString = std::to_string(ret.int_part);
        gtk_entry_set_text (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_maximages),varAsString.c_str());
        gtk_entry_set_input_purpose (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_maximages),GTK_INPUT_PURPOSE_NUMBER);
        gtk_entry_set_max_length(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_maximages),4);
        label = gtk_label_new("Max number of image pairs");
        entry_settings(row,hbox,label,config_window->conf_menu_stack_advanced_frame_matching_box,config_window->conf_menu_stack_advanced_frame_matching_entry_maximages);
        g_signal_connect(G_OBJECT(config_window->conf_menu_stack_advanced_frame_matching_entry_maximages), "insert-text", G_CALLBACK(insert_text_event_int), NULL);

        config_window->conf_menu_stack_advanced_frame_matching_entry_maxkeypoints = gtk_entry_new ();
        ret = util::processValue(main_window->config_->max_keypoints, 99999);
        varAsString = std::to_string(ret.int_part);
        gtk_entry_set_text (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_maxkeypoints),varAsString.c_str());
        gtk_entry_set_input_purpose (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_maxkeypoints),GTK_INPUT_PURPOSE_NUMBER);
        gtk_entry_set_max_length(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_maxkeypoints),5);
        label = gtk_label_new("Max number of keypoints");
        entry_settings(row,hbox,label,config_window->conf_menu_stack_advanced_frame_matching_box,config_window->conf_menu_stack_advanced_frame_matching_entry_maxkeypoints);
        g_signal_connect(G_OBJECT(config_window->conf_menu_stack_advanced_frame_matching_entry_maxkeypoints), "insert-text", G_CALLBACK(insert_text_event_int), NULL);

        config_window->conf_menu_stack_advanced_frame_matching_entry_RANSAC = gtk_entry_new ();
        ret = util::processValue(main_window->config_->RANSAC_iterations, 9999);
        varAsString = std::to_string(ret.int_part);
        gtk_entry_set_text (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_RANSAC),varAsString.c_str());
        gtk_entry_set_input_purpose (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_RANSAC),GTK_INPUT_PURPOSE_NUMBER);
        gtk_entry_set_max_length(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_RANSAC),4);
        label = gtk_label_new("RANSAC iterations");
        entry_settings(row,hbox,label,config_window->conf_menu_stack_advanced_frame_matching_box,config_window->conf_menu_stack_advanced_frame_matching_entry_RANSAC);
        g_signal_connect(G_OBJECT(config_window->conf_menu_stack_advanced_frame_matching_entry_RANSAC), "insert-text", G_CALLBACK(insert_text_event_int), NULL);

        config_window->conf_menu_stack_advanced_frame_matching_entry_xmargin = gtk_entry_new ();
        ret = util::processValue(main_window->config_->x_margin, 9999);
        varAsString = std::to_string(ret.int_part);
        gtk_entry_set_text (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_xmargin),varAsString.c_str());
        gtk_entry_set_input_purpose (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_xmargin),GTK_INPUT_PURPOSE_NUMBER);
        gtk_entry_set_max_length(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_xmargin),4);
        label = gtk_label_new("Keypoint inlier margin");
        entry_settings(row,hbox,label,config_window->conf_menu_stack_advanced_frame_matching_box,config_window->conf_menu_stack_advanced_frame_matching_entry_xmargin);
        g_signal_connect(G_OBJECT(config_window->conf_menu_stack_advanced_frame_matching_entry_xmargin), "insert-text", G_CALLBACK(insert_text_event_float), NULL);

        config_window->conf_menu_stack_advanced_frame_matching_entry_minoverlap = gtk_entry_new ();
        ret = util::processValue(main_window->config_->min_overlap, 1);
        varAsString = std::to_string(ret.double_part);
        gtk_entry_set_text (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_minoverlap),varAsString.c_str());
        gtk_entry_set_input_purpose (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_minoverlap),GTK_INPUT_PURPOSE_NUMBER);
        gtk_entry_set_max_length(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_minoverlap),6);
        label = gtk_label_new("Imag-wise minimum overlap area");
        entry_settings(row,hbox,label,config_window->conf_menu_stack_advanced_frame_matching_box,config_window->conf_menu_stack_advanced_frame_matching_entry_minoverlap);
        g_signal_connect(G_OBJECT(config_window->conf_menu_stack_advanced_frame_matching_entry_minoverlap), "insert-text", G_CALLBACK(insert_text_event_float), NULL);

        config_window->conf_menu_stack_advanced_frame_matching_entry_overlapinlmatch = gtk_entry_new ();
        ret = util::processValue(main_window->config_->overlap_inl_match, 1);
        varAsString = std::to_string(ret.double_part);
        gtk_entry_set_text (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_overlapinlmatch),varAsString.c_str());
        gtk_entry_set_input_purpose (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_overlapinlmatch),GTK_INPUT_PURPOSE_NUMBER);
        gtk_entry_set_max_length(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_overlapinlmatch),6);
        label = gtk_label_new("Imag-wise min inlier/match-Ratio");
        entry_settings(row,hbox,label,config_window->conf_menu_stack_advanced_frame_matching_box,config_window->conf_menu_stack_advanced_frame_matching_entry_overlapinlmatch);
        g_signal_connect(G_OBJECT(config_window->conf_menu_stack_advanced_frame_matching_entry_overlapinlmatch), "insert-text", G_CALLBACK(insert_text_event_float), NULL);

        config_window->conf_menu_stack_advanced_frame_matching_entry_overlapinlkeyp = gtk_entry_new ();
        ret = util::processValue(main_window->config_->overlap_inl_keyp, 1);
        varAsString = std::to_string(ret.double_part);
        gtk_entry_set_text (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_overlapinlkeyp),varAsString.c_str());
        gtk_entry_set_input_purpose (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_overlapinlkeyp),GTK_INPUT_PURPOSE_NUMBER);
        gtk_entry_set_max_length(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_overlapinlkeyp),6);
        label = gtk_label_new("Imag-wise min inlier/keypoint-Ratio");
        entry_settings(row,hbox,label,config_window->conf_menu_stack_advanced_frame_matching_box,config_window->conf_menu_stack_advanced_frame_matching_entry_overlapinlkeyp);
        g_signal_connect(G_OBJECT(config_window->conf_menu_stack_advanced_frame_matching_entry_overlapinlkeyp), "insert-text", G_CALLBACK(insert_text_event_float), NULL);

        config_window->conf_menu_stack_advanced_frame_matching_entry_conf = gtk_entry_new ();
        ret = util::processValue(main_window->config_->conf, 1);
        varAsString = std::to_string(ret.double_part);
        gtk_entry_set_text (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_conf),varAsString.c_str());
        gtk_entry_set_input_purpose (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_conf),GTK_INPUT_PURPOSE_NUMBER);
        gtk_entry_set_max_length(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_matching_entry_conf),6);
        label = gtk_label_new("Min total inlier/keypoint-Ratio");
        entry_settings(row,hbox,label,config_window->conf_menu_stack_advanced_frame_matching_box,config_window->conf_menu_stack_advanced_frame_matching_entry_conf);
        g_signal_connect(G_OBJECT(config_window->conf_menu_stack_advanced_frame_matching_entry_conf), "insert-text", G_CALLBACK(insert_text_event_float), NULL);


        gtk_container_add(GTK_CONTAINER(config_window->conf_menu_stack_advanced_frame_matching),config_window->conf_menu_stack_advanced_frame_matching_box);


        //SIFT.............................
        config_window->conf_menu_stack_advanced_frame_SIFT = gtk_frame_new ("SIFT Settings");
        gtk_box_pack_start (GTK_BOX(config_window->conf_menu_stack_advanced_box),config_window->conf_menu_stack_advanced_frame_SIFT,FALSE,FALSE,5);
        config_window->conf_menu_stack_advanced_frame_SIFT_box = gtk_list_box_new ();
        gtk_list_box_set_selection_mode(GTK_LIST_BOX(config_window->conf_menu_stack_advanced_frame_SIFT_box), GTK_SELECTION_NONE);

        config_window->conf_menu_stack_advanced_frame_system_entry_nfeatures = gtk_entry_new ();
        ret = util::processValue(main_window->config_->nfeatures, 999);
        varAsString = std::to_string(ret.int_part);
        gtk_entry_set_text (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_nfeatures),varAsString.c_str());
        gtk_entry_set_input_purpose (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_nfeatures),GTK_INPUT_PURPOSE_NUMBER);
        gtk_entry_set_max_length(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_nfeatures),3);
        label = gtk_label_new("nfeatures");
        entry_settings(row,hbox,label,config_window->conf_menu_stack_advanced_frame_SIFT_box,config_window->conf_menu_stack_advanced_frame_system_entry_nfeatures);
        g_signal_connect(G_OBJECT(config_window->conf_menu_stack_advanced_frame_system_entry_nfeatures), "insert-text", G_CALLBACK(insert_text_event_int), NULL);

        config_window->conf_menu_stack_advanced_frame_system_entry_nOctaveLayers = gtk_entry_new ();
        ret = util::processValue(main_window->config_->nOctaveLayers, 999);
        varAsString = std::to_string(ret.int_part);
        gtk_entry_set_text (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_nOctaveLayers),varAsString.c_str());
        gtk_entry_set_input_purpose (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_nOctaveLayers),GTK_INPUT_PURPOSE_NUMBER);
        gtk_entry_set_max_length(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_nOctaveLayers),3);
        label = gtk_label_new("n Octave Layers");
        entry_settings(row,hbox,label,config_window->conf_menu_stack_advanced_frame_SIFT_box,config_window->conf_menu_stack_advanced_frame_system_entry_nOctaveLayers);
        g_signal_connect(G_OBJECT(config_window->conf_menu_stack_advanced_frame_system_entry_nOctaveLayers), "insert-text", G_CALLBACK(insert_text_event_int), NULL);

        config_window->conf_menu_stack_advanced_frame_system_entry_contrastThreshold = gtk_entry_new ();
        ret = util::processValue(main_window->config_->contrastThreshold, 10);
        varAsString = std::to_string(ret.double_part);
        gtk_entry_set_text (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_contrastThreshold),varAsString.c_str());
        gtk_entry_set_input_purpose (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_contrastThreshold),GTK_INPUT_PURPOSE_NUMBER);
        gtk_entry_set_max_length(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_contrastThreshold),8);
        label = gtk_label_new("Contrast threshold");
        entry_settings(row,hbox,label,config_window->conf_menu_stack_advanced_frame_SIFT_box,config_window->conf_menu_stack_advanced_frame_system_entry_contrastThreshold);
        g_signal_connect(G_OBJECT(config_window->conf_menu_stack_advanced_frame_system_entry_contrastThreshold), "insert-text", G_CALLBACK(insert_text_event_float), NULL);

        config_window->conf_menu_stack_advanced_frame_system_entry_edgeThreshold = gtk_entry_new ();
        ret = util::processValue(main_window->config_->edgeThreshold, 99);
        varAsString = std::to_string(ret.double_part);
        gtk_entry_set_text (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_edgeThreshold),varAsString.c_str());
        gtk_entry_set_input_purpose (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_edgeThreshold),GTK_INPUT_PURPOSE_NUMBER);
        gtk_entry_set_max_length(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_edgeThreshold),8);
        label = gtk_label_new("Edge threshold");
        entry_settings(row,hbox,label,config_window->conf_menu_stack_advanced_frame_SIFT_box,config_window->conf_menu_stack_advanced_frame_system_entry_edgeThreshold);
        g_signal_connect(G_OBJECT(config_window->conf_menu_stack_advanced_frame_system_entry_edgeThreshold), "insert-text", G_CALLBACK(insert_text_event_float), NULL);

        config_window->conf_menu_stack_advanced_frame_system_entry_sigma = gtk_entry_new ();
        ret = util::processValue(main_window->config_->sigma_sift, 99);
        varAsString = std::to_string(ret.double_part);
        gtk_entry_set_text (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_sigma),varAsString.c_str());
        gtk_entry_set_input_purpose (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_sigma),GTK_INPUT_PURPOSE_NUMBER);
        gtk_entry_set_max_length(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_sigma),8);
        label = gtk_label_new("Sigma");
        entry_settings(row,hbox,label,config_window->conf_menu_stack_advanced_frame_SIFT_box,config_window->conf_menu_stack_advanced_frame_system_entry_sigma);
        g_signal_connect(G_OBJECT(config_window->conf_menu_stack_advanced_frame_system_entry_sigma), "insert-text", G_CALLBACK(insert_text_event_float), NULL);

        gtk_container_add(GTK_CONTAINER(config_window->conf_menu_stack_advanced_frame_SIFT),config_window->conf_menu_stack_advanced_frame_SIFT_box);

        //Bundle Adjustment.............................
        config_window->conf_menu_stack_advanced_frame_adjustment = gtk_frame_new ("Bundle Adjustment");
        gtk_box_pack_start (GTK_BOX(config_window->conf_menu_stack_advanced_box),config_window->conf_menu_stack_advanced_frame_adjustment,FALSE,FALSE,5);
        config_window->conf_menu_stack_advanced_frame_adjustment_box = gtk_list_box_new ();

        config_window->conf_menu_stack_advanced_frame_system_entry_focal = gtk_entry_new ();
        ret = util::processValue(main_window->config_->focal, 99999);
        varAsString = std::to_string(ret.int_part);
        gtk_entry_set_text (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_focal),varAsString.c_str());
        gtk_entry_set_input_purpose (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_focal),GTK_INPUT_PURPOSE_NUMBER);
        gtk_entry_set_max_length(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_focal),5);
        label = gtk_label_new("Default focal");
        entry_settings(row,hbox,label,config_window->conf_menu_stack_advanced_frame_adjustment_box,config_window->conf_menu_stack_advanced_frame_system_entry_focal);
        g_signal_connect(G_OBJECT(config_window->conf_menu_stack_advanced_frame_system_entry_focal), "insert-text", G_CALLBACK(insert_text_event_float), NULL);

        config_window->conf_menu_stack_advanced_frame_system_entry_lambda = gtk_entry_new ();
        ret = util::processValue(main_window->config_->lambda, 999);
        varAsString = std::to_string(ret.double_part);
        gtk_entry_set_text (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_lambda),varAsString.c_str());
        gtk_entry_set_input_purpose (GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_lambda),GTK_INPUT_PURPOSE_NUMBER);
        gtk_entry_set_max_length(GTK_ENTRY(config_window->conf_menu_stack_advanced_frame_system_entry_lambda),8);
        label = gtk_label_new("Initial optimization step size");
        entry_settings(row,hbox,label,config_window->conf_menu_stack_advanced_frame_adjustment_box,config_window->conf_menu_stack_advanced_frame_system_entry_lambda);
        g_signal_connect(G_OBJECT(config_window->conf_menu_stack_advanced_frame_system_entry_lambda), "insert-text", G_CALLBACK(insert_text_event_float), NULL);

        gtk_container_add(GTK_CONTAINER(config_window->conf_menu_stack_advanced_frame_adjustment),config_window->conf_menu_stack_advanced_frame_adjustment_box);

    }


    void connect_signals(struct config_* config_window,struct main_window_ *main_window){

        g_signal_connect(config_window->conf_menu_CANCEL, "button-release-event", G_CALLBACK(cancel), config_window);
        g_signal_connect(config_window->conf_menu_OK, "button-release-event", G_CALLBACK(OK), config_window);

    }


    void open_conf_window(struct menu_bar_ *add_to,struct config_* config_window,struct main_window_ *main_window){

        config_window->conf_menu = gtk_window_new(GTK_WINDOW_TOPLEVEL);
        gtk_window_set_default_size(GTK_WINDOW(config_window->conf_menu), 800, 640);
        gtk_window_set_title(GTK_WINDOW(config_window->conf_menu), "Settings");

        config_window->conf_menu_paned = gtk_paned_new(GTK_ORIENTATION_HORIZONTAL);
        gtk_paned_set_wide_handle(GTK_PANED(config_window->conf_menu_paned), true);

        config_window->conf_menu_box = gtk_box_new(GTK_ORIENTATION_VERTICAL,1);
        gtk_box_pack_start (GTK_BOX(config_window->conf_menu_box),config_window->conf_menu_paned,TRUE,TRUE,1);
        config_window->conf_menu_cancelok_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL,1);
        gtk_box_pack_end (GTK_BOX(config_window->conf_menu_box),config_window->conf_menu_cancelok_box,FALSE,TRUE,1);

        config_window->conf_menu_OK = gtk_button_new_with_label ("Save");
        gtk_box_pack_start (GTK_BOX(config_window->conf_menu_cancelok_box),config_window->conf_menu_OK,FALSE,FALSE,1);
        config_window->conf_menu_CANCEL = gtk_button_new_with_label ("Cancel");
        gtk_box_pack_start (GTK_BOX(config_window->conf_menu_cancelok_box),config_window->conf_menu_CANCEL,FALSE,FALSE,1);

        gtk_container_add(GTK_CONTAINER(config_window->conf_menu), config_window->conf_menu_box);

        // sidebar
        config_window->conf_menu_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
        config_window->conf_menu_paned_sidebar = gtk_stack_switcher_new();
        gtk_orientable_set_orientation(GTK_ORIENTABLE(config_window->conf_menu_paned_sidebar), GTK_ORIENTATION_VERTICAL);
        gtk_box_pack_start(GTK_BOX(config_window->conf_menu_box), config_window->conf_menu_paned_sidebar, FALSE, FALSE, 0);
        gtk_paned_add1(GTK_PANED(config_window->conf_menu_paned), config_window->conf_menu_box);


        config_window->conf_menu_stack = gtk_stack_new();
        gtk_stack_set_transition_type(GTK_STACK(config_window->conf_menu_stack), GTK_STACK_TRANSITION_TYPE_SLIDE_LEFT_RIGHT);
        gtk_paned_add2(GTK_PANED(config_window->conf_menu_paned), config_window->conf_menu_stack);
        gtk_stack_switcher_set_stack(GTK_STACK_SWITCHER(config_window->conf_menu_paned_sidebar), GTK_STACK(config_window->conf_menu_stack));


        conf_blending_settings(config_window,main_window);
        conf_advanced_settings(config_window,main_window);
        connect_signals(config_window,main_window);

        gtk_widget_show_all(config_window->conf_menu);

    }

}

