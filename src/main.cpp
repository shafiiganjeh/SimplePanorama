
#include <gtk/gtk.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <string>
#include "_build_ui.h"



int main(int argc, char **argv) {

        image_paths ipts;
        ui::build_window(argc,argv);
        ui::connect_signals(&ipts);

        gtk_main();

}
