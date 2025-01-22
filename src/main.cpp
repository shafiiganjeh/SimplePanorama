
#include <gtk/gtk.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <string>
#include "_gtk_vars.h"
#include "_main_windows.h"

struct main_window_ main_window;

int main(int argc, char **argv) {


        build_window(argc,argv,&main_window);


        gtk_main();

}
