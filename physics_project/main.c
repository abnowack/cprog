#include "application.h"

int main(int argc, char *argv[])
{
    app_setup(800, 600);

    while(app.running)
    {
        app_input();
        app_update();
        app_render();
    }

    app_destroy();

    return 0;
}