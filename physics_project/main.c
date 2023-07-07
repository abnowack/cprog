#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include "application.h"

void main_loop()
{
    if (app.running)
    {
        app_input();
        app_update();
        app_render();
    }
    else
    {
#ifdef __EMSCRIPTEN__
        emscripten_cancel_main_loop();
#endif
    }
}

int main(int argc, char *argv[])
{
    app_setup(1980, 1200);

#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop(main_loop, 0, 1);
#endif
#ifndef __EMSCRIPTEN__
    while (app.running)
    {
        app_input();
        app_update();
        app_render();
    }
#endif

    app_destroy();

    return 0;
}