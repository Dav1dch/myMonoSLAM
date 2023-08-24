#ifndef RENDER_H
#define RENDER_H

#include "tools.h"
#include "type.h"
#include <coroutine>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/handler/handler.h>

class viewer {
  public:
    std::string window_name;
    std::vector<renderFrame> renderQueue;
    MyCoroutine *co;

    void setup();
    void final();
    MyCoroutine run();
    viewer();
};

#endif /* ifndef RENDER */
