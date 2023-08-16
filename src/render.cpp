#include "render.h"

viewer::viewer() { this->window_name = "myMonoSLAM"; }

void viewer::setup() {
    // create a window and bind its context to the main thread
    pangolin::CreateWindowAndBind(this->window_name, 640, 480);

    // enable depth
    glEnable(GL_DEPTH_TEST);

    // unset the current context from the main thread
    pangolin::GetBoundWindow()->RemoveCurrent();
}

MyCoroutine viewer::run() {
    // fetch the context and bind it to this thread
    pangolin::BindToContext(this->window_name);

    // we manually need to restore the properties of the context
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 1000),
        // 对应的是gluLookAt,摄像机位置,参考点位置,up vector(上向量)
        pangolin::ModelViewLookAt(0, 0, -2, 0, 0, 0, pangolin::AxisNegY));

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
                                .SetHandler(&handler);

    renderFrame rf;
    while (!pangolin::ShouldQuit()) {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        co_await std::suspend_always{};
        std::cout << "co_await" << std::endl;
        rf = this->renderQueue.front();
        this->renderQueue.pop();
        std::cout << rf.f->t << std::endl;

        // 在原点绘制一个立方体
        // pangolin::glDrawColouredCube();
        // 绘制坐标系
        // glLineWidth(3);
        // glBegin(GL_LINES);
        // glColor3f(0.8f, 0.f, 0.f);
        // glVertex3f(-1, -1, -1);
        // glVertex3f(0, -1, -1);
        // glColor3f(0.f, 0.8f, 0.f);
        // glVertex3f(-1, -1, -1);
        // glVertex3f(-1, 0, -1);
        // glColor3f(0.2f, 0.2f, 1.f);
        // glVertex3f(-1, -1, -1);
        // glVertex3f(-1, -1, 0);
        // glEnd();
        // Render OpenGL Cube
        // pangolin::glDrawColouredCube();
        // pangolin::glDraw
        glPointSize(20.0);
        glBegin(GL_POINTS); // 点设置的开始
        glColor3f(0.0, 255.0, 0.0);
        glVertex3f(rf.f->t.at<float>(0), rf.f->t.at<float>(1),
                   rf.f->t.at<float>(2));
        // glPointSize(20.0);
        // glColor3f(255.0, 255.0, 0.0);
        // for (auto pt : rf.points) {
        //     std::cout << pt.pose << std::endl;
        //     glVertex3d(pt.pose.at<float>(0), pt.pose.at<float>(1),
        //                pt.pose.at<float>(2));
        // }
        glEnd(); // 点设置的结束

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    // unset the current context from the main thread
    pangolin::GetBoundWindow()->RemoveCurrent();
}
