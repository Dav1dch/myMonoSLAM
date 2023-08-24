#include "render.h"
#include <pangolin/gl/gldraw.h>

viewer::viewer() { this->window_name = "myMonoSLAM"; }

void viewer::setup() {
    // create a window and bind its context to the main thread
    pangolin::CreateWindowAndBind(this->window_name, 1920, 1080);

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
        pangolin::ProjectionMatrix(1920, 1080, 960, 540, 960, 540, 0.02, 1000),
        // 对应的是gluLookAt,摄像机位置,参考点位置,up vector(上向量)
        // pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY));
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View &d_cam =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, 1920.0f / 1080.0f * 1.3f)
            .SetHandler(&handler);

    // renderFrame rf;
    while (!pangolin::ShouldQuit()) {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        co_await std::suspend_always{};
        // std::cout << this->renderQueue.back().f->t << std::endl;
        glPointSize(3.0);
        glBegin(GL_POINTS);

        for (auto rf : this->renderQueue) {
            glColor3f(0.0, 255.0, 0.0);
            glVertex3f(rf.f->t.at<float>(0), rf.f->t.at<float>(1),
                       rf.f->t.at<float>(2));
            glColor3f(255.0, 0.0, 0.0);
            for (auto pt : rf.points) {
                // std::cout << pt.pose << std::endl;
                // std::cout << pt.pose.at<float>(0) << " ," <<
                // pt.pose.at<float>(1)
                //           << " ," << pt.pose.at<float>(2) << std::endl;
                glVertex3f(pt.pose.at<float>(0), pt.pose.at<float>(1),
                           pt.pose.at<float>(2));
            }
        }
        glEnd();

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    // unset the current context from the main thread
    pangolin::GetBoundWindow()->RemoveCurrent();
}

void viewer::final() {

    // fetch the context and bind it to this thread
    pangolin::BindToContext(this->window_name);

    // we manually need to restore the properties of the context
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640, 480, 420, 320, 240, 240, 0.02, 1000),
        // 对应的是gluLookAt,摄像机位置,参考点位置,up vector(上向量)
        // pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY));
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View &d_cam =
        pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, 1920.0f / 1080.0f)
            .SetHandler(&handler);

    // renderFrame rf;
    while (!pangolin::ShouldQuit()) {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        // std::cout << this->renderQueue.back().f->t << std::endl;
        glPointSize(3.0);
        glBegin(GL_POINTS);

        for (auto rf : this->renderQueue) {
            glColor3f(0.0, 255.0, 0.0);
            glVertex3f(rf.f->t.at<float>(0), rf.f->t.at<float>(1),
                       rf.f->t.at<float>(2));
            glColor3f(255.0, 0.0, 0.0);
            for (auto pt : rf.points) {
                // std::cout << pt.pose << std::endl;
                // std::cout << pt.pose.at<float>(0) << " ," <<
                // pt.pose.at<float>(1)
                //           << " ," << pt.pose.at<float>(2) << std::endl;
                glVertex3f(pt.pose.at<float>(0), pt.pose.at<float>(1),
                           pt.pose.at<float>(2));
            }
        }
        glEnd();

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    // unset the current context from the main thread
    pangolin::GetBoundWindow()->RemoveCurrent();
}
