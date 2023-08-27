#include "slam.h"

int main(int argc, char **argv) {

    viewer *v = new viewer();
    v->setup();
    auto co = v->run();
    v->co = &co;
    PointMap *map = new PointMap();
    process *p = new process(map);
    // VideoCapture cap(
    //     "/Users/david/Downloads/twitchslam/videos/test_countryroad.mp4");
    // VideoCapture cap("/Users/david/Codes/myMonoSLAM/test.mp4");
    VideoCapture cap(

        "/Users/david/Downloads/rgbd_dataset_freiburg1_xyz/rgb/%03d.png");
    // VideoCapture cap("/Users/david/Codes/slambook2/ch7/%01d.png");
    if (!cap.isOpened()) {
        cout << "Error opening video stream or file" << endl;
        return -1;
    }

    while (1) {

        Mat frame;
        // Capture frame-by-frame
        cap >> frame;

        // If the frame is empty, break immediately
        if (frame.empty())
            break;

        // Display the resulting frame

        p->process_frame_orb(frame, v);

        // Press  ESC on keyboard to exit
        char c = (char)waitKey(25);
        if (c == 27)
            break;
    }
    v->final();

    std::cout << p->posees.at(0).at(0) << std::endl;

    // Open a file for writing
    ofstream outfile("output.txt");
    std::vector<std::vector<double>> vec = p->posees;

    // Write the 2D vector to the file
    for (int i = 0; i < vec.size(); i++) {
        for (int j = 0; j < vec[i].size(); j++) {
            if (j != vec[i].size() - 1) {
                outfile << vec[i][j] << " ";
            } else {
                outfile << vec[i][j];
            }
        }
        outfile << endl;
    }

    // Close the file

    // When everything done, release the video capture object
    cap.release();

    // Closes all the frames
    destroyAllWindows();

    return 0;
}
