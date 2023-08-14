#include "slam.h"

// fe_extract::feature_extractor extractor;

// void process_frame(Mat frame) {
// Mat frame_;
// // cv::resize(frame, frame_, Size(540, 480));
// vector<Point2f> corners;
// Mat descriptors;
// extractor.extract(frame, corners, descriptors);
// // cout << corners.size() << endl;
// }

int main(int argc, char **argv) {

    frame::frame *f = new frame::frame();
    // VideoCapture cap("/Users/david/Codes/myMonoSLAM/test.mp4");
    VideoCapture cap("/Users/david/Downloads/sfm_lab_room_1/rgb/img_%05d.png");
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

        f->process_frame(frame);

        // Press  ESC on keyboard to exit
        char c = (char)waitKey(25);
        if (c == 27)
            break;
    }

    std::cout << f->posees.at(0).at(0) << std::endl;

    // Open a file for writing
    ofstream outfile("output.txt");
    std::vector<std::vector<float>> vec = f->posees;

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
