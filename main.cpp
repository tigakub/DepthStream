//
//  main.cpp
//  DepthStream
//
//  Created by Edward Janne on 3/18/22.
//

#include <iostream>
#include <chrono>
#include <depthai/depthai.hpp>
#include <math.h>
#include <thread>

using namespace std;
using namespace std::chrono;
using namespace dai;
using namespace dai::node;

// Structure to encapsulate a 3D vertex
typedef struct vertex
{
    vertex(float ix = 0.0, float iy = 0.0, float iz = 0.0): x(ix), y(ix), z(iz) { }
    float x, y, z;
} vertex;

// Template class to encapsulate a point cloud that knows how to map raw depth data
// into cartesian 3-space
template <int WIDTH = 640, int HEIGHT = 400>
class StructuredPointCloud
{
    public:
        // Constructor which takes camera intrinsics
        StructuredPointCloud(float iCameraIntrinsicFx, float iCameraIntrinsicFy, float iCameraIntrinsicCx, float iCameraIntrinsicCy)
        // Pre-compute inverse transformation factors from camera intrinsics
        : rfx(1.0 / iCameraIntrinsicFx), rfy(1.0 / iCameraIntrinsicFy),
          cx_over_fx(iCameraIntrinsicCx * rfx), cy_over_fy(iCameraIntrinsicCy * rfy)
        {
            cout << "rfx: " << rfx << ", rfy: " << rfy << ", cx_over_fx: " << cx_over_fx << ", cy_over_fy: " << cy_over_fy << endl;
        }

        // Function to convert depth data into cartesian vertices and store them
        void convert(const cv::Mat &iRawDepth) {
            for(int y = 0; y < HEIGHT; y++) {
                for(int x = 0; x < WIDTH; x++) {
                    int i = y * WIDTH + x;
                    float u = float(x), v = float(y);
                    float z = float(*iRawDepth.ptr<uint16_t>(y, x));
                    data[i] = vertex(
                        z * (u * rfx - cx_over_fx),
                        z * (v * rfy - cy_over_fy),
                        z);
                }
            }
        }

        // Function to convert depth data into cartesian vertices and store them
        void convert(const uint16_t *iRawDepth) {
            for(int y = 0; y < HEIGHT; y++) {
                for(int x = 0; x < WIDTH; x++) {
                    int i = y * WIDTH + x;
                    float u = float(x), v = float(y);
                    float z = float(iRawDepth[i]);
                    data[i] = vertex(
                        z * (u * rfx - cx_over_fx),
                        z * (v * rfy - cy_over_fy),
                        z);
                }
            }
        }

        // Return a const pointer to the point cloud vertices
        const vertex *cartesianMap() const { return data; }

        const vertex &operator()(int x, int y) const {
            return data[y * WIDTH + x];
        }

    protected:
        float rfx, rfy, cx_over_fx, cy_over_fy;
        vertex data[WIDTH * HEIGHT];
};

/*
template <int WIDTH=640, int HEIGHT=400, class T>
T replaceZero(cv::Mat &ioImg, T iReplacement) {
    T max = 0;
    T *ptr = ioImg.ptr<T>();
    int i = WIDTH * HEIGHT;
    while(i--) {
        if(ptr[i] > max) max = ptr[i];
        if(!ptr[i]) ptr[i] = iReplacement;
    }
    return max;
}
*/

template <class T>
class DepthProxy {
    public:
        DepthProxy(cv::Mat &iImage) : img(iImage)
        { }

        int width() const { return img.cols; }
        int height() const { return img.rows; }

        T value(int x, int y) const { return img.ptr<T>()[y * width() + x]; }
        T &value(int x, int y) { return img.ptr<T>()[y * width() + x]; }

        T replaceZero(T iReplacement) {
            T max = 0;
            T *ptr = img.ptr<T>();
            int i = width() * height();
            while(i--) {
                if(ptr[i] > max) max = ptr[i];
                if(!ptr[i]) ptr[i] = iReplacement;
            }
        }

        static void interpolateZeroThreadFunc(DepthProxy<T> *iSelf, int iMaxSearch, T iMaxDif, T iDefaultReplacement, T iCutoff, int iStartY, int iRows) {
            iSelf->interpolateZeroBlock(iMaxSearch, iMaxDif, iDefaultReplacement, iCutoff, iStartY, iRows);
        }

        void interpolateZero(int iMaxSearch, T iMaxDif, T iDefaultReplacement, T iCutoff, bool iMultithreaded = true) {
            if(iMultithreaded) {
                vector<thread *> threads;
                int h = iMaxSearch << 1;
                for(int y = 0; y < height(); y += iMaxSearch) {
                    if((y + h) > height()) h = height() - y;
                    threads.push_back(
                        new thread(DepthProxy<T>::interpolateZeroThreadFunc, this, iMaxSearch, iMaxDif, iDefaultReplacement, iCutoff, y, h)
                    );
                }
                for(auto *aThread : threads) {
                    aThread->join();
                    delete aThread;
                }
            } else {
                interpolateZeroBlock(iMaxSearch, iMaxDif, iDefaultReplacement, iCutoff, 0, height());
            }
        }

        void interpolateZeroBlock(int iMaxSearch, T iMaxDif, T iDefaultReplacement, T iCutoff, int iStartY, int iRows) {
            for(int y = iStartY; y < iStartY + iRows; y++) {
                for(int x = 0; x < width(); x++) {
                    if(!value(x, y)) {
                        int usex = 0, minx, maxx; T xval1, xval2;
                        int usey = 0, miny, maxy; T yval1, yval2;
                        for(int i = 1; (i < iMaxSearch) && (usex < 3) && (usey < 3); i++) {
                            int lowerx(x - i), upperx(x + i), lowery(y - i), uppery(y + i);
                            if((lowerx >= 0) && (xval1 = value(lowerx, y))) {
                                usex |= 1;
                                minx = lowerx;
                            }
                            if((upperx < width()) && (xval2 = value(upperx, y))) {
                                usex |= 2;
                                maxx = upperx;
                            }
                            if((lowery >= 0) && (yval1 = value(x, lowery))) {
                                usey |= 1;
                                miny = lowery;
                            }
                            if((uppery < height()) && (yval2 = value(x, uppery))) {
                                usey |= 2;
                                maxy = uppery;
                            }
                        }
                        float xrange(xval2 - xval1), yrange(yval2 - yval1);
                        if((usex == 3) && (fabs(xrange) < iMaxDif)) {
                            float xstep = float(xrange) / float(maxx - minx);
                            float newx = xval1 + xstep;
                            for(int ix = minx + 1; ix < maxx; ix++, newx += xstep) {
                                value(ix, y) = T(newx);
                            }
                        } else if((usey == 3) && (fabs(yrange) < iMaxDif)) {
                            float ystep = float(yrange) / float(maxy - miny);
                            float newy = yval1 + ystep;
                            for(int iy = miny + 1; iy < maxy; iy++, newy += ystep) {
                                value(x, iy) = T(newy);
                            }
                        } else {
                            value(x, y) = iDefaultReplacement;
                        }
                    }
                    if(value(x, y) > iCutoff) {
                        value(x, y) = iDefaultReplacement;
                    }
                }
            }
        }

    protected:
        cv::Mat &img;
};

int main(int argc, const char * argv[]) {

    bool visualizeDepth = false;
    for(int a = 1; a < argc; a++) {
        string arg(argv[a]);
        if((arg == "-h") || (arg == "--help")) {
            cout
                << "Usage: " << argv[0] << " <option(s)>" << endl
                << "Options: -h, --help\t\tdisplay this message" << endl
                << "         -d, --visualize\tdisplay depth telemetry" << endl;
            return 0;
        }
        if((arg == "-d") || (arg == "--depth")) {
          visualizeDepth = true;
        }
    }

    cout << "<DepthStream>" << endl;

    cout << "  Setting up pipeline" << endl;

    Pipeline pl;
    //------------------------
    // Create MonoCamera Nodes
    //------------------------

    cout << "    Creating left and right mono camera nodes" << endl;

    // Create a MonoCamra node
    auto leftMono = pl.create<MonoCamera>();
    // Specify a resolution
    leftMono->setResolution(MonoCameraProperties::SensorResolution::THE_400_P);
    // Associate this node with the left camera hardware socket
    leftMono->setBoardSocket(CameraBoardSocket::LEFT);
    leftMono->setFps(30);

    // Create a MonoCamra node
    auto rightMono = pl.create<MonoCamera>();
    // Specify a resolution
    rightMono->setResolution(MonoCameraProperties::SensorResolution::THE_400_P);
    // Associate this node with the right camera hardware socket
    rightMono->setBoardSocket(CameraBoardSocket::RIGHT);
    rightMono->setFps(30);

    //--------------------------
    // Create a StereoDepth Node
    //--------------------------

    cout << "    Creating stereo depth node" << endl;

    auto stereo = pl.create<StereoDepth>();

    // Use a default profile preset
    // Favor accuracy over density (the other option is HIGH_DENSITY)
    stereo->setDefaultProfilePreset(StereoDepth::PresetMode::HIGH_ACCURACY);

    //-------------------------------
    // Stereo Matching Block Settings
    //-------------------------------

    // Remove disparity pixels that are incorrectly calculated due to
    // occlusions at object borders
    stereo->setLeftRightCheck(true);

    // Increase max disparity search from 96 to 191 (decreased range, but
    // better detection of near-by objects)
    stereo->setExtendedDisparity(true);

    // Extended disparity and subpixel modes are mutually exclusive at
    // the moment. Subpixel mode is used to inprove accuracy at long
    // range, and to provide better surface normal estimation, but I am
    // turning it off in favor of better recognition of near-by objects.
    stereo->setSubpixel(false);

    //-------------------------------
    // Post-processing Block Settings
    //-------------------------------

    // Median filters are very efficient hardware based filters to
    // smooth the depth map. They are non-edge preserving.
    // The other options are MEDIAN_OFF, KERNEL_3x3, and KERNEL_7x7
    // (please note the lower-case 'x').
    stereo->initialConfig.setMedianFilter(MedianFilter::KERNEL_5x5);

    stereo->initialConfig.setConfidenceThreshold(191);

    //--------------------------------
    // Post-processing noise reduction
    //--------------------------------

    // Get the current raw configuration
    RawStereoDepthConfig rawConfig = stereo->initialConfig.get();

    // Interpolate missing depth values
    rawConfig.postProcessing.spatialFilter.enable = true;

    // Reduce unevenness
    rawConfig.postProcessing.speckleFilter.enable = true;

    // Use depth/disparity in previous temporal frames to fill
    // in for missing pixels in the current frame
    rawConfig.postProcessing.temporalFilter.enable = true;

    // Write the raw configuratino to the the initialConfig object
    stereo->initialConfig.set(rawConfig);

    /*
    //-------------------------
    // Create VideoEncoder Node
    //-------------------------

    auto videoEncoder = pl.create<VideoEncoder>();
    videoEncoder->setQuality(100);
    */

    //-------------
    // Node Linking
    //-------------

    cout << "    Linking mono camera nodes to stereo depth node" << endl;

    // Feed the left and right monochrome camera outputs into
    // the StereoDepth node's left and right inputs.
    leftMono->out.link(stereo->left);
    rightMono->out.link(stereo->right);

    cout << "    Creating output node, and linking stereo depth output to it" << endl;

    // Create an output stream, named "depth_frames", and link
    // the StereoDepthNode output to its input
    auto depthOutput = pl.create<XLinkOut>();
    depthOutput->setStreamName("depth_frames");
    stereo->depth.link(depthOutput->input);

    //-------------------------
    // Retrieve Depth Telemetry
    //-------------------------

    cout << "    Retrieving reference to depth frame output queue" << endl;

    // Instantiate a device for the pipeline and get a reference to
    // the queue associated with the stream named "depth"
    Device dev(pl);
    auto depthFrameQueue = dev.getOutputQueue("depth_frames", 8, false);

    cout << "  Retrieving camera intrinsics" << endl;

    // Get the intrinsics for the right mono camera
    CalibrationHandler calibration = dev.readCalibration();

    // Not really sure why I have to do this. Without passing topLeft and bottomRight
    // set to these values, getCameraIntrinsics() returns approximately 640 and 400
    // as the camera principle point. I assume this is  because the max frame size is
    // 1280x800. But the getCameraIntrinsics() function calculates cx and cy by
    // subtracting topLeft.x from the cx and bottomRight.y from the cy returned from
    // the calibrated intrinsics stored in eeprom.
    Point2f topLeft, bottomRight;
    topLeft.x = 320; topLeft.y = 200;
    bottomRight.x = 320; bottomRight.y = 200;
    auto monoIntrinsics = calibration.getCameraIntrinsics(CameraBoardSocket::RIGHT, -1, -1, topLeft, bottomRight);

    // The intrinsics matrix is stored in row major order.
    float fx = monoIntrinsics[0][0];
    float fy = monoIntrinsics[1][1];
    float cx = monoIntrinsics[0][2];
    float cy = monoIntrinsics[1][2];

    cout << "    fx: " << fx << ", fy: " << fy << ", cx: " << cx << ", cy: " << cy << endl;

    // Instantiate storage for the point cloud
    StructuredPointCloud<640, 400> pc(fx, fy, cx, cy);

    cout << "  Commencing capture (press Ctrl-C to terminate)" << endl;

    // Cyclic buffer to calculate simple moving average of fps
    const int bufSize = 20;
    float cyclicBuffer[bufSize];
    int tail = 0;
    int count = 0;
    float sum;
    float fps = 0.0;

    // Initialize a timestamp to time the loop
    time_point<steady_clock> timeStamp = steady_clock::now();

    // And a timestamp for throttling the output
    time_point<steady_clock> reportTimeStamp = timeStamp;

    // Image buffer to store color conversion from greyscale depth
    cv::Mat cvRGBImg;

    int key = 0;
    bool quit = false;

    do {

        // Get the latest depth frame
        auto depthImgFrame = depthFrameQueue->get<ImgFrame>();

        // Get the depth frame as a cv2::Mat without any buffer copying
        auto cvDepthImg = depthImgFrame->getFrame(false);

        DepthProxy<uint16_t> proxy(cvDepthImg);
        // If a zero depth has non-zero neighbors within 20 pixels, and not more than 100 mm difference
        // then interpolate linearly between the neighboring values. Otherwise, replace the zero with
        // the max depth.
        proxy.interpolateZero(10, 100, 33119, 3000);
        // proxy.replaceZero(33119);

        // Get a direct pointer to the raw depth buffer
        uint16_t *rawDepth = (uint16_t *) cvDepthImg.ptr<uint16_t>();

        // Pass the raw pointer to the StructuredPointCloud object for mapping into 3-space
        pc.convert(rawDepth);

        // Calculate moving average of fps
        time_point<steady_clock> currentTime = steady_clock::now();
        auto elapsed = currentTime - timeStamp;
        float instantaneousFPS = 1000000.0 / float (duration_cast<microseconds>(elapsed).count());
        cyclicBuffer[tail] = instantaneousFPS;
        sum += instantaneousFPS;
        tail++;
        tail %= bufSize;
        (count < bufSize) && (count++);
        fps = sum / float(count);
        (count == bufSize) && (sum -= cyclicBuffer[tail]);
        timeStamp = currentTime;

        // Throttle reporting
        elapsed = currentTime - reportTimeStamp;
        float secondsElapsed = float(duration_cast<microseconds>(elapsed).count()) * 0.000001;
        if(secondsElapsed > 1.0) {
            const auto &upper_left = pc(50, 20);
            const auto &upper_right = pc(540, 20);
            const auto &center = pc(320, 200);
            const auto &lower_left = pc(50, 380);
            const auto &lower_right = pc(540, 380);
            cout
                << "    fps: " << fps << endl
                << "    upper_left  (" << upper_left.x * 0.1 << ", " << upper_left.y * 0.1 << ", " << upper_left.z * 0.1 << ") cm" << endl
                << "    upper_right (" << upper_right.x * 0.1 << ", " << upper_right.y * 0.1 << ", " << upper_right.z * 0.1 << ") cm" << endl
                << "    center      (" << center.x * 0.1 << ", " << center.y * 0.1 << ", " << center.z * 0.1 << ") cm" << endl
                << "    lower_left  (" << lower_left.x * 0.1 << ", " << lower_left.y * 0.1 << ", " << lower_left.z * 0.1 << ") cm" << endl
                << "    lower_right (" << lower_right.x * 0.1 << ", " << lower_right.y * 0.1 << ", " << lower_right.z * 0.1 << ") cm" << endl;
            reportTimeStamp = currentTime;
        }

        if(visualizeDepth) {
            // Convert depth image to a format that applyColorMap() will accept, scaled to a visible range
            cvDepthImg.convertTo(cvRGBImg, CV_8U, 0.03);

            // Map greyscale to a rainbow ramp with shallow depth values mapped to warm colors,
            // and deep values mapped to cool colors
            cv::applyColorMap(cvRGBImg, cvRGBImg, cv::COLORMAP_RAINBOW);

            // Call OpenCV to display the color image
            cv::imshow("depth", cvRGBImg);
            // Allow OpenCV to process window and key events
            key = cv::waitKey(1);
            quit = (key == 'q') || (key == 'Q');
        }

    // Loop endlessly
  } while(!quit);

    return 0;
}
