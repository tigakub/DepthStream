//
//  main.cpp
//  DepthStream
//
//  Created by Edward Janne on 3/18/22.
//

#include <iostream>
#include <iomanip>
#include <chrono> // For steady_clock and time_point
#include <math.h> // For fabs()

#include <termios.h>
#include <signal.h>

#include "Exception.hpp"
#include "Server.hpp"
#include "Connection.hpp"
#include "DepthProxy.hpp"
#include "StructuredPointCloud.hpp"

#include <depthai/depthai.hpp>

using namespace std;
using namespace std::chrono;
using namespace dai;
using namespace dai::node;
using namespace ds;

static struct termios stdTerm, rawTerm;

void setTermRaw(bool echo = true) {
    tcgetattr(0, &stdTerm);
    rawTerm = stdTerm;
    rawTerm.c_lflag &= ~(ICANON | ECHO);
    if(echo) rawTerm.c_lflag |= ECHO;
    tcsetattr(0, TCSANOW, &rawTerm);
}

void resetTerm() {
    tcsetattr(0, TCSANOW, &stdTerm);
}

atomic_bool quit(false);
struct sigaction oldsa, newsa;
void sigIntHandler(int iSig) {
    quit = true;
}

class ServerBufferFunctor : public BufferFunctor
{
    virtual void operator()(Connection &cnx, ds::Buffer &buf) {
        cout << ".";
    }
};

class ClientBufferFunctor : public BufferFunctor
{
    virtual void operator()(Connection &cnx, ds::Buffer &buf) {
        cout << ".";
    }
};

ServerBufferFunctor serverFunctor;

Server Server::shared(serverFunctor);

void printUsage(const string &iAppName) {
    cout
        << "Usage: " << iAppName << " <option(s)>" << endl
        << "Options: -h, --help\t\t\tdisplay this message" << endl
        << "         -d, --depth\t\t\tdisplay depth telemetry" << endl
        << "         -v, --verbose\t\t\tprint capture info to contsole" << endl
        << "         -s, --server\t\t\tstart server" << endl
        << "         -c, --connect <ip> <port>\tconnect to server" << endl;
}

int main(int argc, const char * argv[]) {
    ClientBufferFunctor clientFunctor;
    Connection client(clientFunctor);

    bool visualizeDepth = false;
    bool verbose = false;
    bool startServer = false;
    bool startClient = false;
    string interface("en0");
    string host;
    int port;

    for(int a = 1; a < argc; a++) {
        string arg(argv[a]);
        if((arg == "-h") || (arg == "--help")) {
            printUsage(argv[0]);
            return 0;
        }
        
        if((arg == "-d") || (arg == "--depth")) {
            visualizeDepth = true;
        } else if((arg == "-v") || (arg == "--verbose")) {
            verbose = true;
        } else if((arg == "-s") || (arg == "--server")) {
            if(startClient) {
                cerr << "The -s and -c options are mutually exclusive" << endl;
                return 0;
            }
            startServer = true;
            interface = argv[a+1];
            a += 1;
        } else if((arg == "-c") || (arg == "--connect")) {
            if(startServer) {
                cerr << "The -s and -c options are mutually exclusive" << endl;
                return 0;
            }
            if((a+2) >= argc) {
                printUsage(argv[0]);
                return 0;
            } else {
                startClient = true;
                host = argv[a+1];
                port = stoi(argv[a+2]);
                a += 2;
            }
        }
    }

    // Intercept SIGINT
    newsa.sa_handler = sigIntHandler;
    sigemptyset(&newsa.sa_mask);
    newsa.sa_flags = 0;
    sigaction(SIGINT, NULL, &oldsa);
    sigaction(SIGINT, &newsa, NULL);
    
    cout << "<DepthStream>" << endl;

    if(startServer) {
        cout << "  Setting up streaming server" << endl;
        try {
            // Attempt to start listening
            Server::shared.start(interface);

            // Attempt to retrieve the local hostname and ip address
            char hostBuffer[256];
            char *ipBuffer;
            struct hostent *hostEnt;
            int hostname;
            hostname = gethostname(hostBuffer, sizeof(hostBuffer));
            if(hostname != -1) {
                cout << "    " << hostBuffer << ".local:3060 (" << Server::shared.ip() << ":3060)" << endl;
            } else {
                cerr << "    Unable to retrieve hostname" << endl;
            }
        } catch(Exception &e) {
            startServer = false;
            cerr << "  " << e << endl;
        } catch(...) {
            startServer = false;
            cerr << "    Unknown exception while attempting to start streaming server" << endl;
        }
    }

    if(startClient) {
        cout << "  Starting connection to " << host << ":" << port << endl;
        try {
            client.connect(host, port);
        } catch(Exception &e) {
            cout << e << endl;
            startClient = false;
        }
    }

    cout << "  Setting up pipeline" << endl;

    Pipeline pl;

    //----------------
    // Create IMU Node
    //----------------

    cout << "    Creating IMU node" << endl;

    auto imu = pl.create<IMU>();

    // Enable fused orientation at 60 Hz
    imu->enableIMUSensor(IMUSensor::ROTATION_VECTOR, 60);
    // Don't really know what this means. How can you ever receive
    // more than 1 packet at a time?!? If I specify anything higher
    // than 1, does that mean the camera will delay sending them
    // until the specified number of packets has been accumulated?
    // Not really sure when you would EVER want that.
    imu->setBatchReportThreshold(1);
    // Stop sending if the queue gets backed up until host can pull some off
    imu->setMaxBatchReports(5);

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

    cout << "    Linking IMU to output node" << endl;

    // Feed IMU telemetry to an output stream
    auto imuOutput = pl.create<XLinkOut>();
    imuOutput->setStreamName("rotation_frames");

    imu->out.link(imuOutput->input);

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
    // the queues associated with the streams named "depth_frames"
    // and "rotation_frames"
    Device dev(pl);
    auto depthFrameQueue = dev.getOutputQueue("depth_frames", 8, false);
    auto rotationFrameQueue = dev.getOutputQueue("rotation_frames", 8, false);

    cout << "    Retrieving camera intrinsics" << endl;

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

    cout << "      fx: " << fx << ", fy: " << fy << ", cx: " << cx << ", cy: " << cy << endl;

    // Instantiate storage for the point cloud
    StructuredPointCloud<640, 400> pc(fx, fy, cx, cy);

    cout << "  Commencing capture (press q to terminate)" << endl;

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

    time_point<steady_clock> frameTimeStamp = timeStamp;

    // Image buffer to store color conversion from greyscale depth
    cv::Mat cvRGBImg;

    int key = 0;

    // Set console to unbuffered mode
    setTermRaw(false);

    do {
        // Get the latest rotation frame
        auto rotationFrame = rotationFrameQueue->get<IMUData>();
        auto rotationPackets = rotationFrame->packets;

        float qi, qj, qk, qr, qaccuracy;
        if(rotationPackets.size()) {
            auto &packet = rotationPackets.back();
            auto rotation = packet.rotationVector;
            qi = rotation.i;
            qj = rotation.j;
            qk = rotation.k;
            qr = rotation.real;
            qaccuracy = rotation.rotationVectorAccuracy;
        }

        // Get the latest depth frame
        auto depthImgFrame = depthFrameQueue->get<ImgFrame>();

        // Get the depth frame as a cv2::Mat without any buffer copying
        auto cvDepthImg = depthImgFrame->getFrame(false);

        DepthProxy<uint16_t> proxy(cvDepthImg);
        // If a zero depth has non-zero neighbors within 20 pixels, and not more than 100 mm difference
        // then interpolate linearly between the neighboring values. Otherwise, replace the zero with
        // the max depth.
        proxy.interpolateZero(10, 100, 33119, 10000);
        // proxy.replaceZero(33119);

        // Get a direct pointer to the raw depth buffer
        uint16_t *rawDepth = (uint16_t *) cvDepthImg.ptr<uint16_t>();

        time_point<steady_clock> currentTime = steady_clock::now();

        if(startClient) {
            if((float(duration_cast<microseconds>(currentTime - frameTimeStamp).count()) * 0.000001) > 0.05) {
                ds::Buffer *buf = client.recoverSendBuffer();
                if(!buf) {
                    buf = new ds::Buffer(new Header());
                }

                uint16_t *bufPtr = (uint16_t *) buf->getPayload();
                Header &header = buf->getHeader();
                int i = 640*400;
                while(i--) bufPtr[i] = rawDepth[i];
                header.setPayloadSize(1024000);
                header.setRotation(qi, qj, qk, qr);
                client.submitSendBuffer(buf);

                frameTimeStamp = currentTime;
            }
        }

        // Pass the raw pointer to the StructuredPointCloud object for mapping into 3-space
        pc.convert(rawDepth);

        // Calculate moving average of fps
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

        if(verbose) {
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
                    << "    fps: " << fixed << setw(4) << setprecision(1) << right << showpoint << fps
                    << " center: ("
                        << setw(8) << setprecision(2) << center.x * 0.1
                    << ", "
                        << setw(8) << setprecision(2) << center.y * 0.1
                    << ", " 
                        << setw(8) << setprecision(2) << center.z * 0.1
                    << ") cm"
                    << " rotation: q("
                        << setw(8) << setprecision(2) << qi
                    << ", "
                        << setw(8) << setprecision(2) << qj
                    << ", "
                        << setw(8) << setprecision(2) << qk
                    << ", "
                        << setw(8) << setprecision(2) << qr
                    << ") accuracy: "
                        << setw(8) << setprecision(2) << qaccuracy
                        << "\r" << flush;
                reportTimeStamp = currentTime;
            }
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
            if((key == 'q') || (key == 'Q')) {
                quit = true;
            }
        }

        // Is data available on stdin?
        struct pollfd fds;
        int ret = 0;
        fds.fd = STDIN_FILENO;
        fds.events = POLLIN;
        ret = poll(&fds, 1, 0);
        if(ret) {
            key = getc(stdin);
            switch(key) {
                case 'q':
                    quit = true;
                    break;
            }
        }

    // Loop endlessly
    } while(!quit);

    if(verbose) {
        cout << endl;
    }

    if(startServer) {
        cout << "  Stopping streaming server" << endl;
        Server::shared.stop();
    }

    if(startClient) {
        client.stop();
    }

    cout << "</DepthStream>" << endl;

    // Restore original SIGINT handler
    sigaction(SIGINT, &oldsa, NULL);

    // Restore terminal to original mode
    resetTerm();

    return 0;
}
