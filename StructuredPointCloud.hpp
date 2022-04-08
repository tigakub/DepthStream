#ifndef __STRUCTUREDPOINTCLOUD_HPP__
#define __STRUCTUREDPOINTCLOUD_HPP__

#include <depthai/depthai.hpp>
#include "LinearAlgebra.hpp"

namespace ds {
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
            cx_over_fx(iCameraIntrinsicCx * rfx), cy_over_fy(iCameraIntrinsicCy * rfy) {
                // cout << "rfx: " << rfx << ", rfy: " << rfy << ", cx_over_fx: " << cx_over_fx << ", cy_over_fy: " << cy_over_fy << endl;
            }

            // Function to convert depth data into cartesian vertices and store them
            void convert(const cv::Mat &iRawDepth) {
                for(int y = 0; y < HEIGHT; y++) {
                    for(int x = 0; x < WIDTH; x++) {
                        int i = y * WIDTH + x;
                        float u = float(x), v = float(y);
                        float z = float(*iRawDepth.ptr<uint16_t>(y, x));
                        data[i] = vertex3(
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
                        data[i] = vertex3(
                            z * (u * rfx - cx_over_fx),
                            z * (v * rfy - cy_over_fy),
                            z);
                    }
                }
            }

            // Return a const pointer to the point cloud vertices
            const vertex3 *cartesianMap() const { return data; }

            const vertex3 &operator()(int x, int y) const {
                return data[y * WIDTH + x];
            }

        protected:
            float rfx, rfy, cx_over_fx, cy_over_fy;
            vertex3 data[WIDTH * HEIGHT];
    };
}

#endif
