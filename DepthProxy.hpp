#ifndef __DEPTHPROXY_HPP__
#define __DEPTHPROXY_HPP__

#include <depthai/depthai.hpp>

namespace ds {
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
}

#endif
