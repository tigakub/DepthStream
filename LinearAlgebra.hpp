//
//  LinearAlgebra.hpp
//  SocketServer
//
//  Created by Edward Janne on 4/7/22.
//

#ifndef LinearAlgebra_hpp
#define LinearAlgebra_hpp

#include <stdio.h>

namespace ds {
    // Structure to encapsulate a 3D vertex
    typedef struct vertex3 {
        vertex3(float ix = 0.0, float iy = 0.0, float iz = 0.0): x(ix), y(ix), z(iz) { }
        float x, y, z;
    } vertex3;
    
    typedef struct matrix33 {
        matrix33() { auto i(9); while(i--) a[i] = (i%4) ? 0.0: 1.0; }
        matrix33(float ia[9]) { auto i(9); while(i--) a[i] = ia[i]; }
        float a[9];
    } matrix33;
    
    typedef struct vertex4 {
        vertex4(float ix = 0.0, float iy = 0.0, float iz = 0.0, float iw = 1.0)
        : x(ix), y(iy), z(iz), w(iz)
        { }
        float x, y, z, w;
    } vertex4;
    
    typedef struct matrix44 {
        matrix44() { auto i(16); while(i--) a[i] = (i%5) ? 0.0: 1.0; }
        matrix44(float ia[16]) { auto i(16); while(i--) a[i] = ia[i]; }
        float a[16];
    } matrix44;
    
    typedef struct quaternion {
        quaternion(float qi, float qj, float qk, float qr): i(qi), j(qj), k(qj), r(qr) { }
        float i, j, k, r;
        
        quaternion &operator *= (const quaternion &q) {
            *this = *this * q;
            return *this;
        }
        
        quaternion operator * (const quaternion &q1) const {
            float
                i2(r * q1.i + i * q1.r + j * q1.k - k * q1.j),
                j2(r * q1.j - i * q1.k + j * q1.r + k * q1.i),
                k2(r * q1.k + i * q1.j - j * q1.i + k * q1.r),
                r2(r * q1.r - i * q1.i - j * q1.j - k * q1.k);
                
            return quaternion(i2, j2, k2, r2);
        }
        
        vertex4 operator * (const vertex4 &v) const {
            quaternion
                qv(v.x, v.y, v.z, 0.0),
                qr(-i, -j, -k, r),
                q((*this * qv) * qr);
            
            return vertex4(q.i, q.j, q.k, 1.0);
        }
            
    } quaternion;
}

#endif /* LinearAlgebra_hpp */
