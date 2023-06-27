

#include "Vectors.h"
#include <math.h>


VECT_3D VECT_Add3D(VECT_3D v1, VECT_3D v2){
    v1.x += v2.x;
    v1.y += v2.y;
    v1.z += v2.z;
    return v1;
}
VECT_3D VECT_Sub3D(VECT_3D v1, VECT_3D v2){
    v1.x -= v2.x;
    v1.y -= v2.y;
    v1.z -= v2.z;
    return v1;
}
float VECT_Dot3D(VECT_3D v1, VECT_3D v2)
{
    return (v1.x*v2.x) + (v1.y*v2.y) + (v1.z*v2.z);
}
VECT_3D VECT_Cros3D(VECT_3D v1, VECT_3D v2){
    VECT_3D v3;
    v3.x = (v1.y*v2.z) - (v1.z*v2.y);
    v3.y = (v1.z*v2.x) - (v1.x*v2.z);
    v3.z = (v1.x*v2.y) - (v1.y*v2.x);
    return v3;
}
float VECT_Mag3D(VECT_3D v){
    return sqrt(pow(v.x,2)+pow(v.y,2)+pow(v.z,2));
}
