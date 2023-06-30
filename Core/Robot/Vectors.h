#ifndef VECT3D_H_
#define VECT3D_H_


typedef struct Vect3D{
    float x;
    float y;
    float z;
    
}VECT_3D;

typedef struct Vect2D{
    float x;
    float y;
}VECT_2D;

VECT_3D VECT_Add3D(VECT_3D v1, VECT_3D v2);
VECT_3D VECT_Sub3D(VECT_3D v1, VECT_3D v2);
float VECT_Dot3D(VECT_3D v1, VECT_3D v2);
VECT_3D VECT_Cros3D(VECT_3D v1, VECT_3D v2);
float VECT_Mag3D(VECT_3D v);
float VECT_AngleXY(VECT_3D p1, VECT_3D p2);
float VECT_DistXY(VECT_3D p1, VECT_3D p2);





#endif /* VECT3D_H_ */
