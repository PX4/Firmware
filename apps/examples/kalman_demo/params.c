#include <systemlib/param/param.h>

/*PARAM_DEFINE_FLOAT(NAME,0.0f);*/
PARAM_DEFINE_FLOAT(KF_V_GYRO,0.01f);
PARAM_DEFINE_FLOAT(KF_V_ACCEL,0.01f);
PARAM_DEFINE_FLOAT(KF_R_MAG,1.0e-4f);
PARAM_DEFINE_FLOAT(KF_R_GPS_V,0.0f);
PARAM_DEFINE_FLOAT(KF_R_GPS_GEO,0.0f);
PARAM_DEFINE_FLOAT(KF_R_GPS_ALT,0.0f);
PARAM_DEFINE_FLOAT(KF_R_ACCEL,1.0e-4f);
