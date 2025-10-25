#pragma once

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_Q_ENABLED

#include "AC_CustomControl_Backend.h"
#include <AP_Math/AP_Math.h>

class AC_CustomControl_Q : public AC_CustomControl_Backend {
public:
    AC_CustomControl_Q(AC_CustomControl& frontend,
                       AP_AHRS_View*& ahrs,
                       AC_AttitudeControl*& att_control,
                       AP_MotorsMulticopter*& motors,
                       float dt);

    Vector3f update(void) override;
    void reset(void) override;
    Quaternion qconj(const Quaternion& q);

    static const struct AP_Param::GroupInfo var_info[];

protected:
    // ===== Ganancias (por componente, expuestas como parámetros) =====
    AP_Float Kd_x,    Kd_y,    Kd_z;     // Kd diagonal
    AP_Float Alp_x,   Alp_y,   Alp_z;    // Alpha diagonal
    AP_Float Gam_x,   Gam_y,   Gam_z;    // Gamma diagonal
    AP_Float beta;                        // escalar

    Quaternion qbn_ant;
    Quaternion qd_ant;
    Quaternion qdx;
    Quaternion qdy;
    Quaternion qmul;
    Quaternion qdmul;
    Quaternion qd;
    Vector3f   omegad;
    Vector3f   omega_d;
    Quaternion qd_bn;
    bool qflag;
    uint64_t   t0_us{0};//////
    float      alpha{1.0f}, betaa{1.0f};///////

    // ===== Inercias =====
    AP_Float Ixx, Iyy, Izz;

    // ===== Límites =====
    AP_Float rlim_x, rlim_y, rlim_z;     // |omega_cmd| [rad/s]
    AP_Float dlim_x, dlim_y, dlim_z;     // |domega_cmd| [rad/s^2]

    // ===== Estado interno =====
    Vector3f omega_cmd;
    bool     first_run = true;

    // <<< NUEVO: dt local del controlador >>>
    float    dt_s = 0.0f;

    // === logging state ===
    bool    log_enabled = false;    // activamos tras despegar
    uint32_t last_log_us = 0;       // para rate-limit


    inline Vector3f tanh_vec(const Vector3f& v) const {
        return Vector3f(tanf(v.x), tanf(v.y), tanf(v.z));
    }
};

#endif  // AP_CUSTOMCONTROL_Q_ENABLED


