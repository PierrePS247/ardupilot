#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_Q_ENABLED

#include "AC_CustomControl_Q.h"
#include <GCS_MAVLink/GCS.h>
#include <cmath>

// -------------------------
// Tabla de parámetros
// -------------------------
const AP_Param::GroupInfo AC_CustomControl_Q::var_info[] = {
    // Kd
    AP_GROUPINFO("KDX",   1, AC_CustomControl_Q, Kd_x,   0.35f),
    AP_GROUPINFO("KDY",   2, AC_CustomControl_Q, Kd_y,   0.45f),
    AP_GROUPINFO("KDZ",   3, AC_CustomControl_Q, Kd_z,   0.7f),

    // Alpha
    AP_GROUPINFO("ALPX",  4, AC_CustomControl_Q, Alp_x, 1.504f),
    AP_GROUPINFO("ALPY",  5, AC_CustomControl_Q, Alp_y, 1.504f),
    AP_GROUPINFO("ALPZ",  6, AC_CustomControl_Q, Alp_z, 1.504f),

    // Gamma
    AP_GROUPINFO("GMX",   7, AC_CustomControl_Q, Gam_x, 15.0f),
    AP_GROUPINFO("GMY",   8, AC_CustomControl_Q, Gam_y, 15.0f),
    AP_GROUPINFO("GMZ",   9, AC_CustomControl_Q, Gam_z, 15.0f),

    // beta
    AP_GROUPINFO("BETA", 10, AC_CustomControl_Q, beta,   0.0f),

    // Inercias
    AP_GROUPINFO("IXX",  11, AC_CustomControl_Q, Ixx,    0.004856f),
    AP_GROUPINFO("IYY",  12, AC_CustomControl_Q, Iyy,    0.004856f),
    AP_GROUPINFO("IZZ",  13, AC_CustomControl_Q, Izz,    0.008801f),

    // Límites de tasa (rad/s)
    AP_GROUPINFO("RLIMX",14, AC_CustomControl_Q, rlim_x, radians(220.0f)),
    AP_GROUPINFO("RLIMY",15, AC_CustomControl_Q, rlim_y, radians(220.0f)),
    AP_GROUPINFO("RLIMZ",16, AC_CustomControl_Q, rlim_z, radians(200.0f)),

    // Límites de aceleración angular (rad/s^2)
    AP_GROUPINFO("DLIMX",17, AC_CustomControl_Q, dlim_x, 1000.0f),
    AP_GROUPINFO("DLIMY",18, AC_CustomControl_Q, dlim_y, 1000.0f),
    AP_GROUPINFO("DLIMZ",19, AC_CustomControl_Q, dlim_z,  800.0f),

    AP_GROUPEND
};

// -------------------------
// Constructor
// -------------------------
AC_CustomControl_Q::AC_CustomControl_Q(AC_CustomControl& frontend,
                                       AP_AHRS_View*& ahrs,
                                       AC_AttitudeControl*& att_control,
                                       AP_MotorsMulticopter*& motors,
                                       float dt)
: AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
    omega_cmd = Vector3f(0,0,0);
    dt_s = dt; 
}

// -------------------------
// Reset
// -------------------------
void AC_CustomControl_Q::reset(void)
{
    omega_cmd.zero();
    first_run = true;
}

// -------------------------
// Update
// -------------------------
Vector3f AC_CustomControl_Q::update(void)
{
    // Reset si estamos en tierra
    switch (_motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            reset();
            break;
        default: break;
    }

    // Estados actuales
    Quaternion q_bn;                             // cuerpo->NED
    _ahrs->get_quat_body_to_ned(q_bn);          // <-- En tu rama esta es la que existe

    Vector3f omega_b = _ahrs->get_gyro_latest();  // rad/s

    // Referencias deseadas (tu modo de vuelo las fija)
    Quaternion qd_bn = _att_control->get_attitude_target_quat();
    Vector3f  omega_d = _att_control->get_attitude_target_ang_vel();

    // Error de cuaternión: qe = qd^{-1} * q
    Quaternion qe = qd_bn.inverse() * q_bn;
    qe.normalize();                              // (tu rama usa 'normalize')
    if (qe.q4 < 0.0f) { qe.q1 = -qe.q1; qe.q2 = -qe.q2; qe.q3 = -qe.q3; qe.q4 = -qe.q4; }
    Vector3f qe_v(qe.q1, qe.q2, qe.q3);

    // Error de velocidad
    Vector3f omega_e = omega_b - omega_d;

    // === Extraer parámetros a escalares float ===
    const float kdx = Kd_x.get(), kdy = Kd_y.get(), kdz = Kd_z.get();
    const float ax  = Alp_x.get(), ay  = Alp_y.get(), az  = Alp_z.get();
    const float gx  = Gam_x.get(), gy  = Gam_y.get(), gz  = Gam_z.get();
    const float beta_v = beta.get();

    const float ixx = Ixx.get(), iyy = Iyy.get(), izz = Izz.get();
    const float rlx = rlim_x.get(), rly = rlim_y.get(), rlz = rlim_z.get();
    const float dlx = dlim_x.get(), dly = dlim_y.get(), dlz = dlim_z.get();

    // Reconstruir vectores
    const Vector3f Alpha(ax, ay, az);
    const Vector3f Kd(kdx, kdy, kdz);
    const Vector3f Gam(gx, gy, gz);

    // Superficie s_r = (omega - omega_d) + Alpha * qe_v
    Vector3f s_r = omega_e + Vector3f(Alpha.x * qe_v.x,
                                    Alpha.y * qe_v.y,
                                    Alpha.z * qe_v.z);

    // No lineal: tanh(Gamma ∘ s_r)  (∘ = producto elemento a elemento)
    Vector3f nonlin = tanh_vec(Vector3f(Gam.x * s_r.x,
                                        Gam.y * s_r.y,
                                        Gam.z * s_r.z));

    // Ley de control (componente a componente):
    // tau_i = -Kd_i * s_r_i - beta * nonlin_i
    Vector3f tau( -kdx * s_r.x - beta_v * nonlin.x,
                -kdy * s_r.y - beta_v * nonlin.y,
                -kdz * s_r.z - beta_v * nonlin.z );


    // Dinámica inversa -> domega_cmd
    const Vector3f J(ixx, iyy, izz);
    const Vector3f J_omega(J.x * omega_b.x, J.y * omega_b.y, J.z * omega_b.z);
    const Vector3f coriolis = omega_b % J_omega;

    Vector3f domega_cmd( (tau.x + coriolis.x) / J.x,
                         (tau.y + coriolis.y) / J.y,
                         (tau.z + coriolis.z) / J.z );

    // Limitar aceleración e integrar
    const Vector3f dlim(dlx, dly, dlz);
    domega_cmd.x = constrain_float(domega_cmd.x, -dlim.x, dlim.x);
    domega_cmd.y = constrain_float(domega_cmd.y, -dlim.y, dlim.y);
    domega_cmd.z = constrain_float(domega_cmd.z, -dlim.z, dlim.z);

    if (first_run) { omega_cmd = omega_b; first_run = false; }
    omega_cmd += domega_cmd * dt_s;

    // Limitar tasa objetivo
    const Vector3f rlim(rlx, rly, rlz);
    omega_cmd.x = constrain_float(omega_cmd.x, -rlim.x, rlim.x);
    omega_cmd.y = constrain_float(omega_cmd.y, -rlim.y, rlim.y);
    omega_cmd.z = constrain_float(omega_cmd.z, -rlim.z, rlim.z);

    // Salida: tasas objetivo (p,q,r)
    return omega_cmd;

}

#endif  // AP_CUSTOMCONTROL_Q_ENABLED
