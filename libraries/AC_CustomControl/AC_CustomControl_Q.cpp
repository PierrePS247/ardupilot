#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_Q_ENABLED

#include "AC_CustomControl_Q.h"
#include <GCS_MAVLink/GCS.h>
#include <cmath>
#include <AP_Logger/AP_Logger.h>  
#include <AP_HAL/Semaphores.h>

// -------------------------
// Tabla de parámetros
// -------------------------
const AP_Param::GroupInfo AC_CustomControl_Q::var_info[] = {
    // Kd
    AP_GROUPINFO("KDX",   1, AC_CustomControl_Q, Kd_x,   0.23f), 
    AP_GROUPINFO("KDY",   2, AC_CustomControl_Q, Kd_y,   0.23f), 
    AP_GROUPINFO("KDZ",   3, AC_CustomControl_Q, Kd_z,   0.23f), 

    // Alpha
    AP_GROUPINFO("ALPX",  4, AC_CustomControl_Q, Alp_x, 8.0f),
    AP_GROUPINFO("ALPY",  5, AC_CustomControl_Q, Alp_y, 8.0f),         
    AP_GROUPINFO("ALPZ",  6, AC_CustomControl_Q, Alp_z, 8.0f),         

    // Gamma
    AP_GROUPINFO("GMX",   7, AC_CustomControl_Q, Gam_x, 0.000245f),
    AP_GROUPINFO("GMY",   8, AC_CustomControl_Q, Gam_y, 0.000315f),
    AP_GROUPINFO("GMZ",   9, AC_CustomControl_Q, Gam_z, 0.00049f),

    // beta
    AP_GROUPINFO("BETA", 10, AC_CustomControl_Q, beta,   0.0000f),

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
    //qbn_ant = Quaternion(1,0,0,0);
    //qd_ant = Quaternion(1,0,0,0);
    //qdx = Quaternion(1,0,0,0);
    //qdy = Quaternion(1,0,0,0);
    qflag = false;
    t0_us = AP_HAL::micros64();////
}

// -------------------------
// Reset
// -------------------------
void AC_CustomControl_Q::reset(void)
{
    omega_cmd.zero();
    first_run = true;
    t0_us = AP_HAL::micros64();
}

Quaternion AC_CustomControl_Q::qconj(const Quaternion& q)
{
    Quaternion r{q.q1, -q.q2, -q.q3, -q.q4};
    return r;
}

// -------------------------
// Update
// -------------------------
Vector3f AC_CustomControl_Q::update(void)
{
    // Estados actuales
    Quaternion q_bn;                             // cuerpo->NED
    _ahrs->get_quat_body_to_ned(q_bn);  //me estoy quedando en frame NED        // <-- En tu rama esta es la que existe
    //_ahrs->get_quaternion(q_bn); //////////////////////////////////////////////////////
    //// return the quaternion defining the rotation from NED to XYZ (body) axes
    
    Vector3f omega_b = _ahrs->get_gyro_latest();  // rad/s

    
    // Referencias deseadas (tu modo de vuelo las fija)
    Quaternion qd_bn = _att_control->get_attitude_target_quat(); // This represents the desired orientation in NED earth frame.
    Vector3f  omega_d = _att_control->get_attitude_target_ang_vel(); // Return the angular velocity of the target (setpoint) [rad/s] in the target attitude frame


    qd_bn.normalize();
    q_bn.normalize();
    
    // if (!qflag){
    //     qbn_ant = q_bn;
    //     qd_ant = qd_bn;
    //     qflag = true;
    // }

    // if (qbn_ant.q1 * q_bn.q1 + qbn_ant.q2 * q_bn.q2 + qbn_ant.q3 * q_bn.q3 + qbn_ant.q4 * q_bn.q4 < 0.0){
    //     q_bn.q1 = -q_bn.q1;
    //     q_bn.q2 = -q_bn.q2;
    //     q_bn.q3 = -q_bn.q3;
    //     q_bn.q4 = -q_bn.q4;

    // }

    // if (qd_ant.q1 * qd_bn.q1 + qd_ant.q2 * qd_bn.q2 + qd_ant.q3 * qd_bn.q3 + qd_ant.q4 * qd_bn.q4 < 0.0){
    //     qd_bn.q1 = -qd_bn.q1;
    //     qd_bn.q2 = -qd_bn.q2;
    //     qd_bn.q3 = -qd_bn.q3;
    //     qd_bn.q4 = -qd_bn.q4;
    // }


    // Error de cuaternión: qe = qd^{-1} * q
    Quaternion qe = qd_bn.inverse() * q_bn;
    qe.normalize();
    Vector3f qe_v(qe.q2, qe.q3, qe.q4);   // parte vectorial del error


    // Error de velocidad
    Vector3f omega_e = omega_b - omega_d;

    // === Extraer parámetros a escalares float ===
    const float kdx = Kd_x.get(), kdy = Kd_y.get(), kdz = Kd_z.get();
    const float ax  = Alp_x.get(), ay  = Alp_y.get(), az  = Alp_z.get();
    const float gx  = Gam_x.get(), gy  = Gam_y.get(), gz  = Gam_z.get();
    ////const float beta_v = beta.get();

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
    // tau_i =  s_r_i-Kd_i * - beta * nonlin_i
    Vector3f tau( -kdx * s_r.x - beta * nonlin.x,
                 -kdy * s_r.y - beta * nonlin.y,
                 -kdz * s_r.z - beta * nonlin.z );

                 //Para lazo abierto
    //Vector3f tau( -kdx * omega_b.x -1.84 * q_bn.q2 ,
    //            -kdy * omega_b.y -1.84 * q_bn.q3 + 0.5* qd_bn.q3,
    //            -kdz * omega_b.z -1.84 * q_bn.q4 );


    // ===== LOGGING SEGURO (50 Hz), DESPUÉS DE CALCULAR q_bn, omega_b, tau, etc. =====
    #if HAL_LOGGING_ENABLED
    // activa logging cuando ya estamos en el aire

    if (_motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
        log_enabled = true;
    }

    if (log_enabled) {
        const uint32_t now_us = AP_HAL::micros();
        if (now_us - last_log_us >= 20000U) {  // 50 Hz
            last_log_us = now_us;
            const uint64_t t64 = AP_HAL::micros64();

            // 1) Cuaternión (actitud real) body->NED
            //   Tag 4 letras: "ZQBD"
            AP::logger().Write(
                "ZQBD",                 // nombre corto (4 chars)
                "TimeUS,q1,q2,q3,q4",   // labels
                "Qffff",                // 1x uint64 + 4x float
                t64,
                (float)q_bn.q1, (float)q_bn.q2, (float)q_bn.q3, (float)q_bn.q4
            );

            // 2) Velocidad angular real (gyro)
            //   Tag: "ZWBD"
            AP::logger().Write(
                "ZWBD",
                "TimeUS,wx,wy,wz",
                "Qfff",
                t64,
                (float)omega_b.x, (float)omega_b.y, (float)omega_b.z
            );

            // 3) Torques calculados por tu ley (tau)
            //   Tag: "ZTOR"
            AP::logger().Write(
                "ZTOR",
                "TimeUS,tx,ty,tz",
                "Qfff",
                t64,
                (float)tau.x, (float)tau.y, (float)tau.z
            );

            // 4) (opcional) Error de quat (vector + escalar) – dividido en dos mensajes simples:
            //   Tag: "ZQER" (vector de error)
            AP::logger().Write(
                "ZQER",
                "TimeUS,eqx,eqy,eqz",
                "Qfff",
                t64,
                (float)qe_v.x, (float)qe_v.y, (float)qe_v.z
            );
            //   Tag: "ZQEW" (escalar del error, por separado)
            AP::logger().Write(
                "ZQEW",
                "TimeUS,ew",
                "Qf",
                t64,
                (float)qe.q1
            );

            // 5) (opcional) Error de velocidad angular
            //   Tag: "ZWER"
            AP::logger().Write(
                "ZWER",
                "TimeUS,ewx,ewy,ewz",
                "Qfff",
                t64,
                (float)(omega_b.x - omega_d.x),
                (float)(omega_b.y - omega_d.y),
                (float)(omega_b.z - omega_d.z)
            );

            // quaternion deseado
            AP::logger().Write(
                "ZQD",
                "TimeUS,qd1,qd2,qd3,qd4",   // labels
                "Qffff",                // 1x uint64 + 4x float
                t64,
                (float)qd_bn.q1, (float)qd_bn.q2, (float)qd_bn.q3, (float)qd_bn.q4
            );

            //   velocidad angular deseada
            AP::logger().Write(
                "ZWD",
                "TimeUS,wdx,wdy,wdz",
                "Qfff",
                t64,
                (float)omega_d.x, (float)omega_d.y, (float)omega_d.z
            );

        }
    }
    #endif  // HAL_LOGGING_ENABLED
    
    // qd_ant = qd_bn;
    // qbn_ant = q_bn;

    return tau;
    
}

#endif  // AP_CUSTOMCONTROL_Q_ENABLED
