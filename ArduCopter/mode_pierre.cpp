#include "Copter.h"

/*
 * Init and run calls for guided_nogps flight mode
 */

// initialise guided_nogps controller
bool ModePierre::init(bool ignore_checks)
{
    // Initialize position controller for Z axis if not already active
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }
    

    // Set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_cm(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_NE_cm(get_pilot_speed_dn(), g.pilot_accel_z);
    return true;

    // Tiempo
    t0_us = AP_HAL::micros64();
    // ajustar al tu gusto (rad/s). Ejemplo:
    ////if (alpha == 0) alpha = 2.0f;   // default si no seteaste parámetros
    ////if (beta  == 0) beta  = 2.0f;
    return true;


}
// comentario
// guided_run - runs the guided controller
// should be called at 100hz or more

void ModePierre::exit()
{
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }

    // Set vertical speed and acceleration limits
    // pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    // pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    // pos_control->set_alt_target_with_slew(200.0f);
    // pos_control->update_z_controller();
}

namespace {
    // producto Hamilton: r = a ⊗ b
    static inline Quaternion qmul(const Quaternion& a, const Quaternion& b)
    {
        Quaternion r;
        r.q1 = a.q1*b.q1 - a.q2*b.q2 - a.q3*b.q3 - a.q4*b.q4;     // w
        r.q2 = a.q1*b.q2 + a.q2*b.q1 + a.q3*b.q4 - a.q4*b.q3;     // x
        r.q3 = a.q1*b.q3 - a.q2*b.q4 + a.q3*b.q1 + a.q4*b.q2;     // y
        r.q4 = a.q1*b.q4 + a.q2*b.q3 - a.q3*b.q2 + a.q4*b.q1;     // z
        r.normalize();
        return r;
    }

    static inline Quaternion qconj(const Quaternion& q)
    {
        Quaternion r{q.q1, -q.q2, -q.q3, -q.q4};
        return r;
    }
}


void ModePierre::run()
{
    const float t = (AP_HAL::micros64() - t0_us) * 1.0e-6f;  // [s]

    // ángulos y derivadas  
    const float phi     = 0.5f * sinf(alpha * t);                 // φ(t)
    const float theta   = 0.5f * sinf(betaa  * t);                 // θ(t)
    const float phi_dot = 0.5f * alpha * cosf(alpha * t);         // φ̇(t)
    const float th_dot  = 0.5f * betaa * cosf(betaa  * t);   // θ̇(t)  

    const float hphi   = 0.5f * phi;
    const float htheta = 0.5f * theta;

    // qdx: rotación sobre X por φ
    qdx.q1 = cosf(hphi);
    qdx.q2 = sinf(hphi);
    qdx.q3 = 0.0f;
    qdx.q4 = 0.0f;
    qdx.normalize();

    // qdy: rotación sobre Y por θ
    qdy.q1 = cosf(htheta);
    qdy.q2 = 0.0f;
    qdy.q3 = sinf(htheta);
    qdy.q4 = 0.0f;
    qdy.normalize();

    // derivadas q̇dx y q̇dy
    Quaternion qdx_dot;
    qdx_dot.q1 = -0.5f * phi_dot * sinf(hphi);
    qdx_dot.q2 =  0.5f * phi_dot * cosf(hphi);
    qdx_dot.q3 =  0.0f;
    qdx_dot.q4 =  0.0f;

    Quaternion qdy_dot;
    qdy_dot.q1 = -0.5f * th_dot * sinf(htheta);
    qdy_dot.q2 =  0.0f;
    qdy_dot.q3 =  0.5f * th_dot * cosf(htheta);
    qdy_dot.q4 =  0.0f;

    // producto qdmul = qdx ⊗ qdy  y su derivada
    ////qdmul    = qmul(qdx, qdy);
    ////const Quaternion term1    = qmul(qdx_dot, qdy);
    ////const Quaternion term2    = qmul(qdx,     qdy_dot);
    qdmul    = qdx * qdy;
    const Quaternion term1    = qdx_dot * qdy;
    const Quaternion term2    = qdx * qdy_dot;
    Quaternion qdmul_dot { term1.q1 + term2.q1,
                           term1.q2 + term2.q2,
                           term1.q3 + term2.q3,
                           term1.q4 + term2.q4 };

    // ======== ELEGIR UNO ========
    // qd = qdx;      Quaternion qd_dot = qdx_dot;      // sólo rotación en X (φ)
    //qd = qdy;      Quaternion qd_dot = qdy_dot;      // sólo rotación en Y (θ)
    qd = qdmul;       Quaternion qd_dot = qdmul_dot;    // composición qdx ⊗ qdy
    // =============================================================

    // ω_d consistente con qd elegido:  Ω = 2 * (q* ⊗ q̇), Ω=[0, ω]
    ///////const Quaternion tmp = qmul(qconj(qd), qd_dot);
    const Quaternion tmp = qconj(qd) * qd_dot;
    omegad.x = 2.0f * tmp.q2;
    omegad.y = 2.0f * tmp.q3;
    omegad.z = 2.0f * tmp.q4;

    qd.normalize();


        // Handle motor spool states
    if (!motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_U_controller(0.0f);   // forces throttle output to decay to zero
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }
    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();  
        attitude_control->reset_rate_controller_I_terms();
        pos_control->relax_U_controller(0.0f);
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();

        pos_control->relax_U_controller(0.0f);
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        
        // Flying - run quaternion controller // q_d , w_d
        attitude_control->input_quaternion(qd, omegad);
        pos_control->set_alt_target_with_slew_cm(400.0f); //4 m
        pos_control->update_U_controller();

        break;
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // Do nothing
        break;
    default:
        // Do nothing
        break;
    }

}




