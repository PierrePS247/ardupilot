#include "Copter.h"
#include "GCS_MAVLink/ekf_target.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

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

    // referencia de tiempo del modo
    t0_us = AP_HAL::micros64();
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

    // producto Hamilton SIN normalizar (necesario para omega = 2*(q*dot) )
    static inline Quaternion qmul_raw(const Quaternion& a, const Quaternion& b)
    {
        Quaternion r;
        r.q1 = a.q1*b.q1 - a.q2*b.q2 - a.q3*b.q3 - a.q4*b.q4;
        r.q2 = a.q1*b.q2 + a.q2*b.q1 + a.q3*b.q4 - a.q4*b.q3;
        r.q3 = a.q1*b.q3 - a.q2*b.q4 + a.q3*b.q1 + a.q4*b.q2;
        r.q4 = a.q1*b.q4 + a.q2*b.q3 - a.q3*b.q2 + a.q4*b.q1;
        return r;
    }

    // Cuaternión de "rotación mínima" que lleva el vector unitario a -> b (ambos en el mismo marco)
    // Útil cuando NO quieres imponer yaw.
    static inline Quaternion quat_from_unit_vecs(const Vector3f& a_unit, const Vector3f& b_unit)
    {
        const float dot = a_unit * b_unit;
        Vector3f v = a_unit % b_unit;     // cross

        // si están casi opuestos, el eje de rotación no está bien definido (dot ~ -1)
        if (dot < -0.999f) {
            Vector3f axis = (fabsf(a_unit.x) < 0.9f) ? Vector3f(1,0,0) : Vector3f(0,1,0);
            v = a_unit % axis;
            v.normalize();
            Quaternion q{0.0f, v.x, v.y, v.z}; // 180 deg
            q.normalize();
            return q;
        }

        // fórmula estándar para unit vectors:
        // q = [1+dot, cross(a,b)]
        Quaternion q{1.0f + dot, v.x, v.y, v.z};
        q.normalize();
        return q;
    }
}


void ModePierre::run()
{

    const uint32_t now_ms = AP_HAL::millis();

    Vector3f p_i_bt, v_i_bt;

    // Hold-last (para que tu modo tenga referencia aun si se cae el stream un momento)
    static bool have_hold = false;
    static Vector3f p_i_bt_hold, v_i_bt_hold;

    if (EKFTarget::get(p_i_bt, v_i_bt, now_ms, 200)) {
        // dato fresco: úsalo y guárdalo
        p_i_bt_hold = p_i_bt;
        v_i_bt_hold = v_i_bt;
        have_hold = true;
    } else if (have_hold) {
        // no hay dato fresco: reutiliza el último (esto mantiene el “mejor estimado”)
        p_i_bt = p_i_bt_hold;
        v_i_bt = v_i_bt_hold;
    } else {
        return;
    }

    // ====== Aterrizaje con sigmoide ======
    // Parámetros
    constexpr float z_land    = 0.0f;    // altura de contacto [m]
    constexpr float v_down    = 1.2f;    // velocidad de descenso [m/s]
    constexpr float rho       = 0.10f;   // umbral sigmoide XY [m]
    constexpr float alpha_sig = 30.0f;   // pendiente sigmoide XY [1/m]
    constexpr float z_f       = 0.50f;   // altura donde inicia flare [m]
    constexpr float beta_sig  = 7.0f;    // pendiente sigmoide Z [1/m]
    constexpr float z_snap    = 0.20f;   // cierre de referencia a 0 [m]

    // Estado persistente
    static float h_des = 0.0f;
    static bool  h_des_initialized = false;
    static bool  contact_detected = false;
    static uint32_t prev_run_ms   = 0;

    if (!h_des_initialized) {
        h_des = p_i_bt.z; // arranca en la altura real inicial
        h_des_initialized = true;
    }

    // Error horizontal (p_des_xy = 0)
    const float e_xy = Vector2f(p_i_bt.x, p_i_bt.y).length();

    // sigma_xy(e_xy) = 1 / (1 + exp(alpha*(e_xy - rho)))
    //   e_xy >> rho  ->  sigma_xy ~ 0  ->  hover a z_safe
    //   e_xy << rho  ->  sigma_xy ~ 1  ->  descender
    const float exp_arg_xy = constrain_float(alpha_sig * (e_xy - rho), -20.0f, 20.0f);
    const float sigma_xy = 1.0f / (1.0f + expf(exp_arg_xy));

    // sigma_z(h_des) = 1 / (1 + exp(-beta*(h_des - z_f)))
    //   h_des >> z_f -> sigma_z ~ 1  (sin flare)
    //   h_des << z_f -> sigma_z ~ 0  (flare: velocidad vertical se anula suave)
    const float exp_arg_z = constrain_float(-beta_sig * (h_des - z_f), -20.0f, 20.0f);
    const float sigma_z = 1.0f / (1.0f + expf(exp_arg_z));

    // Integrar h_des:  ḣ = -v_down * sigma_xy * sigma_z
    const float dt_land = (now_ms - prev_run_ms) * 1.0e-3f;
    prev_run_ms = now_ms;

    float h_dot = 0.0f;
    if (dt_land > 0.0f && dt_land < 0.1f) {
        h_dot = -v_down * sigma_xy * sigma_z;
        h_des += h_dot * dt_land;
        h_des = MAX(h_des, z_land);

        // Evita comportamiento asintótico: cerramos a 0 cuando ya estamos muy cerca del suelo
        // y centrados sobre el target.
        constexpr float h_des_eps = 1.0e-4f;
        if (h_des <= (z_land + h_des_eps)) {
            h_dot = 0.0f;
        }
    }

    // p_des con altura dinámica
    const Vector3f p_des(0.0f, 0.0f, h_des);

    // e_p = p_i_bt - p_des
    const Vector3f e_p = p_i_bt - p_des;

    // e'_p = v_i_bt - ṗ_des   (feedforward: el término derivativo no pelea
    //                          con el descenso intencional)
    const Vector3f p_des_dot(0.0f, 0.0f, h_dot);
    const Vector3f e_v = v_i_bt - p_des_dot;

    // Ganancias por eje: XY y Z se ajustan de forma independiente.
    // Nota: con PD puro puede persistir offset estacionario por sesgos de modelo/estimacion.
    constexpr float Kp_xy = 0.25f;   // [1/s^2] 0.25
    constexpr float Kv_xy = 1.2f;    // [1/s]   1.2
    constexpr float Kp_z  = 0.90f;   // [1/s^2] 0.90
    constexpr float Kv_z  = 2.7f;    // [1/s]   2.7
    constexpr float GRAV  = 9.80665f; // m/s^2 (usa 980.665f si trabajas en cm/s^2)

    // e3 (eje "down" en NED)
    const Vector3f e3(0.0f, 0.0f, 1.0f);

    // Ley: u = g e3 - (Kv*e_v + Kp*e_p)
    // a_des por eje (aceleración deseada del dron en NED)
    Vector3f a_des(
        (Kv_xy * e_v.x) + (Kp_xy * e_p.x),
        (Kv_xy * e_v.y) + (Kp_xy * e_p.y),
        (Kv_z  * e_v.z) + (Kp_z  * e_p.z)
    );
    //Vector3f a_des(0.0f, 0.0f, 0.1f); // ABIERTO---------------------------

    // guardar antes de saturación
    const Vector3f a_des_raw = a_des; //----------------

    // ----- Aceleración máxima horizontal -----
    const float axy_max = 3.0f;   // m/s^2 (prueba 3 a 6). Ajusta según tu dron/SITL.
    Vector2f axy(a_des.x, a_des.y);
    const float axy_norm = axy.length();
    if (axy_norm > axy_max) {
        const float s = axy_max / axy_norm;
        a_des.x *= s;
        a_des.y *= s;
    }

    // ----- Aceleración máxima vertical ----- 1.0
    // Sin esto, un error de 20m genera a_des_z = 5 m/s² → caída a 4.5 m/s²
    const float az_max = 2.0f;   // m/s^2 (limita descenso/ascenso a ~0.2g)
    a_des.z = constrain_float(a_des.z, -az_max, az_max);

    // guardar después de saturación
    const Vector3f a_des_sat = a_des; //----------------

    // u = (T/m) R e3  y en NED:  a = g e3 - u  => u = g e3 - a_des
    Vector3f u = (e3 * GRAV) - a_des;

    // Evitar degeneración
    float u_mag = u.length();
    if (u_mag < 1.0e-3f) {
        // si u es casi cero, no podemos definir dirección
        // fallback: hover "nivelado"
        u = e3 * GRAV;
        u_mag = GRAV;
    }

    // ----- Tilt limit -----
    //const float tilt_max_deg = 30.0f;
    //const float tilt_max = radians(tilt_max_deg);      // si no tienes radians(), usa tilt_max_deg * M_PI/180
    //const float tan_tilt = tanf(tilt_max);

    // Evita inversión: u.z debe ser positivo (Down) y no demasiado pequeño
    const float uz_min = 0.2f * GRAV;   // 0.2g
    if (u.z < uz_min) {
        u.z = uz_min;
    }

    // u_mag final (después de tilt-limit), para log y para throttle
    //u_mag = u.length(); //--------------------

    // Limita u_xy para respetar tilt_max: u_xy <= u_z * tan(tilt_max)
    //const float u_xy = hypotf(u.x, u.y);
    //const float u_xy_max = u.z * tan_tilt;

    //if (u_xy > u_xy_max) {
    //    const float s = u_xy_max / u_xy;
    //    u.x *= s;
    //    u.y *= s;
    //}

    // u_mag final (después de TODO el tilt-limit), para throttle y logs
    u_mag = u.length();

    // Dirección deseada del eje thrust (body z en inercial)
    Vector3f b3_des = u;
    b3_des.normalize();

    // rotación mínima que lleva (0,0,1)_body -> b3_des_inercial
    //const Vector3f body_z(0.0f, 0.0f, 1.0f);
    //Quaternion qd_local = quat_from_unit_vecs(body_z, b3_des);

    // ====== "Yaw estable" sin yaw_d explícito ======
    // Referencia fija en inercial: eje X (North en NED)
    const Vector3f a_ref(1.0f, 0.0f, 0.0f);

    // Proyección de a_ref sobre b3_des: proj = (a·b3) b3
    // Nota: en ArduPilot, (v1 * v2) suele ser dot product.
    // Si tu versión no soporta eso, usa a_ref.dot(b3_des) si existe.
    const float a_dot_b3 = a_ref * b3_des;
    Vector3f proj = b3_des * a_dot_b3;

    // Remover componente paralela a b3: v_temp = a - proj
    Vector3f v_temp = a_ref - proj;

    // Caso degenerado: si b3_des está casi alineado con a_ref,
    // entonces v_temp ~ 0 y no se puede normalizar.
    // En ese caso, usa otra referencia (eje Y inercial).
    if (v_temp.length() < 1.0e-3f) {
        const Vector3f a_ref2(0.0f, 1.0f, 0.0f);
        const float a2_dot_b3 = a_ref2 * b3_des;
        v_temp = a_ref2 - (b3_des * a2_dot_b3);
    }

    // b1_des = v_temp / ||v_temp||
    v_temp.normalize();
    Vector3f b1_des = v_temp;

    // b2_des = b3_des × b1_des  (mano derecha)
    Vector3f b2_des = b3_des % b1_des;
    if (b2_des.length() < 1.0e-6f) {
        // Muy raro si b1_des y b3_des quedaron casi paralelos por numérica
        b2_des = b3_des % Vector3f(0.0f, 1.0f, 0.0f);
    }
    b2_des.normalize();

    // (Opcional pero MUY recomendado para ortonormalidad numérica)
    // Si quieres mantener estrictamente la regla "solo un cross al final", comenta estas dos líneas.
    // b1_des = b2_des % b3_des;
    // b1_des.normalize();

    // ====== Construir R_des y qd_local ======
    // IMPORTANTE: en ArduPilot, Matrix3f.a/b/c suelen representar FILAS.
    // Como b1,b2,b3 son ejes del cuerpo expresados en inercial (COLUMNAS),
    // cargamos las FILAS como [b1.x b2.x b3.x; b1.y b2.y b3.y; b1.z b2.z b3.z]
    Matrix3f R_des;
    R_des.a = Vector3f(b1_des.x, b2_des.x, b3_des.x);
    R_des.b = Vector3f(b1_des.y, b2_des.y, b3_des.y);
    R_des.c = Vector3f(b1_des.z, b2_des.z, b3_des.z);    

    Quaternion qd_local;
    qd_local.from_rotation_matrix(R_des);
    qd_local.normalize();

    // ====== Derivar qd para ωd (finite difference) ======
    static bool have_prev = false;
    static Quaternion qd_prev;
    static uint32_t prev_ms = 0;

    Vector3f omegad_local(0.0f, 0.0f, 0.0f);

    if (have_prev) {
        const float dt = (now_ms - prev_ms) * 1.0e-3f;
        if (dt > 1.0e-3f) {

            // Asegurar continuidad del cuaternión (evitar salto de signo)
            const float dotq = qd_local.q1*qd_prev.q1 + qd_local.q2*qd_prev.q2 + qd_local.q3*qd_prev.q3 + qd_local.q4*qd_prev.q4;
            if (dotq < 0.0f) {
                qd_local.q1 = -qd_local.q1; qd_local.q2 = -qd_local.q2;
                qd_local.q3 = -qd_local.q3; qd_local.q4 = -qd_local.q4;
            }
            
            Quaternion qd_dot;
            qd_dot.q1 = (qd_local.q1 - qd_prev.q1) / dt;
            qd_dot.q2 = (qd_local.q2 - qd_prev.q2) / dt;
            qd_dot.q3 = (qd_local.q3 - qd_prev.q3) / dt;
            qd_dot.q4 = (qd_local.q4 - qd_prev.q4) / dt;

            // omega_d = 2 * (q* ⊗ q_dot)_{vec}
            //const Quaternion tmp = qconj(qd_local) * qd_dot;
            const Quaternion tmp = qmul_raw(qconj(qd_local), qd_dot);
            omegad_local.x = 2.0f * tmp.q2;
            omegad_local.y = 2.0f * tmp.q3;
            omegad_local.z = 2.0f * tmp.q4;
            
            qd_local.normalize();
        }
    }

    // actualizar memoria
    qd_prev = qd_local;
    prev_ms = now_ms;
    have_prev = true;

    // (Opcional) Magnitud de thrust: en términos de aceleración equivalente u_mag
    // si quieres mapear a throttle aproximado: throttle ≈ hover_throttle * (u_mag/GRAV)
    float throttle_local = motors->get_throttle_hover() * (u_mag / GRAV);
    if (throttle_local < 0.0f) throttle_local = 0.0f;
    if (throttle_local > 1.0f) throttle_local = 1.0f;

    // ====== Detección de contacto ======
    // `land_complete` es la señal más robusta de contacto real con suelo.
    const bool landed = copter.ap.land_complete;
    const bool contact_geom = (p_i_bt.z <= z_snap && e_xy < 0.3f);

    // Latch: una vez detectado contacto, no se libera hasta desarmar/cambiar de modo.
    if (landed || contact_geom) {
        if (!contact_detected) {
            contact_detected = true;
            gcs().send_text(MAV_SEVERITY_INFO, "PIERRE: Contact detected");
        }
    }

    // Al detectar contacto real/geométrico, forzar cero throttle para evitar rebote.
    if (contact_detected || landed) {
        throttle_local = 0.0f;
    }

    #if HAL_LOGGING_ENABLED
        // ===== Logging a BIN (50 Hz) =====
        static bool log_enabled = false;
        static uint32_t last_log_us = 0;

        // Activa logging cuando ya estamos en el aire (puedes ajustar el gating si quieres)
        if (motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
            log_enabled = true;
        }

        if (log_enabled) {
            const uint32_t now_us = AP_HAL::micros();
            if ((uint32_t)(now_us - last_log_us) >= 20000U) { // 50 Hz
                last_log_us = now_us;

                const uint64_t t64 = AP_HAL::micros64();

                AP::logger().Write(
                    "ZZP",                 // nombre corto (4 chars)
                    "TimeUS,pbx,pby,pbz",   // labels
                    "Qfff",                // 1x uint64 + 4x float
                    t64,
                    p_i_bt.x, p_i_bt.y, p_i_bt.z
                );

                AP::logger().Write(
                    "ZZV",                 // nombre corto (4 chars)
                    "TimeUS,vbx,vby,vbz",   // labels
                    "Qfff",                // 1x uint64 + 4x float
                    t64,
                    v_i_bt.x, v_i_bt.y, v_i_bt.z
                );

                AP::logger().Write(
                    "ZZEP",                 // nombre corto (4 chars)
                    "TimeUS,epx,epy,epz",   // labels
                    "Qfff",                // 1x uint64 + 4x float
                    t64,
                    e_p.x, e_p.y, e_p.z
                );

                AP::logger().Write(
                    "ZZEV",                 // nombre corto (4 chars)
                    "TimeUS,evx,evy,evz",   // labels
                    "Qfff",                // 1x uint64 + 4x float
                    t64,
                    e_v.x, e_v.y, e_v.z
                );

                AP::logger().Write(
                    "ZZAR",                 // nombre corto (4 chars)
                    "TimeUS,arx,ary,arz",   // labels
                    "Qfff",                // 1x uint64 + 4x float
                    t64,
                    a_des_raw.x, a_des_raw.y, a_des_raw.z
                );

                AP::logger().Write(
                    "ZZAS",                 // nombre corto (4 chars)
                    "TimeUS,axs,ays,azs",   // labels
                    "Qfff",                // 1x uint64 + 4x float
                    t64,
                    a_des_sat.x, a_des_sat.y, a_des_sat.z
                );

                AP::logger().Write(
                    "ZZUT",                 // nombre corto (4 chars)
                    "TimeUS,ux,uy,uz,thr",   // labels
                    "Qffff",                // 1x uint64 + 4x float
                    t64,
                    u.x, u.y, u.z, throttle_local
                );

                AP::logger().Write(
                    "ZZLD",
                    "TimeUS,hdes,h_dot,sxy,sz,exy,cntc",
                    "Qfffffi",
                    t64,
                    h_des, h_dot, sigma_xy, sigma_z, e_xy, (int)contact_detected
                );
                
            }
        }
    #endif


        // Handle motor spool states
    if (!motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_U_controller(0.0f);   // forces throttle output to decay to zero
    } else if (contact_detected || landed) {
        // Contacto detectado: apagar motores
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
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

        // Desarmar cuando hacemos contacto y motores ya están en idle
        if (contact_detected || landed) {
            gcs().send_text(MAV_SEVERITY_INFO, "PIERRE: Disarming");
            copter.arming.disarm(AP_Arming::Method::LANDED);
        }
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        
        // 1) actitud deseada (cuaternión + omega)
        attitude_control->input_quaternion(qd_local, omegad_local);

        // 2) thrust / throttle (tu conversión u -> T -> throttle)
        // Nota: el nombre exacto puede variar según tu versión, pero en Copter normalmente existe:
        // set_throttle_out(throttle, apply_angle_boost, filt)
        // apply_angle_boost = false porque throttle_local ya se basa en u_mag
        // (magnitud total del thrust, no solo componente vertical)
        attitude_control->set_throttle_out(throttle_local, false, 0.0f);

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
