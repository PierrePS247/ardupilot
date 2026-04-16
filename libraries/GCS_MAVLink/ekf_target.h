#pragma once

#include <stdint.h>
#include <AP_Math/AP_Math.h>                 // Vector3f
#include <GCS_MAVLink/GCS_MAVLink.h>         // mavlink_message_t, mavlink_named_value_float_t, decode

class EKFTarget {
public:
    // Llamar desde el switch MAVLink cuando llegue NAMED_VALUE_FLOAT
    static void handle_named_value_float(const mavlink_message_t &msg);

    // Leer el último estado recibido.
    // - timeout_ms: considera inválido si es más viejo que timeout_ms
    // Retorna true si hay dato válido y fresco.
    static bool get(Vector3f &p_i_bt, Vector3f &v_i_bt,
                    uint32_t now_ms, uint32_t timeout_ms = 200);

    // Para debug / reset
    static void reset();

private:
    struct State {
        bool valid;
        uint8_t mask;       // bits 0..5 indican si llegaron px..vz
        uint32_t last_ms;
        Vector3f p;         // ^i p^b_t  (NED)
        Vector3f v;         // ^i v^b_t
    };

    static State s;

    // helpers
    static void update_named(const char *name, float value, uint32_t now_ms);
    static void set_bit(uint8_t bit, uint32_t now_ms);
};
