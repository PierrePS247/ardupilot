#include "ekf_target.h"

#include <string.h>                 // memcpy, strcmp
#include <AP_HAL/AP_HAL.h>          // AP_HAL::millis()

EKFTarget::State EKFTarget::s = {false, 0, 0, Vector3f{}, Vector3f{}};

void EKFTarget::reset()
{
    s.valid = false;
    s.mask = 0;
    s.last_ms = 0;
    s.p = Vector3f{};
    s.v = Vector3f{};
}

void EKFTarget::set_bit(uint8_t bit, uint32_t now_ms)
{
    s.mask |= (1U << bit);
    s.last_ms = now_ms;
    // 0x3F = 0b111111: ya recibimos las 6 componentes al menos una vez
    if (s.mask == 0x3F) {
        s.valid = true;
    }
}

void EKFTarget::update_named(const char *name, float value, uint32_t now_ms)
{
    // Nombres esperados desde tu companion (Python):
    // "ekf_px","ekf_py","ekf_pz","ekf_vx","ekf_vy","ekf_vz"
    if (!strcmp(name, "ekf_px")) { s.p.x = value; set_bit(0, now_ms); }
    else if (!strcmp(name, "ekf_py")) { s.p.y = value; set_bit(1, now_ms); }
    else if (!strcmp(name, "ekf_pz")) { s.p.z = value; set_bit(2, now_ms); }
    else if (!strcmp(name, "ekf_vx")) { s.v.x = value; set_bit(3, now_ms); }
    else if (!strcmp(name, "ekf_vy")) { s.v.y = value; set_bit(4, now_ms); }
    else if (!strcmp(name, "ekf_vz")) { s.v.z = value; set_bit(5, now_ms); }
}

void EKFTarget::handle_named_value_float(const mavlink_message_t &msg)
{
    mavlink_named_value_float_t nv;
    mavlink_msg_named_value_float_decode(&msg, &nv);

    // nv.name es un array fijo (10 bytes). Puede no estar terminado en '\0'
    char name[11] = {};
    memcpy(name, nv.name, 10);
    name[10] = '\0';

    const uint32_t now = AP_HAL::millis();
    update_named(name, nv.value, now);
}

bool EKFTarget::get(Vector3f &p_i_bt, Vector3f &v_i_bt,
                    uint32_t now_ms, uint32_t timeout_ms)
{
    if (!s.valid) {
        return false;
    }
    if ((now_ms - s.last_ms) > timeout_ms) {
        return false;
    }
    p_i_bt = s.p;
    v_i_bt = s.v;
    return true;
}
