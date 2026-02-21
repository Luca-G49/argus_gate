#include "packet_utils.hpp"
#include <arpa/inet.h>

RawTxData encode_packet(const argus_interfaces::msg::PlcCommand& d) {
    RawTxData p {};
    p.id = htons(200);
    p.life_word = htons(d.life_word);

    // Pack bools into bits
    p.flags = (d.ack << 0) | (d.exec << 1) | (d.fire << 2);
    p.jogs  = (d.pitch_jog_p << 0) | (d.pitch_jog_n << 1) | (d.yaw_jog_p << 2) | (d.yaw_jog_n << 3);

    p.mode = htons(static_cast<uint16_t>(d.mode));

    // Convert from float to Big Endian DINT (scaled by 100)
    p.pitch_override = htonl(static_cast<int32_t>(d.pitch_override * 100.0f));
    p.yaw_override   = htonl(static_cast<int32_t>(d.yaw_override * 100.0f));
    p.target_pitch   = htonl(static_cast<int32_t>(d.target_pitch * 100.0f));
    p.target_yaw     = htonl(static_cast<int32_t>(d.target_yaw * 100.0f));

    p.checksum = htons(d.checksum);
    return p;
}

argus_interfaces::msg::PlcStatus decode_packet(const RawRxData& raw) {
    argus_interfaces::msg::PlcStatus s;
    s.life_word = ntohs(raw.life_word);

    // Unpack bits into bools
    s.done      = (raw.flags >> 0) & 1;
    s.busy      = (raw.flags >> 1) & 1;
    s.synch     = (raw.flags >> 2) & 1;
    s.on_target = (raw.flags >> 3) & 1;

    s.status    = static_cast<int16_t>(ntohs(raw.status));
    s.error     = static_cast<int16_t>(ntohs(raw.error));

    // Convert from Big Endian and scale back to float
    s.pos_pitch = static_cast<float>(static_cast<int32_t>(ntohl(raw.pos_pitch))) / 100.0f;
    s.pos_yaw   = static_cast<float>(static_cast<int32_t>(ntohl(raw.pos_yaw))) / 100.0f;

    s.checksum  = ntohs(raw.checksum);
    return s;
}