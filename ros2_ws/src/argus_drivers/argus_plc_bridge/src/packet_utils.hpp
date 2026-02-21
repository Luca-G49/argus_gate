#ifndef PACKET_UTILS_HPP
#define PACKET_UTILS_HPP

#include "argus_interfaces/msg/plc_command.hpp"
#include "argus_interfaces/msg/plc_status.hpp"
#include "plc_bridge_node.hpp"  // For RawTxData and RawRxData

/**
 * @brief Encode PlcCommand into the RawTxData binary format (Big Endian + Scaling).
 */
RawTxData encode_packet(const argus_interfaces::msg::PlcCommand& d);

/**
 * @brief Decode the RawRxData binary struct (Big Endian + Scaling) into PlcStatus.
 */
argus_interfaces::msg::PlcStatus decode_packet(const RawRxData& raw);

#endif // PACKET_UTILS_HPP