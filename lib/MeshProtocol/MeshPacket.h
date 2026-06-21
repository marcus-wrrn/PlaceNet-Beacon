#pragma once

#include <cstddef>
#include <cstdint>

// MeshCore-compatible packet framing (wire format only — no radio, no crypto).
//
// Frame layout (see MeshCore docs/packet_format.md, v1 protocol):
//
//   [header:1][transport_codes:0|4][path_length:1][path:0..64][payload:0..184]
//
//   header      0bVVPPPPRR  V=PayloadVersion P=PayloadType R=RouteType
//   path_length 0bSSHHHHHH  S=hashSize-1     H=hopCount (0..63)
//
// All multi-byte integer fields on the wire are little-endian.
namespace meshcore {

// Hard limits enforced by MeshCore firmware (Dispatcher, v1.x). Frames that
// exceed these are dropped by MeshCore nodes
static constexpr uint8_t MESH_MAX_PATH_SIZE    = 64;
static constexpr uint8_t MESH_MAX_PAYLOAD_SIZE = 184;
static constexpr uint8_t MESH_MAX_HOPS         = 63; 

// header bits 0-1 — how the packet is routed through the mesh.
enum RouteType : uint8_t {
    ROUTE_TYPE_TRANSPORT_FLOOD  = 0x00, 
    ROUTE_TYPE_FLOOD            = 0x01,
    ROUTE_TYPE_DIRECT           = 0x02,
    ROUTE_TYPE_TRANSPORT_DIRECT = 0x03,
};

enum PayloadType : uint8_t {
    PAYLOAD_TYPE_REQ        = 0x00,
    PAYLOAD_TYPE_RESPONSE   = 0x01,
    PAYLOAD_TYPE_TXT_MSG    = 0x02,
    PAYLOAD_TYPE_ACK        = 0x03,
    PAYLOAD_TYPE_ADVERT     = 0x04,
    PAYLOAD_TYPE_GRP_TXT    = 0x05,
    PAYLOAD_TYPE_GRP_DATA   = 0x06,
    PAYLOAD_TYPE_ANON_REQ   = 0x07,
    PAYLOAD_TYPE_PATH       = 0x08,
    PAYLOAD_TYPE_TRACE      = 0x09,
    PAYLOAD_TYPE_MULTIPART  = 0x0A,
    PAYLOAD_TYPE_CONTROL    = 0x0B,
    PAYLOAD_TYPE_RAW_CUSTOM = 0x0F, // what PlaceNet uses for now, will avoid flood routing for the time being
};

enum PayloadVersion : uint8_t {
    PAYLOAD_VER_1 = 0x00,  // 1-byte src/dest hashes, 2-byte MAC
    PAYLOAD_VER_2 = 0x01,
    PAYLOAD_VER_3 = 0x02,
    PAYLOAD_VER_4 = 0x03,
};

class MeshPacket {
public:
    RouteType      routeType;
    PayloadType    payloadType;
    PayloadVersion payloadVersion;

    // Transport codes — present on the wire only for the TRANSPORT_* route
    // types. Ignored otherwise.
    uint16_t transportCode1;
    uint16_t transportCode2;

    uint8_t hashSize;
    uint8_t hopCount;
    uint8_t path[MESH_MAX_PATH_SIZE];

    uint8_t payload[MESH_MAX_PAYLOAD_SIZE];
    uint8_t payloadLen;

    MeshPacket();
    bool hasTransportCodes() const;
    size_t pathBytes() const { return static_cast<size_t>(hopCount) * hashSize; }
    size_t encodedSize() const;
    size_t serialize(uint8_t* out, size_t cap) const;
    bool parse(const uint8_t* in, size_t len);
};

}  // namespace meshcore
