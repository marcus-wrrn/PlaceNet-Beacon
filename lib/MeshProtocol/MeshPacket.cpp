#include "MeshPacket.h"

#include <cstring>

namespace meshcore {

// Header bit masks (0bVVPPPPRR).
static constexpr uint8_t HDR_ROUTE_MASK   = 0x03;  // bits 0-1
static constexpr uint8_t HDR_TYPE_MASK    = 0x3C;  // bits 2-5
static constexpr uint8_t HDR_TYPE_SHIFT   = 2;
static constexpr uint8_t HDR_VER_MASK     = 0xC0;  // bits 6-7
static constexpr uint8_t HDR_VER_SHIFT    = 6;

// path_length bit layout (0bSSHHHHHH).
static constexpr uint8_t PATH_HOP_MASK    = 0x3F;  // bits 0-5
static constexpr uint8_t PATH_HASH_SHIFT  = 6;     // bits 6-7 hold (hashSize-1)

static constexpr size_t TRANSPORT_CODE_BYTES = 4;  // 2x uint16_t

MeshPacket::MeshPacket()
    : routeType(ROUTE_TYPE_FLOOD),
      payloadType(PAYLOAD_TYPE_RAW_CUSTOM),
      payloadVersion(PAYLOAD_VER_1),
      transportCode1(0),
      transportCode2(0),
      hashSize(1),
      hopCount(0),
      payloadLen(0) {
    memset(path, 0, sizeof(path));
    memset(payload, 0, sizeof(payload));
}

bool MeshPacket::hasTransportCodes() const {
    return routeType == ROUTE_TYPE_TRANSPORT_FLOOD ||
           routeType == ROUTE_TYPE_TRANSPORT_DIRECT;
}

size_t MeshPacket::encodedSize() const {
    return 1                                                  // header
         + (hasTransportCodes() ? TRANSPORT_CODE_BYTES : 0)   // transport codes
         + 1                                                  // path_length
         + pathBytes()                                        // path
         + payloadLen;                                        // payload
}

size_t MeshPacket::serialize(uint8_t* out, size_t cap) const {
    if (hashSize < 1 || hashSize > 3) return 0;          
    if (hopCount > MESH_MAX_HOPS) return 0;
    if (pathBytes() > MESH_MAX_PATH_SIZE) return 0;
    if (payloadLen > MESH_MAX_PAYLOAD_SIZE) return 0;

    const size_t total = encodedSize();
    if (out == nullptr || cap < total) return 0;

    size_t pos = 0;

    // header
    out[pos++] = static_cast<uint8_t>(
        ((payloadVersion << HDR_VER_SHIFT) & HDR_VER_MASK) |
        ((payloadType << HDR_TYPE_SHIFT) & HDR_TYPE_MASK) |
        (routeType & HDR_ROUTE_MASK));

    // transport codes
    if (hasTransportCodes()) {
        out[pos++] = static_cast<uint8_t>(transportCode1 & 0xFF);
        out[pos++] = static_cast<uint8_t>((transportCode1 >> 8) & 0xFF);
        out[pos++] = static_cast<uint8_t>(transportCode2 & 0xFF);
        out[pos++] = static_cast<uint8_t>((transportCode2 >> 8) & 0xFF);
    }

    // path_length
    out[pos++] = static_cast<uint8_t>(
        (static_cast<uint8_t>(hashSize - 1) << PATH_HASH_SHIFT) |
        (hopCount & PATH_HOP_MASK));

    // path + payload
    memcpy(out + pos, path, pathBytes());
    pos += pathBytes();
    memcpy(out + pos, payload, payloadLen);
    pos += payloadLen;

    return pos;
}

bool MeshPacket::parse(const uint8_t* in, size_t len) {
    if (in == nullptr) return false;

    // Minimum frame is header + path_length.
    size_t pos = 0;
    if (len < pos + 1) return false;

    const uint8_t header = in[pos++];
    routeType      = static_cast<RouteType>(header & HDR_ROUTE_MASK);
    payloadType    = static_cast<PayloadType>((header & HDR_TYPE_MASK) >> HDR_TYPE_SHIFT);
    payloadVersion = static_cast<PayloadVersion>((header & HDR_VER_MASK) >> HDR_VER_SHIFT);

    if (hasTransportCodes()) {
        if (len < pos + TRANSPORT_CODE_BYTES) return false;
        transportCode1 = static_cast<uint16_t>(in[pos]) |
                         (static_cast<uint16_t>(in[pos + 1]) << 8);
        transportCode2 = static_cast<uint16_t>(in[pos + 2]) |
                         (static_cast<uint16_t>(in[pos + 3]) << 8);
        pos += TRANSPORT_CODE_BYTES;
    } else {
        transportCode1 = 0;
        transportCode2 = 0;
    }

    if (len < pos + 1) return false;
    const uint8_t pathLength = in[pos++];
    hopCount = pathLength & PATH_HOP_MASK;
    hashSize = static_cast<uint8_t>((pathLength >> PATH_HASH_SHIFT) + 1);
    if (hashSize > 3) return false;  // 0b11 -> hashSize 4 is reserved/invalid

    const size_t pathLen = pathBytes();
    if (pathLen > MESH_MAX_PATH_SIZE) return false;
    if (len < pos + pathLen) return false;
    memcpy(path, in + pos, pathLen);
    pos += pathLen;

    // Whatever remains is the payload.
    const size_t remaining = len - pos;
    if (remaining > MESH_MAX_PAYLOAD_SIZE) return false;
    payloadLen = static_cast<uint8_t>(remaining);
    memcpy(payload, in + pos, remaining);

    return true;
}

}  // namespace meshcore
