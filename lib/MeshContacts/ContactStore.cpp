#include "ContactStore.h"

#include <cstring>
#include <cstdlib>

#if defined(ESP_PLATFORM) || defined(ARDUINO)
#include <esp_heap_caps.h>
#endif

namespace meshcore {

ContactStore::ContactStore(size_t capacity)
    : contacts_(nullptr), capacity_(capacity), count_(0) {}

ContactStore::~ContactStore() {
    if (contacts_) {
#if defined(ESP_PLATFORM) || defined(ARDUINO)
        heap_caps_free(contacts_);
#else
        free(contacts_);
#endif
        contacts_ = nullptr;
    }
}

bool ContactStore::begin() {
    if (contacts_) return true;
    if (capacity_ == 0) return false;

    const size_t bytes = capacity_ * sizeof(Contact);
#if defined(ESP_PLATFORM) || defined(ARDUINO)
    contacts_ = static_cast<Contact*>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM));
    if (!contacts_) {  // no PSRAM / exhausted — fall back to internal RAM
        contacts_ = static_cast<Contact*>(heap_caps_malloc(bytes, MALLOC_CAP_8BIT));
    }
#else
    contacts_ = static_cast<Contact*>(malloc(bytes));
#endif
    if (!contacts_) return false;

    memset(contacts_, 0, bytes);
    count_ = 0;
    return true;
}

int ContactStore::upsert(const uint8_t pubKey[32], const char* name, uint32_t lastSeen) {
    if (!contacts_ || pubKey == nullptr) return -1;

    for (size_t i = 0; i < count_; ++i) {
        if (memcmp(contacts_[i].pubKey, pubKey, 32) == 0) {
            contacts_[i].lastSeen = lastSeen;
            if (name) {
                strncpy(contacts_[i].name, name, sizeof(contacts_[i].name) - 1);
                contacts_[i].name[sizeof(contacts_[i].name) - 1] = '\0';
            }
            return static_cast<int>(i);
        }
    }

    if (count_ >= capacity_) return -1;

    Contact& c = contacts_[count_];
    memcpy(c.pubKey, pubKey, 32);
    c.secretValid = false;
    c.lastSeen    = lastSeen;
    c.name[0]     = '\0';
    if (name) {
        strncpy(c.name, name, sizeof(c.name) - 1);
        c.name[sizeof(c.name) - 1] = '\0';
    }
    return static_cast<int>(count_++);
}

Contact* ContactStore::at(size_t idx) {
    return (contacts_ && idx < count_) ? &contacts_[idx] : nullptr;
}

const Contact* ContactStore::at(size_t idx) const {
    return (contacts_ && idx < count_) ? &contacts_[idx] : nullptr;
}

const uint8_t* ContactStore::sharedSecret(size_t idx, const MeshIdentity& self) {
    if (!contacts_ || idx >= count_) return nullptr;
    Contact& c = contacts_[idx];
    if (!c.secretValid) {
        self.calcSharedSecret(c.sharedSecret, c.pubKey);
        c.secretValid = true;
    }
    return c.sharedSecret;
}

int ContactStore::nextByHash(uint8_t hash, size_t* cursor) const {
    if (!contacts_ || cursor == nullptr) return -1;
    for (size_t i = *cursor; i < count_; ++i) {
        if (contacts_[i].pubKey[0] == hash) {
            *cursor = i + 1;
            return static_cast<int>(i);
        }
    }
    *cursor = count_;
    return -1;
}

}  // namespace meshcore
