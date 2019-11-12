

#include <custom_octomap/OcTreeKey.h>

namespace custom_octomap {


    /***********************************************************************************************************
     * Constructor
     **********************************************************************************************************/
    OcTreeKey::OcTreeKey() {}

    OcTreeKey::OcTreeKey(key_type a, key_type b, key_type c) {
        k[0] = a;
        k[1] = b;
        k[2] = c;
    }

    OcTreeKey::OcTreeKey(const OcTreeKey &other) {
        k[0] = other.k[0];
        k[1] = other.k[1];
        k[2] = other.k[2];
    }

    /***********************************************************************************************************
     * Operator
     **********************************************************************************************************/
    bool OcTreeKey::operator==(const OcTreeKey &other) const {
        return ((k[0] == other[0]) && (k[1] == other[1]) && (k[2] == other[2]));
    }

    bool OcTreeKey::operator!=(const OcTreeKey &other) const {
        return ((k[0] != other[0]) || (k[1] != other[1]) || (k[2] != other[2]));
    }

    OcTreeKey &OcTreeKey::operator=(const OcTreeKey &other) {
        k[0] = other.k[0];
        k[1] = other.k[1];
        k[2] = other.k[2];
        return *this;
    }

    const key_type &OcTreeKey::operator[](unsigned int i) const {
        return k[i];
    }

    key_type &OcTreeKey::operator[](unsigned int i) {
        return k[i];
    }

    size_t OcTreeKey::KeyHash::operator()(const OcTreeKey &key) const {
        // a simple hashing function
        // explicit casts to size_t to operate on the complete range
        // constanst will be promoted according to C++ standard
        return static_cast<size_t>(key.k[0])
               + 1447 * static_cast<size_t>(key.k[1])
               + 345637 * static_cast<size_t>(key.k[2]);
    }

    /***********************************************************************************************************
     * Child
     **********************************************************************************************************/
    void OcTreeKey::computeChildKey(unsigned int pos, key_type center_offset_key, const OcTreeKey &parent_key, OcTreeKey &child_key) {
        // x-axis
        if (pos & 1) {
            child_key[0] = parent_key[0] + center_offset_key;
        } else {
            child_key[0] = parent_key[0] - center_offset_key - (center_offset_key ? 0 : 1);
        }
        // y-axis
        if (pos & 2) {
            child_key[1] = parent_key[1] + center_offset_key;
        } else {
            child_key[1] = parent_key[1] - center_offset_key - (center_offset_key ? 0 : 1);
        }
        // z-axis
        if (pos & 4) {
            child_key[2] = parent_key[2] + center_offset_key;
        } else {
            child_key[2] = parent_key[2] - center_offset_key - (center_offset_key ? 0 : 1);
        }
    }

    uint8_t OcTreeKey::computeChildIndex(const OcTreeKey &key, int depth) {
        uint8_t pos = 0;
        if (key.k[0] & (1 << depth)) {
            pos += 1;
        }

        if (key.k[1] & (1 << depth)) {
            pos += 2;
        }

        if (key.k[2] & (1 << depth)) {
            pos += 4;
        }

        return pos;
    }

    OcTreeKey OcTreeKey::computeIndexKey(key_type level, const OcTreeKey &key) {
        if (level == 0) {
            return key;
        } else {
            key_type mask = 65535 << level;
            OcTreeKey result = key;
            result[0] &= mask;
            result[1] &= mask;
            result[2] &= mask;
            return result;
        }

    }

    KeyRay::KeyRay() {
        ray.resize(maxSize);
        reset();
    }

    KeyRay::KeyRay(const KeyRay &other) {
        ray = other.ray;
        size_t dSize = other.end() - other.begin();
        end_of_ray = ray.begin() + dSize;
    }

    void KeyRay::reset() {
        end_of_ray = begin();
    }

    void KeyRay::addKey(const OcTreeKey &k) {
        assert(end_of_ray != ray.end());
        *end_of_ray = k;
        ++end_of_ray;
    }

    size_t KeyRay::size() const {
        return end_of_ray - ray.begin();
    }

    size_t KeyRay::sizeMax() const {
        return maxSize;
    }

    KeyRay::iterator KeyRay::begin() {
        return ray.begin();
    }

    KeyRay::iterator KeyRay::end() {
        return end_of_ray;
    }

    KeyRay::const_iterator KeyRay::begin() const {
        return ray.begin();
    }

    KeyRay::const_iterator KeyRay::end() const {
        return end_of_ray;
    }

    KeyRay::reverse_iterator KeyRay::rbegin() {
        return (reverse_iterator) end_of_ray;
    }

    KeyRay::reverse_iterator KeyRay::rend() {
        return ray.rend();
    }

}
