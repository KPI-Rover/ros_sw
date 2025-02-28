#pragma once
#include <mutex>

template<typename T>
class ValueCache {
public:
    ValueCache() : value(T{}), valid(false) {} // Initialize value then valid, matching declaration order below.
    
    void update(const T &newValue) {
        std::lock_guard<std::mutex> lock(mtx);
        value = newValue;
        valid = true;
    }

    T get() const {
        std::lock_guard<std::mutex> lock(mtx);
        return valid ? value : T{};
    }
    
    bool isValid() const {
        std::lock_guard<std::mutex> lock(mtx);
        return valid;
    }
    
    void set(const T &newValue, bool isValid) {
        std::lock_guard<std::mutex> lock(mtx);
        value = newValue;
        valid = isValid;
    }
private:
    T value;
    bool valid;
    mutable std::mutex mtx;
};
