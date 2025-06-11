#ifndef OPTIONAL_H
#define OPTIONAL_H

#include <type_traits>
#include <utility>

namespace DT {

template<typename T>
class Optional {
private:
    alignas(T) char storage[sizeof(T)];
    bool has_val = false;

    T* ptr() { return reinterpret_cast<T*>(storage); }
    const T* ptr() const { return reinterpret_cast<const T*>(storage); }

public:
    // Default constructor - empty optional
    Optional() : has_val(false) {}
    
    // Constructor from nullopt_t - make it implicit for conversion
    Optional(const nullptr_t&) : has_val(false) {}
    
    // Constructor from value
    Optional(const T& value) : has_val(true) {
        new (storage) T(value);
    }
    
    Optional(T&& value) : has_val(true) {
        new (storage) T(std::move(value));
    }
    
    // Copy constructor
    Optional(const Optional& other) : has_val(other.has_val) {
        if (has_val) {
            new (storage) T(*other.ptr());
        }
    }
    
    // Move constructor
    Optional(Optional&& other) : has_val(other.has_val) {
        if (has_val) {
            new (storage) T(std::move(*other.ptr()));
            other.reset();
        }
    }
    
    // Assignment operators
    Optional& operator=(const Optional& other) {
        if (this != &other) {
            reset();
            if (other.has_val) {
                new (storage) T(*other.ptr());
                has_val = true;
            }
        }
        return *this;
    }
    
    Optional& operator=(Optional&& other) {
        if (this != &other) {
            reset();
            if (other.has_val) {
                new (storage) T(std::move(*other.ptr()));
                has_val = true;
                other.reset();
            }
        }
        return *this;
    }
    
    Optional& operator=(const T& value) {
        reset();
        new (storage) T(value);
        has_val = true;
        return *this;
    }
    
    Optional& operator=(T&& value) {
        reset();
        new (storage) T(std::move(value));
        has_val = true;
        return *this;
    }
    
    Optional& operator=(const nullptr_t&) {
        reset();
        return *this;
    }
    
    // Destructor
    ~Optional() {
        reset();
    }
    
    // Check if has value
    bool has_value() const { return has_val; }
    explicit operator bool() const { return has_val; }
    
    // Access value (undefined behavior if empty)
    T& operator*() { return *ptr(); }
    const T& operator*() const { return *ptr(); }
    
    T* operator->() { return ptr(); }
    const T* operator->() const { return ptr(); }
    
    // Access value with exception (throws if empty)
    T& value() {
        if (!has_val) {
            // For Arduino/embedded, you might want to handle this differently
            // throw std::runtime_error("Optional has no value");
            // For now, we'll use undefined behavior like operator*
        }
        return *ptr();
    }
    
    const T& value() const {
        if (!has_val) {
            // throw std::runtime_error("Optional has no value");
        }
        return *ptr();
    }
    
    // Get value or default
    template<typename U>
    T value_or(U&& default_value) const {
        return has_val ? *ptr() : static_cast<T>(std::forward<U>(default_value));
    }
    
    // Reset to empty state
    void reset() {
        if (has_val) {
            ptr()->~T();
            has_val = false;
        }
    }
    
    // Emplace new value
    template<typename... Args>
    T& emplace(Args&&... args) {
        reset();
        new (storage) T(std::forward<Args>(args)...);
        has_val = true;
        return *ptr();
    }
};

// Simplified nullopt implementation
template<typename T>
Optional<T> make_nullopt() {
    return Optional<T>{};
}

// Or just use a simple struct that works better with templates
struct nullopt_t {
    explicit constexpr nullopt_t() = default;
};

constexpr nullopt_t nullopt{};

} // namespace DT

#endif