#ifndef DR_MACROS_HPP_
#define DR_MACROS_HPP_

#include <memory>
#include <utility>

// ============================================================================
// Utility Macro Definitions
// ============================================================================

/**
 * Defines aliases and static functions for using the Class with smart pointers.
 *
 * Use in the public section of the class.
 * Make sure to include `<memory>` in the header when using this.
 */
#define DR_SMART_PTR_DEFINITIONS(...) \
  DR_SHARED_PTR_DEFINITIONS(__VA_ARGS__) \
  DR_WEAK_PTR_DEFINITIONS(__VA_ARGS__) \
  DR_UNIQUE_PTR_DEFINITIONS(__VA_ARGS__)

/**
 * Defines aliases and static functions for using the Class with smart pointers.
 *
 * Same as DR_SMART_PTR_DEFINITIONS except it excludes the static
 * Class::make_unique() method definition which does not work on classes which
 * are not CopyConstructable.
 *
 * Use in the public section of the class.
 * Make sure to include `<memory>` in the header when using this.
 */
#define DR_SMART_PTR_DEFINITIONS_NOT_COPYABLE(...) \
  DR_SHARED_PTR_DEFINITIONS(__VA_ARGS__) \
  DR_WEAK_PTR_DEFINITIONS(__VA_ARGS__) \
  __DR_UNIQUE_PTR_ALIAS(__VA_ARGS__)

/**
 * Defines aliases only for using the Class with smart pointers.
 *
 * Same as DR_SMART_PTR_DEFINITIONS except it excludes the static
 * method definitions which do not work on pure virtual classes and classes
 * which are not CopyConstructable.
 *
 * Use in the public section of the class.
 * Make sure to include `<memory>` in the header when using this.
 */
#define DR_SMART_PTR_ALIASES_ONLY(...) \
  __DR_SHARED_PTR_ALIAS(__VA_ARGS__) \
  __DR_WEAK_PTR_ALIAS(__VA_ARGS__) \
  __DR_UNIQUE_PTR_ALIAS(__VA_ARGS__) \
  __DR_MAKE_SHARED_DEFINITION(__VA_ARGS__)

#define __DR_SHARED_PTR_ALIAS(...) \
  using SharedPtr = std::shared_ptr<__VA_ARGS__>; \
  using ConstSharedPtr = std::shared_ptr<const __VA_ARGS__>;

#define __DR_MAKE_SHARED_DEFINITION(...) \
  template<typename ... Args> \
  static std::shared_ptr<__VA_ARGS__> \
  make_shared(Args && ... args) \
  { \
    return std::make_shared<__VA_ARGS__>(std::forward<Args>(args) ...); \
  }

/// Defines aliases and static functions for using the Class with shared_ptrs.
#define DR_SHARED_PTR_DEFINITIONS(...) \
  __DR_SHARED_PTR_ALIAS(__VA_ARGS__) \
  __DR_MAKE_SHARED_DEFINITION(__VA_ARGS__)

#define __DR_WEAK_PTR_ALIAS(...) \
  using WeakPtr = std::weak_ptr<__VA_ARGS__>; \
  using ConstWeakPtr = std::weak_ptr<const __VA_ARGS__>;

/// Defines aliases and static functions for using the Class with weak_ptrs.
#define DR_WEAK_PTR_DEFINITIONS(...) __DR_WEAK_PTR_ALIAS(__VA_ARGS__)

#define __DR_UNIQUE_PTR_ALIAS(...) using UniquePtr = std::unique_ptr<__VA_ARGS__>;

#define __DR_MAKE_UNIQUE_DEFINITION(...) \
  template<typename ... Args> \
  static std::unique_ptr<__VA_ARGS__> \
  make_unique(Args && ... args) \
  { \
    return std::unique_ptr<__VA_ARGS__>(new __VA_ARGS__(std::forward<Args>(args) ...)); \
  }

/// Defines aliases and static functions for using the Class with unique_ptrs.
#define DR_UNIQUE_PTR_DEFINITIONS(...) \
  __DR_UNIQUE_PTR_ALIAS(__VA_ARGS__) \
  __DR_MAKE_UNIQUE_DEFINITION(__VA_ARGS__)

#define DR_STRING_JOIN(arg1, arg2) DR_DO_STRING_JOIN(arg1, arg2)
#define DR_DO_STRING_JOIN(arg1, arg2) arg1 ## arg2

// Safe pointer deletion
#define DR_SAFE_DELETE(p) { if(p) { delete (p); (p) = nullptr; } }
#define DR_SAFE_DELETE_ARRAY(p) { if(p) { delete[] (p); (p) = nullptr; } }


/**
 * Disables the copy constructor and operator= for the given class.
 *
 * Use in the private section of the class.
 */
#define DR_DISABLE_COPY(...) \
  __VA_ARGS__(const __VA_ARGS__ &) = delete; \
  __VA_ARGS__ & operator=(const __VA_ARGS__ &) = delete;

/**
 * Disables the move constructor and operator= for the given class.
 *
 * Use in the private section of the class.
 */
#define DR_DISABLE_MOVE(...) \
    __VA_ARGS__(__VA_ARGS__&&) = delete; \
    __VA_ARGS__& operator=(__VA_ARGS__&&) = delete;

/**
 * Disables the move and copy constructor and operator= for the given class.
 *
 * Use in the private section of the class.
 */
#define DR_DISABLE_COPY_AND_MOVE(...) \
    DISABLE_COPY(__VA_ARGS__) \
    DISABLE_MOVE(__VA_ARGS__)

  
// Logging macros
// #define DR_LOG_INFO(msg) ROS_INFO_STREAM(__FUNCTION__ << ": " << msg)
// #define DR_LOG_WARN(msg) ROS_WARN_STREAM(__FUNCTION__ << ": " << msg)
// #define DR_LOG_ERROR(msg) ROS_ERROR_STREAM(__FUNCTION__ << ": " << msg)


#endif  // DR_MACROS_HPP_