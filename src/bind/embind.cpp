#include <emscripten/bind.h>

using namespace emscripten;

namespace emscripten
{
  /**
   * @brief register_vector_plus. See emscripten/bind.h
   *
   * @tparam T
   * @tparam Allocator
   * @param name
   * @return class_<std::vector<T, Allocator>>
   */
  template <typename T, typename Allocator = std::allocator<T>>
  class_<std::vector<T, Allocator>> register_vector_plus(const char *name)
  {
    typedef std::vector<T, Allocator> VecType;

    void (VecType::*push_back)(const T &) = &VecType::push_back;
    void (VecType::*resize)(const size_t, const T &) = &VecType::resize;
    return class_<std::vector<T, Allocator>>(name)
        .template constructor<>()
        .function("push_back", push_back)
        .function("resize", resize)
        .function("size", &VecType::size)
        .function("capacity", &VecType::capacity)
        .function("max_size", &VecType::max_size)
        .function("clear", &VecType::clear)
        .function("empty", &VecType::empty)
        .function("get", &internal::VectorAccess<VecType>::get)
        .function("set", &internal::VectorAccess<VecType>::set);
  }
}
