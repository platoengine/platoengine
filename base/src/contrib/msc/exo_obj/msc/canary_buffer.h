#ifndef MSC_CANARY_BUFFER_H
#define MSC_CANARY_BUFFER_H

#include <cstring>
#include <memory>

#include "msc/utils_publishable.h"

namespace msc {

/** Many Exodus API functions write an undocumented number of bytes to `void *` inputs.
 *  If you allocate too short of an array, then Exodus will copy data past the end of your array,
 *  at worst infecting your computer with data from the Exodus file,
 *  and at best creating a nondeterministic bug.
 *
 *  `EndCanaryBuffer` sticks a number immediately after the end of your requested array,
 *  known as the "canary".  After letting an Exodus API function write to your array,
 *  you can check whether that canary number has been overwritten,
 *  making it obvious when there is a bug caused by a buffer overrun. */

namespace can_buf { //NOTE: There's a `using can_buf::EndCanaryBuffer;` at the end.

/** For documentation on the following 3 functions,
 *  just see the relevant wiki entries for their C++17 standard library versions. */

template<class T>
void uninitialized_value_construct_n(T * start, size_t n_) {
#if MSC_CPP_2017_MODE
  std::uninitialized_value_construct_n(start, n_);
#else
  for(auto i_ : ind::ices(n_)){ new(start + i_)T{}; }
#endif
}

template<class T>
void uninitialized_fill_n(T * start, size_t n_, T const& init_to) {
#if MSC_CPP_2017_MODE
  std::uninitialized_fill_n(start, n_, init_to);
#else
  for(auto i_ : ind::ices(n_)){ new(start + i_)T{init_to}; }
#endif
}

template<class T>
void destroy_n(T * start, size_t n_) {
#if MSC_CPP_2017_MODE
  std::destroy_n(start, n_);
#else
  for(auto i_ : ind::ices(n_)){ start[i_].~T(); }
#endif
}

template<class T>
class EndCanaryBuffer {
public:
  auto at(size_t i_) const -> T const& { return data()[bounds_check(i_)]; }
  auto at(size_t i_) -> T & { return data()[bounds_check(i_)]; }

  /** Use this whenever the Exodus API needs a `void *` parameter. */
  auto data() const -> T const* { return reinterpret_cast<T const*>(buffer); }
  auto data() -> T * { return reinterpret_cast<T *>(buffer); }

  auto begin() const -> T const* { return data(); }
  auto begin() -> T * { return data(); }
  auto end() const -> T const* { return begin() + ts_size; }
  auto end() -> T * { return begin() + ts_size; }

  auto size() const -> size_t { return ts_size; }

  EndCanaryBuffer() = default;

  using Canary = uint64_t;

  /** Creates an array of `size_in` elements equal to `init_to`,'
   *  and sets the canary value to `canary_in`.
   *
   *  NOTE: `size_t` and `uint64_t` are probably the same type on your system,
   *        which risks mixing them up. */
  template<class RNG>
  explicit EndCanaryBuffer(size_t size_in, RNG & rng, T const& init_to)
  : EndCanaryBuffer{Internal{}, size_in, rng}
  { std::uninitialized_fill_n(data(), ts_size, init_to);
  }

  /** Creates an array of `size_in` elements equal to `T{}`, and sets the canary to `canary_in`. */
  template<class RNG>
  explicit EndCanaryBuffer(size_t size_in, RNG & rng)
  : EndCanaryBuffer{Internal{}, size_in, rng}
  { uninitialized_value_construct_n(data(), ts_size);
  }

  /** If the canary has been overwritten, then that suggests the same for subsequent data,
   *  which would imply that the general state of memory has been corrupted.
   *
   *  That is an unrecoverable state, so this just crashes the program rather than throwing.
   *
   *  Use this after an Exodus API call. */
  void check_canary() const { assert(canary_is_alive()); }

  /** `check_canary` crashes immediately.  If you want to print a message before crashing,
   *  use this function to check for corruption first. */
  auto canary_is_alive() const -> bool {
    return !buffer //we're going to assume an empty buffer to be okay
      || !std::memcmp(&canary_, end(), sizeof(Canary)); //NOTE: `memcmp` returns 0 on equality :/
  }

  /** The move constructor. */
  EndCanaryBuffer(EndCanaryBuffer&& other) { *this = std::move(other); }

  /** The move assignment operator does not guarantee that the moved-from buffer is empty,
   *  as it merely swaps. */
  auto operator=(EndCanaryBuffer&& other) -> EndCanaryBuffer & {
    std::swap(buffer, other.buffer);
    std::swap(ts_size, other.ts_size);
    std::swap(canary_, other.canary_);
    return *this;
  }

  /** The copy constructor. */
  EndCanaryBuffer(EndCanaryBuffer const& other) { assume_no_buffer_initialize_from(other); }

  /** The copy assignment operator doesn't do any kind of buffer re-use a la `std::vector`. */
  auto operator=(EndCanaryBuffer const& other) -> EndCanaryBuffer & {
    delete_buffer();
    assume_no_buffer_initialize_from(other);
    return *this;
  }

  /** The destructor checks for corruption with `check_canary`, just in case you forgot.
   *  You should call it yourself as early as possible, however. */
  ~EndCanaryBuffer() {
    check_canary(); //You want to check as early as possible, but you might as well check now too.
    delete_buffer();
  }

  auto operator==(EndCanaryBuffer const& other) -> bool {
    return std::equal(begin(), end(), other.begin(), other.end());
  }
  
private:
  using Byte = unsigned char;

  //NOTE: The move constructor and assignment operator depend on these default values.
  Byte * buffer = nullptr;
  size_t ts_size = 0;
  Canary canary_ = 0;

  static auto full_buffer_size(size_t ts_size_in) -> size_t {
    return ts_size_in*sizeof(T) + sizeof(Canary);
  }

  static auto allocate(size_t ts_size_in) -> Byte * {
    return static_cast<Byte *>(operator new(full_buffer_size(ts_size_in)));
  }

  struct Internal { }; //just a tag to dispatch the below constructor

  template<class RNG>
  explicit EndCanaryBuffer(Internal, size_t size_in, RNG & rng)
  : buffer{allocate(size_in)}
  , ts_size{size_in}
  , canary_{std::uniform_int_distribution<Canary>{0, std::numeric_limits<Canary>::max()}(rng)}
  { std::memcpy(buffer + ts_size*sizeof(T), &canary_, sizeof(canary_));
    //Note that this does nothing to initialize data in [`begin()`, `end()`).
  }

  auto bounds_check(size_t ind) -> size_t {
    assert(ind < ts_size); //TODO: throw an actual error message here
    return ind;
  }

  void delete_buffer() {
    if (!buffer) { return; }
    destroy_n(begin(), ts_size);
    operator delete(buffer);
  }

  void assume_no_buffer_initialize_from(EndCanaryBuffer const& other) {
    auto const n_bytes = full_buffer_size(other.ts_size);
    buffer = allocate(n_bytes);
    std::uninitialized_copy_n(other.buffer, n_bytes, buffer);
    ts_size = other.ts_size;
    canary_ = other.canary_;
  }
};

#if MSC_CPP_2017_MODE
using std::begin, std::end, std::size;
#else
using std::begin;
using std::end;
//note: `exported_utils.h` has its own `size` definition :p
#endif

}//namespace can_buf

using can_buf::EndCanaryBuffer;

}//namespace msc

#endif//MSC_CANARY_BUFFER_H
