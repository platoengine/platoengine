#ifndef MSC_EXPORTED_UTILS_H
#define MSC_EXPORTED_UTILS_H

#include <array>
#include <algorithm>
#include <cassert>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#define MSC_CPP_2017_MODE (__cplusplus == 201703L)

#define MSC_PRINT_VAR(EXPR) do {\
  cat_to(std::cout, #EXPR,": ",EXPR);\
  std::cout << std::endl;\
} while(0)

namespace msc {

/** This is just an alternate syntax to what `operator<<` provides. */
#if MSC_CPP_2017_MODE
  template<class Out, class ... Tokens>
  void cat_to(Out & out, Tokens const& ... tokens) { (out << ... << tokens); }
#else

  template<class Out>
  void cat_to(Out &) {  }

  template<class Out, class Token, class ... Tokens>
  void cat_to(Out & out, Token const& token, Tokens const& ... tokens) {
    out << token;
    cat_to(out, tokens ...);
  }
#endif

/** Prints a line to cout (e.g. the console) (also, synchronizes to cout) via `operator<<`. */
template<class ... Tokens>
void cat_prln(Tokens const& ... tokens) {
  cat_to(std::cout, tokens ...);
  std::cout << std::endl;
}

/** Prints concatenates `tokens` to a single string via `operator<<`. */
template<class ... Tokens>
auto cat_str(Tokens const& ... tokens) -> std::string {
  auto buffer = std::ostringstream{};
  cat_to(buffer, tokens ...);
  return std::move(buffer).str();
}

/** Opens a file handle at `path`.  Throws an exception if it can't. */
inline
auto file_out(std::string const& path) -> std::ofstream {
  auto out = std::ofstream{path, std::ios_base::out | std::ios_base::binary};
  //TODO: choose a better exception type
  if (!out) { throw std::logic_error{cat_str("Unable to open file for writing at path ",path)}; }
  return out;
}

/** This is just an alternate syntax to `.size()`. */
#if (!MSC_CPP_2017_MODE)
  template<class Range>
  auto size(Range const& range) -> size_t { return range.size(); }
#endif

/** Given some maximum number `maximum`, the following two lines are akin to each other:
 *  `for(size_t i_ = 0; i_ < maximum; ++i_)`
 *  `for(auto i_ : ind::ices(maximum))`
 *  Or, in Python:
 *  `for i_ in range(maximum):`
 *
 *  Given some range `range` (e.g. `auto range = std::vector{5, 6, 7, 8}`),
 *  `ind::ices(size(range))` is equivalent to `ind::ices_of(range)` */
namespace ind {
#if (MSC_CPP_2017_MODE)
  using std::size;
//else, we're using the `size` function above
#endif
  using std::begin;
  using std::end;

  template<class Num>
  struct ItSelf {
    Num contents;
    ItSelf(Num const& contents_in) : contents{contents_in} { }
    void operator++() { ++contents; }
    auto operator!=(ItSelf const& r_) const -> bool { return contents != r_.contents; }
    auto operator*() const -> Num const& { return contents; }

    auto operator-(ItSelf const& r_) const -> Num { return contents - r_.contents; }
  };
  template<class Num>
  auto mk_ItSelf(Num const& num) { return ItSelf<Num>{num}; }

  template<class B, class E = B>
  struct ItRange {
    B b_;
    E e_;
    explicit ItRange(B const& b_in, E const& e_in) : b_{b_in}, e_{e_in} { }
    auto begin() const -> B { return b_; }
    auto end() const -> E { return e_; }
    auto size() const -> size_t { return e_ - b_; }
  };
  template<class B, class E>
  auto mk_ItRange(B const& b_, E const& e_) { return ItRange<B, E>{b_, e_}; }

  //`ind::ices` is akin to Python's `range`
  //(also, note that "range" in C++ is akin to a Python generator)
  inline
  auto ices(size_t max) -> ItRange<ItSelf<size_t>> {
    return mk_ItRange(mk_ItSelf(size_t{0}), mk_ItSelf(max));
  }

  template<class Range>
  auto ices_of(Range const& range) -> ItRange<ItSelf<size_t>> { return ices(size(range)); }
}//namespace ind

/** e.g. `intercalate(std::cout, std::vector{'a', 'b', 'c'}, "!!!")` will print `a!!!b!!!c` */
template<class Out, class Range, class Delim>
void intercalate(Out & out, Range const& range, Delim const& delim) {
  auto it = begin(range);
  auto const e_ = end(range);
  if (it == e_) { return; }
  out << *it;
  ++it;
  for(; it != e_; ++it){ out << delim << *it; }
}

/** `mk_arr_cx<N>(f_)` is akin to
 *  `Table[f_, N]` in Mathematica (ignore that `N` is a Mathematica builtin) or
 *  `List(Map(f_, N))` in Python.
 *
 *  e.g. `mk_arr_cx<3>([&](auto i_){ return cat_str('y',i_); })` equals
 *       `std::array<std::string, 3>{"y0", "y1", "y2"}`
 *
 *  The advantage here is that a template function (e.g. a lambda with an `auto` input)
 *  will get compiled N times, each with a different index.
 *
 *  Specify a `size_t` input to ensure only single compilation of the lambda, e.g.
 *  `mk_arr_cx<3>([&](size_t i_){ return cat_str('y',i_); })` */
template<class F, size_t... Ixs> constexpr
auto make_array_constexpr_helper(F&& f_, std::index_sequence<Ixs...>) {
  return std::array<std::decay_t<
    decltype(f_(std::integral_constant<size_t, 0>{}))
  >, sizeof...(Ixs)>{f_(std::integral_constant<size_t, Ixs>{})...};
}
template<size_t N, class F> constexpr 
auto mk_arr_cx(F&& f_) {
  return make_array_constexpr_helper(f_, std::make_index_sequence<N>{});
}

/** Requires, for some T2: F(T const&)->T2 */
template<class T, size_t N, class F> constexpr
auto map(std::array<T, N> const& arr, F&& on_elem) {
  return mk_arr_cx<N>([&](auto i_){ return on_elem(std::get<i_>(arr)); });
}

/** This just saves a line of code, e.g. `auto out = reserve_vec<std::string>(50);` */
template<class T>
auto reserve_vec(size_t n_) -> std::vector<T> {
  auto out = std::vector<T>{};
  out.reserve(n_);
  return out;
}

/** `Range` should probably be `std::vector<T>` for some `T`.
 *
 *  Combined with `std::lower_bound`, this essentially set-ifies `range`.
 *
 *  Requires, for some T:
 *    Range is a range over T
 *    T const& == T const&
 *    T const& < T const& */
template<class Range>
void sort_unique(Range & range) {
  auto const b_ = begin(range);
  auto const e_ = end(range);
  std::sort(b_, e_);
  range.erase(std::unique(b_, e_), e_);
}

/** Requires:
 *    Range is a range over Elem
 *    Elem const& == Elem const&
 *    Elem const& < Elem const& */
template<class Range, class Elem>
auto sorted_unique_range_contains(Range const& range, Elem const& maybe_an_element) -> bool {
  auto const e_ = end(range);
  auto const find_attempt = std::lower_bound(begin(range), e_, maybe_an_element);
  return find_attempt != e_ && *find_attempt == maybe_an_element;
}

/** Maps a function over an arbitrary range of known size,
 *  but collects the results in a `std::vector`, rather than the original range type,
 *  akin to `List(map(on_elem, range))` in Python (ignoring that `range` is a Python builtin).
 *
 *  Note that because this takes a universal reference, `on_elem` is allowed to mutate `range`,
 *  which is important if you want to call move constructors.
 *
 *  Requires, for some Elem, Result:
 *    Range is a sized range over Elem //TODO: include psuedo-concept documentation
 *    F(Elem&&)->Result */
template<class Range, class F>
auto to_vec(Range&& range, F&& on_elem) {
  using std::begin;
  auto out = reserve_vec<std::decay_t<decltype(on_elem(*begin(range)))>>(size(range));
  for(auto&& elem : range){ out.push_back(on_elem(elem)); }
  return out;
}

/** Akin to `mk_arr_cx`, but without the compile-time indices. */
template<class F>
auto mk_vec(size_t n_, F&& on_ind) { return to_vec(ind::ices(n_), on_ind); }

/** Fills a standard vector of length `n_` with `tile`.
 *
 *  This is useful to avoid an accident such as `std::vector<int>{50, -1}`,
 *  which produces only a 2-element array (in contrast with `std::vector<int>(50, -1)`). */
template<class T>
auto all_vec(size_t n_, T const& tile) -> std::vector<T> { return std::vector<T>(n_, tile); }

/** Crashes if `n_` is negative, and also casts converts `int64_t` to `size_t`. */
inline
auto unsign_64(int64_t n_) -> size_t {
  assert(n_ >= 0);
  return size_t(n_);
}

/** e.g. `sum(std::array{1, -2, 3}, [&](auto num){ return std::pow(num, 2); })` equals `14`.
 *
 *  Requires, for some Elem, Result:
 *    Range is a range over Elem
 *    F(Elem const&)->Result
 *    Elem & += Elem const& */
template<class Range, class F>
auto sum(Range const& range, F&& on_elem) {
  using std::begin;
  auto out = std::decay_t<decltype(on_elem(*begin(range)))>{};
  for(auto const& elem : range){ out += on_elem(elem); }
  return out;
}

/** Calls `on_unique` on any element with no adjacent equal element
 *  (with equality determined by `eq`).
 *
 *  This function is far more useful if `range` is sorted,
 *  in which case it calls `on_unique` only on elements which appear precisely once.
 *
 *  Requires, for some T:
 *    Range is a range over T
 *    Eq(T const&, T const&)->bool
 *    F(T const&) */
template<class Range, class Eq, class F>
void visit_adjacently_nonequal_elements(Range const& range, Eq&& eq, F&& on_unique) {
  auto it = begin(range);
  auto const e_ = end(range);
  while(it != e_){
    auto next_ = std::next(it);
    while(next_ != e_ && eq(*it, *next_)){ ++next_; }
    if (next_ - it == 1) { on_unique(*it); }
    it = next_;
  }
}

template<class F> //Requires, for some L, R, Result: F(L const&, R const&)->Result
auto by_first(F bin_op_in) {
  using std::get;
  return [bin_op=std::move(bin_op_in)](auto const& l_, auto const& r_){
    return bin_op(get<0>(l_), get<0>(r_));
  };
}

/** Checks that `predicate` holds for for every element in `Range`.
 *
 *  Requires, for some T:
 *    Range is a range over T
 *    Pred(T const&)->bool */
template<class Range, class Pred>
auto all(Range const& range, Pred&& predicate) -> bool {
  for(auto const& elem : range){ if (!predicate(elem)) { return false; } }
  return true;
}

/** Concatenates `dest` with `src` by modifying `dest`.
 *  
 *  Requires:
 *    Range is a range over T */
template<class T, class Range>
auto insert_to(std::vector<T> & dest, Range const& src) {
  using std::begin;
  using std::end;
  dest.insert(end(dest), begin(src), end(src));
}

}//namespace msc

#endif//MSC_EXPORTED_UTILS_H
