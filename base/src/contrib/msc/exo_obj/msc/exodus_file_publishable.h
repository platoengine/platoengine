#ifndef MSC_EXODUS_FILE_PUBLISHABLE
#define MSC_EXODUS_FILE_PUBLISHABLE

#include <array>
#include <cassert>
#include <limits>
#include <random>
#include <sstream>

#include "exodusII.h" //ex_*

#include "msc/canary_buffer.h" //canary_buffer
#include "msc/utils_publishable.h" //cat_str, mk_arr_cx

namespace msc {

namespace exo {

using std::vector;
using std::array;
using std::pair;
using std::make_pair;
using std::string;
using std::move;
using std::get;
using std::logic_error;

constexpr
auto double_size = 8;

struct FileOpenErr : public logic_error {
  int code;
  std::string path;
  FileOpenErr(std::string const& path_in, int code_in) 
  : logic_error(cat_str(
      "SEACAS was unable to open an exodus II file at path "
    , path_in," and instead returned error code ",code_in
    ))
  , code(code_in)
  , path(path_in)
  { }
};

struct IOErr : public logic_error {
  int code;
  IOErr(int code_in)
  : logic_error(cat_str("An operation in SEACAS failed with error code ",code_in))
  , code(code_in)
  { }
};

inline
void io_throw_if(int error_code) { 
  if (error_code != 0 && error_code != 1) { throw IOErr{error_code}; } 
}

class odusIIFile64In {
public:
  static auto open(std::string const& path) -> odusIIFile64In {
    auto mb = Metadata{}; //`mb` stands for "metadata buffer"

    int cpu_word_size_buffer = double_size;
    int io_word_size_buffer = double_size;
    float version_buffer = std::numeric_limits<float>::quiet_NaN();
    auto handle = ex_open(
      path.data(), settings, &cpu_word_size_buffer, &io_word_size_buffer, &version_buffer
    );
    if (handle < 0) { throw FileOpenErr{path, handle}; }
    assert(cpu_word_size_buffer == double_size);
    assert(io_word_size_buffer == double_size);

    //auto title_buffer = std::array<char, MAX_LINE_LENGTH>{};
    std::random_device rng{};
    auto title_buffer = EndCanaryBuffer<char>{MAX_LINE_LENGTH + 1, rng};
    for(auto c_ : title_buffer){ assert(c_ == '\0'); } //just making sure I'm doing initialization right

    io_throw_if(ex_get_init(
      handle
    , title_buffer.data()
    , &mb.n_dims
    , &mb.n_nodes
    , &mb.n_elems
    , &mb.n_elem_blocks
    , &mb.n_node_sets
    , &mb.n_side_sets
    ));

    title_buffer.check_canary();

    return odusIIFile64In{handle, std::string(title_buffer.data()), mb};
  }

  auto view_exoid() const -> int const& { return exoid; }

  struct Metadata {
    //static constexpr auto init = -50;
    static constexpr auto init = 0;
    //using Num = int;
    using Num = int64_t;
    Num n_dims = init;
    Num n_nodes = init;
    Num n_elems = init;
    Num n_elem_blocks = init;
    Num n_node_sets = init;
    Num n_side_sets = init;
  };

  auto view_metadata() const -> Metadata const& { return m_; }

  auto view_file_title() const -> std::string const& { return file_title; }

  //auto get_file_io_word_size() -> int { return m_.io_word_size; }

  ~odusIIFile64In() { ex_close(exoid); }

private:
  static constexpr auto settings = EX_READ;

  int exoid = -10;

  std::string file_title;
  Metadata m_;

  explicit odusIIFile64In(int handle_in, std::string title_in, Metadata const& meta_in)
  : exoid(handle_in), file_title(std::move(title_in)), m_(meta_in)
  { }
};

inline
auto get_vertices_assume_3d(odusIIFile64In const& in) -> vector<array<double, 3>> {
  auto const& exoid = in.view_exoid();
  auto const& m_ = in.view_metadata();

  assert(m_.n_dims == 3);

  std::random_device rng{}; //Huh, this is a type where CPP2017 'guaranteed copy elision' matters
  auto buffers = mk_arr_cx<3>([&](auto){
    return EndCanaryBuffer<double>{size_t(m_.n_nodes), rng};
  });
#if MSC_CPP_2017_MODE
  auto & [xs, ys, zs] = buffers;
#else
  auto & xs = get<0>(buffers);
  auto & ys = get<1>(buffers);
  auto & zs = get<2>(buffers);
#endif
  io_throw_if(ex_get_coord(exoid, xs.data(), ys.data(), zs.data()));

  for(auto const& buffer : buffers){ buffer.check_canary(); }

  return mk_vec(m_.n_nodes, [&](auto vertex_ind){
    return mk_arr_cx<3>([&](auto axis){ return get<axis>(buffers).at(vertex_ind); });
  });
}

namespace detail {
  inline
  auto exo_id_to_ind(int64_t id) -> size_t { return unsign_64(id - 1); }

  //NOTE: this returns the result of `on_buffer`
  template<class RNG, class F>
  auto use_ints_buffer(RNG & rng, size_t n_elements, bool elements_are_64_bit, F&& on_buffer) {
    auto mk_use_buffer = [&](auto filler){
      auto buffer = EndCanaryBuffer<decltype(filler)>{n_elements, rng, filler};
      auto out = on_buffer(buffer);
      buffer.check_canary();
      return out;
    };
    return elements_are_64_bit ? mk_use_buffer(int64_t{-1}) : mk_use_buffer(int32_t{-1});
  }

  //In contrast, this examines the state of the buffers, and then returns a separate value,
  //ignoring `on_ptrs_into_buffers`'s result.
  template<size_t ExpectedStrSize, class RNG, class F>
  auto use_str_buffer_ptrs(RNG & rng, size_t n_strs, F&& on_ptrs_into_buffers) -> vector<string> {
    auto buffers = EndCanaryBuffer<array<char, ExpectedStrSize + 1>>{n_strs, rng};

    auto ptrs = EndCanaryBuffer<char *>{n_strs + 1, rng};
    //TODO: add a map function for `EndCanaryBuffer`
    auto const b_ = begin(buffers);
    for(auto i_ : ind::ices_of(ptrs)){ ptrs.at(i_) = (b_ + i_)->data(); }
    //If `n_strs` is too low, then for many Exodus API functions,
    //data will be written to `&ptrs.at(n_strs)`, which points to the canary of `buffers`.
    //
    //Thus, `!buffers.canary_is_alive()` would imply that `n_strs` was too low.

    //Since everything in the Exodus API uses void pointers,
    //there's a chance that `on_ptrs_into_buffers` will overwrite `ptrs`,
    //if say `get_set_ids` is called instead of `get_set_names`.
    auto const initial_ptrs = ptrs;

    on_ptrs_into_buffers(ptrs);

    ptrs.check_canary();
    assert(ptrs == initial_ptrs); //Here we check for an accidental overwrite of `ptrs`.
    buffers.check_canary();
    for(auto const& buffer : buffers){ assert(get<ExpectedStrSize>(buffer) == '\0'); }

    return mk_vec(n_strs, [&](auto i_){ return string{buffers.at(i_).data()}; });
  }

  inline
  auto insert_default_names(string const& default_name_prefix, vector<string> names_or_empties)
  -> vector<string> {
    for(auto i_ : ind::ices_of(names_or_empties)){
      auto & name_or_empty = names_or_empties.at(i_);
      if (!size(name_or_empty)) { name_or_empty = cat_str(default_name_prefix,i_); }
    }
    return names_or_empties;
  }

  //If you need to figure out whether to use `EX_BULK_INT64_API` veruss `EX_IDS_INT64_API`,
  //then whatever you do, don't follow the examples in the Exodus documentation.
  //(There is of course no actionable, specific information in the rest of the documentation)
  //
  //The only way to get the correct values is by examining the Exodus source code,
  //since it uses these enums internally.
  inline
  auto bulk_ints_are_int64(odusIIFile64In const& in) -> bool {
    return bool(ex_int64_status(in.view_exoid()) & EX_BULK_INT64_API);
  }
  inline
  auto ids_are_int64(odusIIFile64In const& in) -> bool {
    return bool(ex_int64_status(in.view_exoid()) & EX_IDS_INT64_API);
  }

  template<class RNG>
  auto uncorrelated_get_ids(
      odusIIFile64In const& in, size_t n_ids, ex_entity_type const& exodus_enum, RNG & rng
  ) -> vector<int64_t> {
    return use_ints_buffer(rng, n_ids, ids_are_int64(in), [&](auto & buffer){
      io_throw_if(ex_get_ids(in.view_exoid(), exodus_enum, buffer.data()));
      buffer.check_canary();
      //TODO: also check that the buffer has been filled to the end
      return mk_vec(n_ids, [&](auto i_){ return int64_t{buffer.at(i_)}; });
    });
  }

  template<class RNG>
  auto get_block_ids(odusIIFile64In const& in, RNG & rng) -> vector<int64_t> {
    return uncorrelated_get_ids(in, in.view_metadata().n_elem_blocks, EX_ELEM_BLOCK, rng);
  }

  template<class RNG>
  auto get_block_names(odusIIFile64In const& in, RNG & rng) -> vector<string> {
    return use_str_buffer_ptrs<MAX_NAME_LENGTH>(rng, in.view_metadata().n_elem_blocks, [&](auto & ptrs){
      io_throw_if(ex_get_names(in.view_exoid(), EX_ELEM_BLOCK, ptrs.data()));
      ptrs.check_canary();
    });
  }

  inline
  auto get_set_size(odusIIFile64In const& in, int64_t set_id, ex_entity_type const& nodes_or_sides)
  -> size_t {
    auto constexpr init = int64_t{0};
    auto set_size = init;
    auto dummy = init;
    io_throw_if(ex_get_set_param(in.view_exoid(), nodes_or_sides, set_id, &set_size, &dummy));
    return set_size;
  }

  template<class RNG>
  auto get_set_names(
      odusIIFile64In const& in, size_t n_sets, ex_entity_type const& nodes_or_sides, RNG & rng
  ) -> vector<string> {
    return detail::use_str_buffer_ptrs<MAX_NAME_LENGTH>(rng, n_sets, [&](auto & ptrs){
      io_throw_if(ex_get_names(in.view_exoid(), nodes_or_sides, ptrs.data()));
      ptrs.check_canary();
    });
  }

  template<class RNG>
  auto get_set_ids(
      odusIIFile64In const& in, size_t n_sets, ex_entity_type const& nodes_or_sides, RNG & rng
  ) -> vector<int64_t> {
    return detail::use_ints_buffer(rng, n_sets, detail::ids_are_int64(in), [&](auto buffer){
      io_throw_if(ex_get_ids(in.view_exoid(), nodes_or_sides, buffer.data()));
      buffer.check_canary();

      return std::vector<int64_t>{begin(buffer), end(buffer)};
    });
  }

}//namespace detail

struct NodeSet {
  vector<size_t> node_inds;
  int64_t id;
  string name;
};

inline
auto get_node_sets(odusIIFile64In const& in) -> vector<NodeSet> {
  using namespace detail;

  std::random_device rng{};

  auto constexpr nset_enum = EX_NODE_SET;

  auto const n_sets = in.view_metadata().n_node_sets;

  auto const ids = get_set_ids(in, n_sets, nset_enum, rng);

  auto names = insert_default_names( "unnamed_node_set_"
  , get_set_names(in, n_sets, nset_enum, rng)
  ); //non-const to allow moving

  return mk_vec(n_sets, [&](auto i_){
    auto const id = ids.at(i_);

    auto const set_size = get_set_size(in, id, nset_enum);

    return NodeSet{
      use_ints_buffer(rng, set_size, bulk_ints_are_int64(in), [&](auto & buffer){
        io_throw_if(ex_get_set(in.view_exoid(), nset_enum, id, buffer.data(), nullptr));
        buffer.check_canary();
        return to_vec(buffer, [&](auto exo_node_ind){ return exo_id_to_ind(exo_node_ind); });
      })
    , id
    , move(names.at(i_))
    };
  });
}

struct Side { size_t tet_ind, face_ind; };

struct SideSet {
  vector<Side> sides;
  int64_t id;
  string name;
};

inline
auto get_side_sets(odusIIFile64In const& in) -> vector<SideSet> {
  using namespace detail;

  std::random_device rng{};

  auto constexpr side_set_enum = EX_SIDE_SET;

  auto const n_sets = in.view_metadata().n_side_sets;

  auto const ids = get_set_ids(in, n_sets, side_set_enum, rng);
  
  auto names = insert_default_names( "unnamed_side_set_"
  , get_set_names(in, n_sets, side_set_enum, rng)
  );

  return mk_vec(n_sets, [&](auto i_){
    auto const id = ids.at(i_);

    auto const size = get_set_size(in, id, side_set_enum);

    auto const blk_64 = bulk_ints_are_int64(in);

    return SideSet{ //I realize this looks super confusing,
      //but it's the simplest way to handle choosing between `int64_t` and `int32_t` buffers :(
      use_ints_buffer(rng, size, blk_64, [&](auto & tet_inds_buffer){
        return use_ints_buffer(rng, size, blk_64, [&](auto & side_inds_buffer){
          io_throw_if(ex_get_set(
            in.view_exoid(), side_set_enum, id, tet_inds_buffer.data(), side_inds_buffer.data()
          ));
          tet_inds_buffer.check_canary();
          side_inds_buffer.check_canary();

          return mk_vec(size, [&](auto j_){
            return Side{
              exo_id_to_ind(tet_inds_buffer.at(j_))
            , exo_id_to_ind(side_inds_buffer.at(j_))
            };
          });
        });
      })
    , id
    , move(names.at(i_))
    };
  });
}

struct TetBlock {
  vector<array<size_t, 4>> tets;
  int64_t id;
  string name;
};

inline
auto get_tet_blocks(odusIIFile64In const& in) -> vector<TetBlock> {
  using namespace detail;

  auto const exoid = in.view_exoid();

  std::random_device rng{};

  auto const bulk_int64_mode = bulk_ints_are_int64(in);

  auto const block_ids = get_block_ids(in, rng);

  auto block_names = insert_default_names("unnamed_block_", get_block_names(in, rng));

  auto ignore_buffer = EndCanaryBuffer<char>(MAX_STR_LENGTH + 1, rng);

  return mk_vec(size(block_ids), [&](auto i_){
    auto const block_id = block_ids.at(i_);

    auto constexpr init = int64_t{0};
    auto n_elems_in_block = init, n_vertices = init;
    auto ignore = init;
    io_throw_if(ex_get_block(
      exoid, EX_ELEM_BLOCK, block_id //inputs
    , ignore_buffer.data(), &n_elems_in_block, &n_vertices, &ignore, &ignore, &ignore //outputs
    ));
    ignore_buffer.check_canary();

    if (n_vertices != 4) {
      throw logic_error{cat_str(
        "Plato only accepts linear tetrahedral meshes.  Element detected with "
      , n_vertices," vertices."
      )};
    }

    //TODO: put this block on the outside to avoid re-allocating `buffer`
    //(the drawback of this is that a larger amount of code would have to be compiled twice)
    auto tets = use_ints_buffer(rng, n_elems_in_block*4, bulk_int64_mode, [&](auto buffer){
      io_throw_if(ex_get_conn(exoid, EX_ELEM_BLOCK, block_id, buffer.data(), nullptr, nullptr));
      buffer.check_canary();

      return mk_vec(n_elems_in_block, [&](auto elem_ind){
        return mk_arr_cx<4>([&](auto node_ind){
          return exo_id_to_ind(buffer.at(elem_ind*4 + node_ind));
        });
      });
    });

    return TetBlock{move(tets), block_id, move(block_names.at(i_))};
  });
}

}//namespace ex

}//namespace msc

#endif//MSC_EXODUS_FILE_PUBLISHABLE
