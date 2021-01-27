#ifndef MSC_EXODUS_SPLITTING_HELPERS_H
#define MSC_EXODUS_SPLITTING_HELPERS_H

#include "msc/exodus_file_publishable.h"
#include "msc/utils_publishable.h"

namespace msc {

namespace exo {

using std::vector;
using std::array;
using std::pair;
using std::string;
using std::get;

/** This mask describes the winding order which Exodus expects out of tetrahedra.
 *  The order in which the four face-masks are presented corresponds to Exodus's face-numbering,
 *  except that of course this is an array which is 0-indexed, while Exodus's uses 1-indexing
 *  (at least, when it isn't using arrays). */
constexpr
auto tet_side_set_to_node_inds
  = array<array<size_t, 3>, 4>{{ {0, 1, 3}, {1, 2, 3}, {0, 3, 2}, {0, 2, 1} }};

/** This applies the above mask to a tetrahedron. */
inline
auto faces_of_tet(array<size_t, 4> const& exo_tet) -> array<array<size_t, 3>, 4> {
  return mk_arr_cx<4>([&](auto face_n){ return mk_arr_cx<3>([&](auto vertex_n){
    return get<get<vertex_n.value>(get<face_n.value>(tet_side_set_to_node_inds))>(exo_tet);
  }); });
}

/** Applies the mask for only a single face. */
inline
auto face_of_tet(array<size_t, 4> const& exo_tet, size_t face_ind) -> std::array<size_t, 3> {
  auto const& mask = tet_side_set_to_node_inds.at(face_ind);
  //NOTE: If `faces_of_tet` can compile, then I think that implies that the below is safe?
  return mk_arr_cx<3>([&](auto i_){ return exo_tet[mask[i_]]; });
}

namespace detail {
  template<class Pred> //Requires: Pred(size_t)->bool
  auto surface_of_tets_base(vector<array<size_t, 4>> const& exo_tets, Pred&& node_ind_is_valid)
  -> vector<array<size_t, 3>> {
    auto face_keys = reserve_vec<std::pair<array<size_t, 3>, size_t>>(4*size(exo_tets));
    auto faces_of_partial_elems = vector<array<size_t, 3>>{}; //no reserve estimate here
    for(auto const& i_ : ind::ices_of(exo_tets)){
      auto const& tet = exo_tets.at(i_);

      auto const mismatch_pseudo_count = [&]{
        auto mismatch_count = size_t{0};
        for(auto ind : tet){
          if (!node_ind_is_valid(ind)) { ++mismatch_count; }
          if (mismatch_count > 1) { return mismatch_count; }
        }
        return mismatch_count;
      }();

      auto faces = faces_of_tet(tet);

      if (mismatch_pseudo_count == 0) { //if this entire tet is in the set,
        //then include this tet in trying to find faces which lie on the model's surface
        for(auto j_ : ind::ices_of(faces)){
          auto & face = faces.at(j_);
          std::sort(begin(face), end(face));
          face_keys.emplace_back(face, 4*i_ + j_);
        }
      } else if (mismatch_pseudo_count == 1) { //if 3 nodes in this tet are in the set,
        //then that means that it has precisely one face in the set,
        //which we'll include in the output no matter what
        for(auto const& face : faces){
          if (all(face, node_ind_is_valid)) { faces_of_partial_elems.push_back(face); }
        }
      } //otherwise, if there are 0, 1, or 2 nodes in this tet in the elset,
      //then that's too bad because they can't visualized in a polygonal mesh
      //so we don't need a final else
    }
    std::sort(begin(face_keys), end(face_keys), by_first(std::less<>{}));

    auto out = vector<array<size_t, 3>>{};

    visit_adjacently_nonequal_elements(face_keys, by_first(std::equal_to<>{}), [&](
        auto const& unique
    ){
      auto const face_ind = unique.second; //TODO: extract only the one needed face
      out.push_back(face_of_tet(exo_tets.at(face_ind/4), face_ind%4));
    });

    //TODO: deal with the case where `faces_of_partial_elems` contains any interior triangles,
    //      in which case it will contain two redundant chiral opposite copies of the same triangle
    insert_to(out, faces_of_partial_elems);

    return out;
  }
}//namespace detail

/** Decomposes a tetrahedral mesh into faces (assuming the Exodus file winding order),
 *  and returns faces which appear in precisely one tetrahedron. */
inline
auto surface_of_tets(vector<array<size_t, 4>> const& exo_tets) -> vector<array<size_t, 3>> {
  return detail::surface_of_tets_base(exo_tets, [](auto){ return true; });
}

/** Combines the results of `exo::get_tet_blocks` into a single array,
 *  which the two below functions are going to need. */
inline
auto cat_tet_blocks(vector<TetBlock> const& blocks)
-> vector<array<size_t, 4>> {
  auto out = reserve_vec<array<size_t, 4>>(sum(blocks, [](auto const& block){
    return size(block.tets);
  }));
  for(auto const& block : blocks){ insert_to(out, block.tets); }
  return out;
}

/** This one's a doozy (although the bad parts are in `surface_of_tets_base`),
 *  just because node sets are not fundamentally easy to display.
 *
 *  For a tetrahedron in `exo_tets`:
 *
 *  If all 4 nodes are in `node_inds`,
 *  then that tet is sent to the usual tets surface extraction algorithm,
 *  and the triangles of that surface is included in the output.
 *  
 *  If 3 nodes are in `node_inds`, then those nodes are included in the output as a triangle.
 *
 *  The problem here is that any triangles formed by nodes interior to the model will appear twice,
 *  once for each of the two tetrahedra which share it, so a big TODO:
 *  check final output for redundant pairs of triangles. */
inline
auto surface_of_nodes_in_tets(vector<size_t> node_inds, vector<array<size_t, 4>> const& exo_tets)
-> vector<array<size_t, 3>> {
  sort_unique(node_inds); //now `node_inds` is a sorted set
  return detail::surface_of_tets_base(exo_tets, [&](auto node_ind){
    return sorted_unique_range_contains(node_inds, node_ind);
  });
}

/** Extracts the faces specified in a side set. */
inline
auto surface_of_sides_in_tets(vector<Side> const& sides, vector<array<size_t, 4>> const& exo_tets)
-> vector<array<size_t, 3>> {
  return to_vec(sides, [&](auto const& side){
    return face_of_tet(exo_tets.at(side.tet_ind), side.face_ind);
  });
}

}//namespace exo

}//namespace msc

#endif//MSC_EXODUS_SPLITTING_HELPERS_H
