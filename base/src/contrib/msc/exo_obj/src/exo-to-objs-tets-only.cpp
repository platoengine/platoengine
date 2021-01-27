#include <array>
#include <cassert>
#include <iostream>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "msc/canary_buffer.h"
#include "msc/exodus_file_publishable.h" //odusIIFile64In, double_size
#include "msc/exodus_splitting_helpers.h"
#include "msc/utils_publishable.h"

namespace msc {

using std::array;
using std::vector;

void write_as_obj_recompact_vertices(
    std::ostream & out
  , vector<array<double, 3>> const& all_vertices
  , vector<array<size_t, 3>> const& indexed_triangles_subset
) {
  auto const n_vertices = size(all_vertices);

  auto subset_to_full_inds = reserve_vec<size_t>(3*size(indexed_triangles_subset));
  for(auto const& triangle : indexed_triangles_subset){
    for(auto const& into_full : triangle){
      assert(into_full < n_vertices);
      subset_to_full_inds.push_back(into_full);
    }
  }
  sort_unique(subset_to_full_inds);

  auto const n_subset_vertices = size(subset_to_full_inds);

  auto constexpr invalid_ind = size_t(-1);

  auto full_to_subset_inds = all_vec(n_vertices, invalid_ind);
  for(auto into_subset : ind::ices_of(subset_to_full_inds)){
    full_to_subset_inds.at(subset_to_full_inds.at(into_subset)) = into_subset;
  }

  auto constexpr delim = ' ';
  for(auto into_full : subset_to_full_inds){
    out << "v "; 
    intercalate(out, all_vertices.at(into_full), delim); 
    out << '\n';
  }

  for(auto const& triangle : indexed_triangles_subset){
    auto const reindexed = map(triangle, [&](auto index){
      auto const vertex_ind = full_to_subset_inds.at(index);
      assert(vertex_ind != invalid_ind);
      assert(vertex_ind < n_subset_vertices);
      return vertex_ind + 1; //OBJ files are 1-indexed
    });
    out << "f ";
    intercalate(out, reindexed, delim);
    out << '\n';
  }
}

/** `at` does bounds checking, so collecting arguments like this avoids some possible UB. */
inline
auto get_args(int argc, char const* const* argv) -> std::vector<std::string> {
  return std::vector<std::string>(argv, argv + argc);
}

}//namespace msc

int main(int argc, char * * argv) {
  using namespace msc;

  auto const args = get_args(argc, argv);

  if (size(args) != 3) { 
    cat_prln(
      "tets-only-exo-to-objs takes 2 arguments "
      "(path to input exodus file and prefix to which output obj paths will be appended); ",
      argc - 1," provided"
    );
    return 1;
  }
  
  auto const exo_file = exo::odusIIFile64In::open(args.at(1));

  auto const out_prefix = std::string{args.at(2)};

  auto const vertex_positions = get_vertices_assume_3d(exo_file);

  auto const blocks = get_tet_blocks(exo_file);

  auto write_obj_ = [&](auto const& path_infix, auto const& label, auto const& triangles){
    auto outname = cat_str(out_prefix,'/',path_infix,".obj");
    auto out = file_out(cat_str(out_prefix,'/',path_infix,".obj"));
    write_as_obj_recompact_vertices(out, vertex_positions, triangles);
    std::cout << label << outname << std::endl;
  };

  for(auto const& block : blocks){
    auto const& name = block.name;
    decltype(name)  const label = "block:";
    write_obj_(name, label, exo::surface_of_tets(block.tets));
  }

  auto const all_tets = cat_tet_blocks(blocks);

  for(auto const& node_set : get_node_sets(exo_file)){
    auto const& name = node_set.name;
    decltype(name) const label = "nodeset:";
    write_obj_(name, label, exo::surface_of_nodes_in_tets(node_set.node_inds, all_tets));
  }

  for(auto const& side_set : get_side_sets(exo_file)){
    auto const& name = side_set.name;
    decltype(name)  const label = "sideset:";
    write_obj_(name, label, exo::surface_of_sides_in_tets(side_set.sides, all_tets));
  }
}
