/*!
\ingroup PkgBGLConcepts
\cgalConcept

Concept from the Boost Graph Library.
See http://www.boost.org/libs/graph/doc/EdgeListGraph.html.

The concept `EdgeListGraph` refines the concept
<a href="http://www.boost.org/libs/graph/doc/Graph.html"><code>Graph</code></a>
and adds the requirement for traversal of all edges in a graph.

\cgalRefines <a href="http://www.boost.org/libs/graph/doc/Graph.html"><code>Graph</code></a>

\cgalHasModel See \link PkgBGLTraits Boost Graph Traits Specializations \endlink

\sa \link PkgBGLConcepts Graph Concepts \endlink
*/
class EdgeListGraph{};


/*! \relates EdgeListGraph
 * returns an iterator range over all edges.
 */
template <typename EdgeListGraph>
std::pair<boost::graph_traits<EdgeListGraph>::edge_iterator,
          boost::graph_traits<EdgeListGraph>::edge_iterator>
edges(const EdgeListGraph& g);


/*! \relates EdgeListGraph
  returns an upper bound of the number of edges of the graph.
  \attention `num_edges()` may return a number larger than `std::distance(edges(g).first, edges(g).second)`.
  This is the case for implementations only marking edges deleted in the edge container.
 */
template <typename EdgeListGraph>
boost::graph_traits<EdgeListGraph>::ver_size_type
num_edges(const EdgeListGraph& g);


/*! \relates EdgeListGraph
returns the source vertex of `h`.
 */
template <typename EdgeListGraph>
boost::graph_traits<EdgeListGraph>::vertex_descriptor
source(boost::graph_traits<EdgeListGraph>::halfedge_descriptor h, const EdgeListGraph& g);


/*! \relates EdgeListGraph
returns the target vertex of `h`.
 */
template <typename EdgeListGraph>
boost::graph_traits<EdgeListGraph>::vertex_descriptor
target(boost::graph_traits<EdgeListGraph>::halfedge_descriptor h, const EdgeListGraph& g);
