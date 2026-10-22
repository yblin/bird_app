//
// Copyright 2016-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GRAPH_GRAPH_H_
#define CODELIBRARY_GRAPH_GRAPH_H_

#include <string>
#include <unordered_map>

#include "codelibrary/base/array.h"
#include "codelibrary/util/list/indexed_list.h"

namespace cl {

/**
 * Graph, represented by adjacency list.
 */ 
class Graph {
public:
    class BaseEdge;
    using Edge = IndexedList<BaseEdge>::Node;

    class BaseEdge {
        friend class Graph;
        friend class EdgeList;

    public:
        int source()       const { return source_; }
        int target()       const { return target_; }
        Edge* twin()             { return twin_;   }
        const Edge* twin() const { return twin_;   }

    private:
        int source_ = -1;      // Source vertex of this edge.
        int target_ = -1;      // Target vertex of this edge.
        Edge* twin_ = nullptr; // Twin edge of this edge.
        Edge* prev_ = nullptr; // For EdgeList.
        Edge* next_ = nullptr; // For EdgeList.
    };

    /**
     * Adjacency list of an incident vertex.
     */ 
    class EdgeList {
        friend class Graph;

    public:
        /**
         * Iterator for EdgeList.
         */ 
        class Iterator {
            using iterator_category = std::forward_iterator_tag;
            using value_type        = Edge*;
            using difference_type   = int;
            using pointer           = const value_type*;
            using reference         = const value_type&;

        public:
            Iterator() = default;

            explicit Iterator(Edge* edge) : edge_(edge) {}

            bool operator == (const Iterator& rhs) const {
                return edge_ == rhs.edge_;
            }

            bool operator != (const Iterator& rhs) const {
                return !(*this == rhs);
            }

            Edge* operator*()  const { return edge_; }
            Edge* operator->() const { return edge_; }

            Iterator& operator++() {
                if (edge_) edge_ = edge_->next_;
                return *this;
            }

            Iterator& operator--() {
                if (edge_) edge_ = edge_->prev_;
                return *this;
            }

        protected:
            Edge* edge_ = nullptr;
        };

        /**
         * Const iterator for EdgeList.
         */
        class ConstIterator {
        public:
            using iterator_category = std::forward_iterator_tag;
            using value_type        = const Edge*;
            using difference_type   = int;
            using pointer           = const value_type*;
            using reference         = const value_type&;

            ConstIterator() = default;
            explicit ConstIterator(Iterator i) : edge_(*i) {}
            explicit ConstIterator(const Edge* e) : edge_(e) {}

            bool operator == (const ConstIterator& rhs) const {
                return edge_ == rhs.edge_;
            }

            bool operator != (const ConstIterator& rhs) const {
                return !(*this == rhs);
            }

            const Edge* operator * () const { return edge_; }
            const Edge* operator ->() const { return edge_; }

            ConstIterator& operator++() {
                if (edge_) edge_ = edge_->next_;
                return *this;
            }

            ConstIterator& operator--() {
                if (edge_) edge_ = edge_->prev_;
                return *this;
            }

        protected:
            const Edge* edge_ = nullptr;
        };

        EdgeList() = default;

        Iterator begin() {
            return Iterator(first_);
        }

        Iterator end() {
            return Iterator(nullptr);
        }

        ConstIterator begin() const {
            return ConstIterator(first_);
        }

        ConstIterator end() const {
            return ConstIterator(nullptr);
        }

        /**
         * Return the number of edges in the list.
         */
        int size() const {
            return size_;
        }

        /**
         * Return if the list is empty.
         */
        bool empty() const {
            return size_ == 0;
        }

    private:
        /**
         * Push back a node into list.
         */
        void PushBack(Edge* e) {
            if (!first_) {
                first_ = e;
                last_ = e;
                e->next_ = nullptr;
                e->prev_ = nullptr;
            } else {
                e->prev_ = last_;
                e->next_ = nullptr;
                last_->next_ = e;
                last_ = e;
            }
            ++size_;
        }

        /**
         * Erase the node from list.
         */
        void Erase(Edge* e) {
            CHECK(size_ > 0);

            if (e == first_) {
                first_ = e->next_;
            }
            if (e == last_) {
                last_ = e->prev_;
            }

            if (e->next_) e->next_->prev_ = e->prev_;
            if (e->prev_) e->prev_->next_ = e->next_;
            e->prev_ = nullptr;
            e->next_ = nullptr;
            --size_;
        }

        Edge* first_ = nullptr; // The first node in the edge list.
        Edge* last_ = nullptr;  // The last node in the edge list.
        int size_ = 0;          // Number of edges in the list.
    };

    template <typename T>
    using EdgeProperty = typename IndexedList<BaseEdge>::template Property<T>;

    explicit Graph(int n_vertices = 0)
        : n_vertices_(n_vertices),
          edges_from_(n_vertices) {
        CHECK(n_vertices_ >= 0);
    }

    /**
     * Use Clone() instead.
     */
    Graph(const Graph& graph) = delete;

    /**
     * Use Clone() instead.
     */
    Graph& operator=(const Graph& graph) = delete;

    /**
     * Return the number of vertices.
     */
    int n_vertices() const {
        return n_vertices_;
    }

    /**
     * Return the number of edges.
     */
    int n_edges() const {
        return edges_.n_available();
    }

    /**
     * Return the number of allocated edges.
     */
    int n_allocated_edges() const {
        return edges_.n_allocated();
    }

    /**
     * Return true if the graph is empty.
     */
    bool empty() const {
        return n_vertices_ == 0;
    }

    /**
     * Clear the data of graph.
     */
    void clear() {
        n_vertices_ = 0;
        n_two_way_edges_ = 0;
        edges_from_.clear();
        edges_.clear();
    }

    /**
     * Resize the vertices of graph. If the given number of vertices is larger
     * than the original number of vertices, the graph will be enlarged and the
     * original edges are preserved. Otherwise, we clear the edges.
     */
    void Resize(int n_vertices) {
        CHECK(n_vertices >= 0);

        if (n_vertices == n_vertices_) return;

        if (n_vertices > n_vertices_) {
            edges_from_.resize(n_vertices);
        } else {
            this->clear();
            Array<EdgeList> t(n_vertices);
            edges_from_.swap(t);
        }
        n_vertices_ = n_vertices;
    }

    /**
     * Clone this graph.
     *
     * Note that, we do NOT clone the properties.
     */
    void Clone(Graph* graph) const {
        CHECK(graph);

        if (this == graph) return;

        graph->Resize(n_vertices_);
        edges_.Clone(&graph->edges_);
        graph->n_two_way_edges_ = n_two_way_edges_;

        using Iterator = IndexedList<BaseEdge>::Iterator;
        using ConstIterator = IndexedList<BaseEdge>::ConstIterator;

        ConstIterator e1 = edges_.begin();
        Iterator e2 = graph->edges_.begin();
        for (; e1 != edges_.end(); ++e1, ++e2) {
            if (e1->twin_)
                e2->twin_ = graph->edges_[e1->twin()->id()];
        }

        for (int i = 0; i < n_vertices_; ++i) {
            for (const Edge* e : edges_from_[i]) {
                graph->edges_from_.at(i).PushBack(graph->edges_[e->id()]);
            }
        }
    }

    /**
     * Insert an isolated vertex.
     */
    int InsertVertex() {
        EdgeList list;
        edges_from_.push_back(list);

        return n_vertices_++;
    }

    /**
     * Insert a one-way edge, return the inserted edge.
     */
    Edge* InsertOneWayEdge(int source, int target) {
        CHECK(source >= 0 && source < n_vertices_);
        CHECK(target >= 0 && target < n_vertices_);

        Edge* e = edges_.Allocate();
        e->source_ = source;
        e->target_ = target;
        e->twin_   = nullptr;

        edges_from_[source].PushBack(e);
        return e;
    }

    /**
     * Insert a one-way edge, return the inserted edge.
     */
    Edge* InsertTwoWayEdge(int source, int target) {
        Edge* e1 = InsertOneWayEdge(source, target);
        Edge* e2 = InsertOneWayEdge(target, source);
        e1->twin_ = e2;
        e2->twin_ = e1;
        ++n_two_way_edges_;
        return e1;
    }

    /**
     * Find the first edge equal to (source, target).
     */
    Edge* FindEdge(int source, int target) {
        CHECK(source >= 0 && source < n_vertices_);
        CHECK(target >= 0 && target < n_vertices_);

        for (Edge* e : edges_from_[source]) {
            if (e->target_ == target) return e;
        }
        return nullptr;
    }

    /**
     * Find the first edge equal to (source, target).
     */
    const Edge* FindEdge(int source, int target) const {
        CHECK(source >= 0 && source < n_vertices_);
        CHECK(target >= 0 && target < n_vertices_);

        for (const Edge* e : edges_from_[source]) {
            if (e->target_ == target) return e;
        }
        return nullptr;
    }

    /**
     * Erase a one-way edge.
     */
    void EraseOneWayEdge(Edge* e) {
        if (e->twin_ != nullptr) {
            e->twin_->twin_ = nullptr;
        }
        edges_from_[e->source_].Erase(e);
        edges_.Deallocate(e);
    }

    /**
     * Erase a two-way edge.
     */
    void EraseTwoWayEdge(Edge* e) {
        if (e->twin_ != nullptr) {
            EraseOneWayEdge(e->twin_);
            --n_two_way_edges_;
        }
        EraseOneWayEdge(e);
    }

    /**
     * Erase all of two-way edges start from vertex i.
     */
    void EraseEdgesOfVertex(int i) {
        Array<Edge*> edges;
        for (Edge* e : edges_from(i)) {
            edges.push_back(e);
        }
        for (Edge* e : edges) {
            EraseTwoWayEdge(e);
        }
    }

    /**
     * Return the edges whose source vertex equal to v.
     */
    EdgeList& edges_from(int v) {
        CHECK(v >= 0 && v < n_vertices_);

        return edges_from_[v];
    }

    /**
     * Return the edges whose source vertex equal to v.
     */
    const EdgeList& edges_from(int v) const {
        CHECK(v >= 0 && v < n_vertices_);

        return edges_from_[v];
    }

    /**
     * Return the edge with the ID: 'eid'.
     */
    Edge* edge(int eid) {
        CHECK(eid >= 0 && eid < edges_.n_allocated());
        return edges_[eid];
    }

    /**
     * Return the edge with the ID: 'eid'.
     */
    const Edge* edge(int eid) const {
        CHECK(eid >= 0 && eid < edges_.n_allocated());
        return edges_[eid];
    }

    /**
     * Return true if the given vertex is available.
     */
    bool IsAvailable(Edge* e) const {
        return edges_.IsAvailable(e);
    }

    /**
     * Add a new edge property with a given name.
     */
    template <typename T>
    EdgeProperty<T> AddEdgeProperty(const std::string& name,
                                    const T& initial_value) {
        return edges_.AddProperty(name, initial_value);
    }
    template <typename T>
    EdgeProperty<T> AddEdgeProperty(const std::string& name) {
        return edges_.AddProperty(name, T());
    }

    /**
     * Add a const edge property.
     */
    template <typename T>
    EdgeProperty<T> AddEdgeProperty(const T& initial_value = T()) const {
        return edges_.AddProperty(initial_value);
    }

    /**
     * Erase a edge property.
     */
    void EraseEdgeProperty(const std::string& name) {
        edges_.EraseProperty(name);
    }

    /**
     * Check if the graph is bidirectional.
     */
    bool IsBidirectional() const {
        return n_two_way_edges_ * 2 == n_edges();
    }

    /**
     * Get the sub-graph according to the given vertices.
     *
     * Parameters:
     *   vertices - the vertices of sub-graph
     *   graph    - returned sub-graph
     *   map      - from the new vertex to old vertex.
     */
    void GetSubGraph(const Array<int>& vertices, Graph* graph,
                     Array<int>* map) const {
        CHECK(graph);
        CHECK(map);

        std::unordered_map<int, int> h;
        Array<bool> flag(n_vertices_, false);

        int n = 0;
        for (int v : vertices) {
            CHECK(v >= 0 && v < n_vertices_);
            flag[v] = true;
            if (h.find(v) == h.end()) h[v] = n++;
        }

        // Map new vertex to old vertex.
        map->resize(n);
        for (const auto& v : h) {
            (*map)[v.second] = v.first;
        }

        graph->Resize(n);
        for (int i = 0; i < n_vertices_; ++i) {
            if (flag[i]) {
                for (const Edge* e : edges_from(i)) {
                    if (flag[e->target()]) {
                        if (e->twin()) {
                            if (i < e->target()) {
                                graph->InsertTwoWayEdge(h[i], h[e->target()]);
                            } else if (i == e->target() && e->id() % 2 == 0) {
                                // Treat the self-loop situation specially.
                                graph->InsertTwoWayEdge(h[i], h[i]);
                            }
                        } else {
                            graph->InsertOneWayEdge(h[i], h[e->target()]);
                        }
                    }
                }
            }
        }
    }

private:
    // Number of vertices.
    int n_vertices_ = 0;

    // Number of two way edges.
    int n_two_way_edges_ = 0;

    // edges_from_[v] stores the edges with the source vertex equal to v.
    Array<EdgeList> edges_from_;

    // Edge pool.
    IndexedList<BaseEdge> edges_;
};

} // namespace cl

#endif // CODELIBRARY_GRAPH_GRAPH_H_
