#ifndef PQUEUE_H
#define PQUEUE_H

#include "Waypoint.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace algos {
class PriorityQueue {
 private:
  struct QueueEntry {
    std::pair<double, double> key;
    int vertex_idx;
    bool valid;

    QueueEntry(std::pair<double, double> k, int idx) : key(k), vertex_idx(idx), valid(true) {}
  };

  // Binary heap for efficient min extraction
  std::vector<QueueEntry> heap;

  // Fast lookup: vertex_idx -> heap_index
  std::unordered_map<int, int> vertex_to_heap_pos;

  // Track which vertices are in queue
  std::unordered_set<int> vertices_in_queue;

 public:
  bool KeyLess(const std::pair<double, double>& key1, const std::pair<double, double>& key2);
  void HeapifyUp(int idx);
  void HeapifyDown(int idx);
  void RemoveFromMaps(int vertex_idx);
  PriorityQueue();

  // Insert or update vertex with new key
  void InsertOrUpdate(int vertex_idx, std::pair<double, double> new_key);

  // Remove specific vertex from queue
  void Remove(int vertex_idx);

  // Get minimum element without removing
  std::pair<std::pair<double, double>, int> Top() const;

  // Remove and return minimum element
  std::pair<std::pair<double, double>, int> PopMin();

  // Check if vertex is in queue
  bool Contains(int vertex_idx) const;

  // Get current key for vertex (if in queue)
  std::pair<double, double> GetKey(int vertex_idx) const;

  // Queue management
  bool Empty() const;
  size_t Size() const;

  void Clear();

  // Validate heap integrity (debug)
  bool ValidateHeap();

  // Efficient cleanup - remove stale entries
  void CompactQueue(std::function<bool(int)> is_vertex_valid,
                    std::function<std::pair<double, double>(int)> get_current_key);
};

}  // namespace algos

#endif