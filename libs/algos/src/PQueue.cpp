#include "PQueue.h"

algos::PriorityQueue::PriorityQueue() {
  heap.reserve(200);  // Pre-allocate reasonable size
}

bool algos::PriorityQueue::KeyLess(const std::pair<double, double>& key1,
                                   const std::pair<double, double>& key2) {
  return (key1.first < key2.first) || (key1.first == key2.first && key1.second < key2.second);
}

void algos::PriorityQueue::HeapifyUp(int idx) {
  while (idx > 0) {
    int parent = (idx - 1) / 2;
    if (!KeyLess(heap[idx].key, heap[parent].key)) break;

    // Swap entries
    std::swap(heap[idx], heap[parent]);

    // Update position mappings
    vertex_to_heap_pos[heap[idx].vertex_idx] = idx;
    vertex_to_heap_pos[heap[parent].vertex_idx] = parent;

    idx = parent;
  }
}

void algos::PriorityQueue::HeapifyDown(int idx) {
  int size = static_cast<int>(heap.size());
  while (true) {
    int smallest = idx;
    int left = 2 * idx + 1;
    int right = 2 * idx + 2;

    if (left < size && KeyLess(heap[left].key, heap[smallest].key)) {
      smallest = left;
    }
    if (right < size && KeyLess(heap[right].key, heap[smallest].key)) {
      smallest = right;
    }

    if (smallest == idx) break;

    // Swap entries
    std::swap(heap[idx], heap[smallest]);

    // Update position mappings
    vertex_to_heap_pos[heap[idx].vertex_idx] = idx;
    vertex_to_heap_pos[heap[smallest].vertex_idx] = smallest;

    idx = smallest;
  }
}

void algos::PriorityQueue::RemoveFromMaps(int vertex_idx) {
  vertex_to_heap_pos.erase(vertex_idx);
  vertices_in_queue.erase(vertex_idx);
}

void algos::PriorityQueue::InsertOrUpdate(int vertex_idx, std::pair<double, double> new_key) {
  auto it = vertex_to_heap_pos.find(vertex_idx);

  if (it != vertex_to_heap_pos.end()) {
    // Vertex already in queue - update key
    int heap_pos = it->second;
    auto old_key = heap[heap_pos].key;
    heap[heap_pos].key = new_key;

    // Maintain heap property
    if (KeyLess(new_key, old_key)) {
      HeapifyUp(heap_pos);
    } else if (KeyLess(old_key, new_key)) {
      HeapifyDown(heap_pos);
    }
  } else {
    // Insert new vertex
    int new_pos = static_cast<int>(heap.size());
    heap.emplace_back(new_key, vertex_idx);
    vertex_to_heap_pos[vertex_idx] = new_pos;
    vertices_in_queue.insert(vertex_idx);
    HeapifyUp(new_pos);
  }
}

// Remove specific vertex from queue
void algos::PriorityQueue::Remove(int vertex_idx) {
  auto it = vertex_to_heap_pos.find(vertex_idx);
  if (it == vertex_to_heap_pos.end()) return;

  int pos = it->second;
  int last_pos = static_cast<int>(heap.size()) - 1;

  if (pos == last_pos) {
    // Removing last element
    RemoveFromMaps(vertex_idx);
    heap.pop_back();
  } else {
    // Move last element to this position
    auto last_vertex = heap[last_pos].vertex_idx;
    heap[pos] = heap[last_pos];
    heap.pop_back();

    // Update mappings
    RemoveFromMaps(vertex_idx);
    vertex_to_heap_pos[last_vertex] = pos;

    // Restore heap property
    HeapifyUp(pos);
    HeapifyDown(pos);
  }
}

std::pair<std::pair<double, double>, int> algos::PriorityQueue::Top() const {
  if (heap.empty()) {
    return {{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()},
            -1};
  }
  return {heap[0].key, heap[0].vertex_idx};
}

std::pair<std::pair<double, double>, int> algos::PriorityQueue::PopMin() {
  if (heap.empty()) {
    return {{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()},
            -1};
  }

  auto result = Top();
  Remove(heap[0].vertex_idx);
  return result;
}

bool algos::PriorityQueue::Contains(int vertex_idx) const {
  return vertices_in_queue.count(vertex_idx) > 0;
}

std::pair<double, double> algos::PriorityQueue::GetKey(int vertex_idx) const {
  auto it = vertex_to_heap_pos.find(vertex_idx);
  if (it == vertex_to_heap_pos.end()) {
    return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
  }
  return heap[it->second].key;
}

bool algos::PriorityQueue::Empty() const { return heap.empty(); }
size_t algos::PriorityQueue::Size() const { return heap.size(); }

void algos::PriorityQueue::Clear() {
  heap.clear();
  vertex_to_heap_pos.clear();
  vertices_in_queue.clear();
}

bool algos::PriorityQueue::ValidateHeap() {
  for (int i = 0; i < static_cast<int>(heap.size()); ++i) {
    int left = 2 * i + 1;
    int right = 2 * i + 2;

    if (left < heap.size() && KeyLess(heap[left].key, heap[i].key)) {
      return false;
    }
    if (right < heap.size() && KeyLess(heap[right].key, heap[i].key)) {
      return false;
    }

    // Check position mapping consistency
    auto it = vertex_to_heap_pos.find(heap[i].vertex_idx);
    if (it == vertex_to_heap_pos.end() || it->second != i) {
      return false;
    }
  }
  return true;
}

void algos::PriorityQueue::CompactQueue(
    std::function<bool(int)> is_vertex_valid,
    std::function<std::pair<double, double>(int)> get_current_key) {
  std::vector<QueueEntry> valid_entries;
  valid_entries.reserve(heap.size());

  for (const auto& entry : heap) {
    if (is_vertex_valid(entry.vertex_idx)) {
      auto current_key = get_current_key(entry.vertex_idx);
      // Only keep if key hasn't changed significantly
      if (std::abs(current_key.first - entry.key.first) < 1e-6 &&
          std::abs(current_key.second - entry.key.second) < 1e-6) {
        valid_entries.push_back(QueueEntry(current_key, entry.vertex_idx));
      }
    }
  }

  // Rebuild queue with valid entries
  Clear();
  for (const auto& entry : valid_entries) {
    InsertOrUpdate(entry.vertex_idx, entry.key);
  }
}