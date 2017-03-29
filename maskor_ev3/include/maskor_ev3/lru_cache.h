#ifndef _LRU_CACHE_H_
#define _LRU_CACHE_H_

#include <list>

// This class implements a small LRU cache. It assumes the number of elements
// is small, and so uses a simple linear search.
template <typename K, typename V>
class lru_cache {

private:
  // typedef st::pair<K, V> item;
  // std::pair seems to be missing necessary move constructors :(
  struct item
  {
    K first;
    V second;

    item(const K &k) : first(k) {}
    item(item &&m) : first(std::move(m.first)), second(std::move(m.second)) {}
  };

public:
  lru_cache(size_t size = 3) : _size(size)
  {
  }

  V &operator[] (const K &k)
  {
    iterator i = find(k);
    if (i != _items.end())
    {
      // Found the key, bring the item to the front.
      _items.splice(_items.begin(), _items, i);
    } else {
      // If the cache is full, remove oldest items to make room.
      while (_items.size() + 1 > _size)
      {
        _items.pop_back();
      }
      // Insert a new default constructed value for this new key.
      _items.emplace_front(k);
    }
    // The new item is the most recently used.
    return _items.front().second;
  }

  void clear()
  {
    _items.clear();
  }

private:
  typedef typename std::list<item>::iterator iterator;

  iterator find(const K &k)
  {
    return std::find_if(_items.begin(), _items.end(),
                        [&](const item &i) { return i.first == k; });
  }

  size_t _size;
  std::list<item> _items;
};

#endif //LRU_CACHE_H
