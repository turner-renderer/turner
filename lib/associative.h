#pragma once

#include "range.h"
#include <cassert>
#include <limits>
#include <vector>

template <typename Key, typename Value> class AssociativeArray {

    static constexpr size_t INVALID_BUCKET_INDEX =
        std::numeric_limits<size_t>::max();
    static constexpr size_t INVALID_VALUE_INDEX =
        std::numeric_limits<size_t>::max();

    struct Bucket {
        Key key;
        size_t begin;
        size_t end;

        Bucket(Key key, size_t begin, size_t end)
            : key(std::move(key)), begin(begin), end(end) {}

        explicit operator bool() const { return begin != INVALID_VALUE_INDEX; }
        bool empty() const { return begin < end; }
        size_t size() const { return end - begin; }
        void mark_invalid() {
            begin = INVALID_VALUE_INDEX;
            end = INVALID_VALUE_INDEX;
        }
    };

public:
    size_t size(size_t index) const {
        assert(index < buckets_.size());
        return buckets_[index].size();
    }

    Range<Value*> values(size_t index) {
        assert(index < buckets_.size());
        const auto& bucket = buckets_[index];
        return {values_.data() + bucket.begin, bucket.size()};
    }

    void push_back(Key key) {
        buckets_.emplace_back(std::move(key), values_.size(), values_.size());
    }

    void insert(size_t index, Value val) {
        assert(index < buckets_.size());
        auto& bucket = buckets_[index];

        size_t prev = prev_bucket_index(index);
        if (prev != INVALID_BUCKET_INDEX) {
            const auto& prev_bucket = buckets_[prev];
            if (prev_bucket.end < bucket.begin) {
                bucket.begin -= 1;
                values_[bucket.begin] = std::move(val);
                return;
            }
        } else if (bucket.begin > 0) {
            bucket.begin -= 1;
            values_[bucket.begin] = std::move(val);
            return;
        }

        size_t next = next_bucket_index(index);
        if (next != INVALID_BUCKET_INDEX) {
            const auto& next_bucket = buckets_[next];
            if (bucket.end < next_bucket.begin) {
                values_[bucket.end] = std::move(val);
                bucket.end += 1;
                return;
            }
        } else {
            if (bucket.end < values_.size()) {
                values_[bucket.end] = val;
            } else {
                values_.push_back(std::move(val));
            }
            bucket.end += 1;
            return;
        }

        // we are somewere in the middle, and there is no place before or after
        // the bucket => move to the end and mark the current bucket as invalid
        push_back(std::move(bucket.key));
        auto& new_bucket = buckets_.back();
        new_bucket.begin = values_.size();
        new_bucket.end = values_.size() + bucket.size() + 1;
        values_.resize(new_bucket.end);
        std::move(values_.begin() + bucket.begin, values_.begin() + bucket.end,
                  values_.begin() + new_bucket.begin);
        values_.back() = std::move(val);
    }

private:
    size_t prev_bucket_index(size_t idx) const {
        if (idx == 0) {
            return INVALID_BUCKET_INDEX;
        }
        for (const Bucket *b = buckets_[--idx]; b->empty(); --b, --idx) {
            if (idx == 0) {
                return INVALID_BUCKET_INDEX;
            }
        }
        return idx;
    }

    size_t next_bucket_index(size_t idx) const {
        if (idx >= buckets_.size()) {
            return INVALID_BUCKET_INDEX;
        }
        for (const Bucket *b = buckets_[idx++]; b->empty(); ++b, ++idx) {
            if (idx == buckets_.size()) {
                return INVALID_BUCKET_INDEX;
            }
        }
        return idx;
    }

private:
    std::vector<Bucket> buckets_;
    std::vector<Value> values_;
};
