#ifndef DR_UTILS_H
#define DR_UTILS_H

#include <cstddef>
#include <deque>
#include <cstdint>
#include <vector>

namespace dr_utils {

template<typename T>
class DequeArray
{
public:
    typedef typename std::deque<T>::iterator iterator;
    typedef typename std::deque<T>::const_iterator const_iterator;
    DequeArray(){};
    DequeArray(int32_t size) : valid_size_(size) {}
    void push_back(T i)
    {
        d.push_back(i);
        if (d.size() > valid_size_) d.pop_front();
    }
    virtual void push_front(T i)
    {
        d.push_front(i);
        if (d.size() > valid_size_) d.pop_back();
    }

    void pop_front() { d.pop_front(); }
    bool empty() { return d.size() == 0; }

    iterator erase(const iterator& it) { return d.erase(it); }

    T& front() { return d.front(); }

    T& back() { return d.back(); }

    typename std::deque<T>::iterator begin() { return d.begin(); }

    typename std::deque<T>::iterator end() { return d.end(); }

    size_t size() const { return d.size(); }

    void clear() { d.clear(); }

    T operator[](int32_t i) { return d[i]; }

    bool is_full() { return valid_size_ == d.size(); }

    void resize(uint32_t size)
    {
        if (valid_size_ != size) valid_size_ = size;
    }

protected:
    std::deque<T> d;
    uint32_t valid_size_{3};
};

class Varint
{
public:
    std::vector<uint8_t> encode(uint32_t encode_data)
    {
        std::vector<uint8_t> tmp_encode_data{};

        for (auto i = 0; encode_data > 0x7F; i++)
        {
            // Little-endian order
            uint8_t tmp_data = (static_cast<uint8_t>(encode_data & 0x7F) | 0x80);
            tmp_encode_data.emplace_back(static_cast<uint8_t>(tmp_data));
            encode_data >>= 7u;
        }
        tmp_encode_data.emplace_back(static_cast<uint8_t>(encode_data));
        return tmp_encode_data;
    };

    uint32_t decode(std::vector<uint8_t> decode_data)
    {
        uint32_t tmp_decode_data = 0;
        if (decode_data.empty())
        {
            return 0;
        }

        auto i = 0;
        for (auto&& data : decode_data)
        {
            tmp_decode_data |= (data & 0x7F) << (7 * i);
            i++;
            if ((data & 0x80) == 0)
            {
                return tmp_decode_data;
            }
        }
        return 0;
    };
};

} // namespace dr_utils

#endif // DR_UTILS_H