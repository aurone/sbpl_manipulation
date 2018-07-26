#ifndef SMPL_ARRAY_RANGE_H
#define SMPL_ARRAY_RANGE_H

#include <utility>

namespace sbpl {
namespace motion {

template <class InputIt>
struct range
{
    std::pair<InputIt, InputIt> p;

    range() = default;
    range(InputIt first, InputIt last) : p(first, last) { }
};

template <class InputIt>
auto make_range(InputIt first, InputIt last) -> range<InputIt>
{
    return range<InputIt>(first, last);
}

template <class InputIt>
auto begin(const range<InputIt>& r) -> InputIt
{
    return r.p.first;
}

template <class InputIt>
auto end(const range<InputIt>& r) -> InputIt
{
    return r.p.second;
}

} // namespace motion
} // namespace sbpl

#endif

