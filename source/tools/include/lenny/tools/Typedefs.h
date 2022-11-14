#pragma once

#include <functional>
#include <memory>

#define LENNY_GENERAGE_TYPEDEFS(X)          \
    typedef std::unique_ptr<X> UPtr;        \
    typedef std::unique_ptr<const X> CUPtr; \
    typedef std::shared_ptr<X> SPtr;        \
    typedef std::shared_ptr<const X> CSPtr; \
    typedef std::reference_wrapper<X> Ref;  \
    typedef std::reference_wrapper<const X> CRef;
