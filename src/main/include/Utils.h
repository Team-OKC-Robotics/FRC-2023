
#pragma once

#include <iostream>

#define OKC_CALL(res)                                                          \
    if (!(res)) {                                                              \
        std::cerr << "[" << __FILE__ << "(" << __LINE__ << ")] - OKC_CALL("    \
                  << #res << ")" << std::endl;                                 \
        return false;                                                          \
    }

#define OKC_CHECK(check)                                                       \
    if (!(check)) {                                                            \
        std::cerr << "[" << __FILE__ << "(" << __LINE__ << ")] - OKC_CHECK("   \
                  << #check << ")" << std::endl;                               \
        return false;                                                          \
    }

#define OKC_CHECK_MSG(check, msg)                                              \
    if (!(check)) {                                                            \
        std::cerr << "[" << __FILE__ << "(" << __LINE__                        \
                  << ")] - OKC_CHECK_MSG(" << #check << "): " << msg           \
                  << std::endl;                                                \
        return false;                                                          \
    }

#define VOKC_CALL(res)                                                         \
    if (!(res)) {                                                              \
        std::cerr << "[" << __FILE__ << "(" << __LINE__ << ")] - VOKC_CALL("   \
                  << #res << ")" << std::endl;                                 \
        return;                                                                \
    }

#define VOKC_CHECK(check)                                                      \
    if (!(check)) {                                                            \
        std::cerr << "[" << __FILE__ << "(" << __LINE__ << ")] - VOKC_CHECK("  \
                  << #check << ")" << std::endl;                               \
        return;                                                                \
    }

#define VOKC_CHECK_MSG(check, msg)                                             \
    if (!(check)) {                                                            \
        std::cerr << "[" << __FILE__ << "(" << __LINE__                        \
                  << ")] - VOKC_CHECK_MSG(" << #check << "): " << msg          \
                  << std::endl;                                                \
        return;                                                                \
    }

namespace TeamOKC {

    bool Clamp(const double &lower, const double &upper, double *value);

}