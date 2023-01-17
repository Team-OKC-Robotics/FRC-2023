
#pragma once

#include <iostream>

// Despite what VS Code highlighting might suggest, this uses __FUNCTION__ when
// compiliing using Visual Studio on Windows, and otherwise uses
// __PRETTY_FUNCTION__
#if !defined(__PRETTY_FUNCTION__) && !defined(__GNUC__)
#define __SHOW_LINE_INFO__ __FUNCTION__
#else
#define __SHOW_LINE_INFO__ __PRETTY_FUNCTION__
#endif

#define OKC_CALL(res)                                                          \
    do {                                                                       \
        if (!(res)) {                                                          \
            std::cerr << "OKC_CHECK FAIL [" << __FILE__ << ":" << __LINE__     \
                      << "] - " << #res << std::endl;                          \
            return false;                                                      \
        }                                                                      \
    } while (0)

#define OKC_CHECK(check)                                                       \
    do {                                                                       \
        if (!(check)) {                                                        \
            std::cerr << "OKC_CHECK FAIL [" << __FILE__ << ":" << __LINE__     \
                      << "] - " << #check << std::endl;                        \
            return false;                                                      \
        }                                                                      \
    } while (0)

#define OKC_CHECK_MSG(check, msg)                                              \
    do {                                                                       \
        if (!(check)) {                                                        \
            std::cerr << "OKC_CHECK FAIL [" << __FILE__ << ":" << __LINE__     \
                      << "] - " << #check << ": " << msg << std::endl;         \
            return false;                                                      \
        }                                                                      \
    } while (0)

#define VOKC_CALL(res)                                                         \
    do {                                                                       \
        if (!(res)) {                                                          \
            std::cerr << "OKC_CHECK FAIL [" << __FILE__ << ":" << __LINE__     \
                      << "] - " << #res << std::endl;                          \
            return;                                                            \
        }                                                                      \
    } while (0)

#define VOKC_CHECK(check)                                                      \
    do {                                                                       \
        if (!(check)) {                                                        \
            std::cerr << "OKC_CHECK FAIL [" << __FILE__ << ":" << __LINE__     \
                      << "] - " << #check << std::endl;                        \
            return;                                                            \
        }                                                                      \
    } while (0)

#define VOKC_CHECK_MSG(check, msg)                                             \
    do {                                                                       \
        if (!(check)) {                                                        \
            std::cerr << "OKC_CHECK FAIL [" << __FILE__ << ":" << __LINE__     \
                      << "] - " << #check << ": " << msg << std::endl;         \
            return;                                                            \
        }                                                                      \
    } while (0)

namespace TeamOKC {

    bool Clamp(const double &lower, const double &upper, double *value);

}