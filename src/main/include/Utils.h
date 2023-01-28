
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
    if (!(res)) {                                                              \
        std::cerr << "OKC_CHECK FAIL [" << __FILE__ << ":" << __LINE__         \
                  << "] - " << __SHOW_LINE_INFO__ << std::endl;                \
        return false;                                                          \
    }

#define OKC_CHECK(check)                                                       \
    if (!(check)) {                                                            \
        std::cerr << "OKC_CHECK FAIL [" << __FILE__ << ":" << __LINE__         \
                  << "] - " << __SHOW_LINE_INFO__ << std::endl;                \
        return false;                                                          \
    }

#define OKC_CHECK_MSG(check, msg)                                              \
    if (!(check)) {                                                            \
        std::cerr << "OKC_CHECK FAIL [" << __FILE__ << ":" << __LINE__         \
                  << "] - " << __SHOW_LINE_INFO__ << ": " << msg << std::endl; \
        return false;                                                          \
    }

#define VOKC_CALL(res)                                                         \
    if (!(res)) {                                                              \
        std::cerr << "OKC_CHECK FAIL [" << __FILE__ << ":" << __LINE__         \
                  << "] - " << __SHOW_LINE_INFO__ << std::endl;                \
        return;                                                                \
    }

#define VOKC_CHECK(check)                                                      \
    if (!(check)) {                                                            \
        std::cerr << "OKC_CHECK FAIL [" << __FILE__ << ":" << __LINE__         \
                  << "] - " << __SHOW_LINE_INFO__ << std::endl;                \
        return;                                                                \
    }

#define VOKC_CHECK_MSG(check, msg)                                             \
    if (!(check)) {                                                            \
        std::cerr << "OKC_CHECK FAIL [" << __FILE__ << ":" << __LINE__         \
                  << "] - " << __SHOW_LINE_INFO__ << ": " << msg << std::endl; \
        return;                                                                \
    }

namespace TeamOKC {

    bool Clamp(const double &lower, const double &upper, double *value);

}