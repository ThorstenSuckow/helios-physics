#ifndef NDEBUG
    #define HELIOS_DEBUG 1
#endif

#if defined(_MSC_VER)
    #define HELIOS_FUNCTION_SIGNATURE __FUNCSIG__
#elif defined(__clang__) || defined(__GNUC__)
    #define HELIOS_FUNCTION_SIGNATURE __PRETTY_FUNCTION__
#else
    #define HELIOS_FUNCTION_SIGNATURE __func__
#endif
