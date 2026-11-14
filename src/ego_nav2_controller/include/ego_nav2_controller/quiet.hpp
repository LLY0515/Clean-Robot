#pragma once
#include <ostream>
namespace ego_quiet {
class Buffer : public std::streambuf { int overflow(int c) override { return c; } };
inline std::ostream &stream() { static thread_local Buffer b; static thread_local std::ostream s(&b); return s; }
template<class... T> int print(const char *, T...) { return 0; }
}
