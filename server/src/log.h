#pragma once
#include <fmt/core.h>

#define GET_MACRO(_1, _2, _3, NAME, ...) NAME

class NullBuffer : public std::streambuf
{
public:
  int overflow( int c ) override { return c; }
};

inline NullBuffer Null;
inline std::ostream NullStream( &Null );
inline std::ostream& output_stream = std::cout;
// inline std::ostream& output_stream = NullStream;

#define key_log(key, rest)                                                         \
  output_stream << key << ": ";                                                         \
  rest;

#define str_log(key, value) output_stream << fmt::format("{}: {}, ", key, value);

#define formatted(key, value, fm)                                              \
  output_stream << key << ": " << fmt::format(fm, value) << ", ";

#define val(...) GET_MACRO(__VA_ARGS__, formatted, str_log, key)(__VA_ARGS__)

#define obj_log(rest)                                                              \
  output_stream << "{";                                                                 \
  rest;                                                                        \
  output_stream << "}, ";

#define list(rest)                                                             \
  output_stream << "[";                                                                 \
  rest;                                                                        \
  output_stream << "], ";

#define item(rest)                                                             \
  output_stream << "{";                                                                 \
  rest;                                                                        \
  output_stream << "}" << endl;
