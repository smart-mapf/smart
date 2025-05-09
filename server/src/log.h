#pragma once
#include <fmt/core.h>

#define GET_MACRO(_1, _2, _3, NAME, ...) NAME

#define key_log(key, rest)                                                         \
  cout << key << ": ";                                                         \
  rest;

#define str_log(key, value) cout << fmt::format("{}: {}, ", key, value);

#define formatted(key, value, fm)                                              \
  cout << key << ": " << fmt::format(fm, value) << ", ";

#define val(...) GET_MACRO(__VA_ARGS__, formatted, str_log, key)(__VA_ARGS__)

#define obj_log(rest)                                                              \
  cout << "{";                                                                 \
  rest;                                                                        \
  cout << "}, ";

#define list(rest)                                                             \
  cout << "[";                                                                 \
  rest;                                                                        \
  cout << "], ";

#define item(rest)                                                             \
  cout << "{";                                                                 \
  rest;                                                                        \
  cout << "}" << endl;
