#define GET_MACRO(_1, _2, _3, NAME, ...) NAME

#define key(key, rest)                                                         \
  cout << key << ": ";                                                         \
  rest;

#define str(key, value) cout << format("{}: {}, ", key, value);

#define formatted(key, value, fmt)                                             \
  cout << key << ": " << format(fmt, value) << ", ";

#define val(...) GET_MACRO(__VA_ARGS__, formatted, str, key)(__VA_ARGS__)

#define obj(rest)                                                              \
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
