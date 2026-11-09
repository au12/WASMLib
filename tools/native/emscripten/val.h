// A stand-in for Emscripten's `val` so WASMLib's bindings (bind/build.cpp) build natively: replaying a real call under
// callgrind or with SMLib's error trace on, outside WebAssembly. Only what the bindings use: a JSON-like value
// (null, undefined, bool, number, string, array, object), shared by reference as JavaScript's are.
#pragma once
#include <functional>
#include <map>
#include <type_traits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace emscripten {
class val {
 public:
  enum Kind { UNDEF, NUL, BOOL, NUM, STR, ARR, OBJ };
  struct Rep { Kind k = UNDEF; bool b = false; double n = 0; std::string s; std::vector<val> a; std::map<std::string, val> o; };
  std::shared_ptr<Rep> r;
  val() : r(std::make_shared<Rep>()) {}
  explicit val(Kind k) : val() { r->k = k; }
  val(double d) : val() { r->k = NUM; r->n = d; }
  val(int d) : val(double(d)) {}
  val(unsigned d) : val(double(d)) {}
  val(long d) : val(double(d)) {}
  val(unsigned long d) : val(double(d)) {}
  val(bool b) : val() { r->k = BOOL; r->b = b; }
  val(const char* s) : val() { r->k = STR; r->s = s; }
  val(const std::string& s) : val() { r->k = STR; r->s = s; }
  static val object() { return val(OBJ); }
  static val array() { return val(ARR); }
  static val null() { return val(NUL); }
  static val undefined() { return val(UNDEF); }
  bool isNull() const { return r->k == NUL; }
  bool isUndefined() const { return r->k == UNDEF; }
  val operator[](const char* k) const { auto it = r->o.find(k); if (it != r->o.end()) return it->second; if (r->k == ARR && std::string(k) == "length") return val(double(r->a.size())); return val(); }
  val operator[](const std::string& k) const { return (*this)[k.c_str()]; }
  val operator[](unsigned i) const { return i < r->a.size() ? r->a[i] : val(); }
  val operator[](int i) const { return (*this)[unsigned(i)]; }
  template <class T> void set(const char* k, const T& v) { r->k = r->k == UNDEF ? OBJ : r->k; r->o[k] = val(v); }
  void put(unsigned i, const val& v) { r->k = r->k == UNDEF ? ARR : r->k; if (r->a.size() <= i) r->a.resize(i + 1); r->a[i] = v; }
  template <class I, class T> std::enable_if_t<std::is_integral_v<I>> set(I i, const T& v) { put(unsigned(i), val(v)); }

  template <class T> T as() const {
    if constexpr (std::is_same_v<T, bool>) return r->k == BOOL ? r->b : r->n != 0;
    else if constexpr (std::is_same_v<T, std::string>) return r->s;
    else return static_cast<T>(r->k == BOOL ? double(r->b) : r->n);
  }
  template <class R, class... A> R call(const char* name, A&&... args) {
    if (std::string(name) == "push") { (r->a.push_back(val(args)), ...); if constexpr (!std::is_void_v<R>) return R(); else return; }
    throw std::runtime_error(std::string("val shim: no method ") + name);
  }
};
template <class T> std::vector<T> vecFromJSArray(const val& v) { std::vector<T> out; for (auto& x : v.r->a) out.push_back(x.as<T>()); return out; }

/** JSON text as a val (numbers, strings, true/false/null, arrays, objects) */
inline val parse_json(const std::string& s) {
  size_t i = 0;
  std::function<val()> value;
  auto ws = [&] { while (i < s.size() && isspace((unsigned char)s[i])) i++; };
  value = [&]() -> val {
    ws();
    if (s[i] == '{') { i++; val o = val::object(); ws(); if (s[i] == '}') { i++; return o; }
      for (;;) { ws(); i++; std::string k; while (s[i] != '"') k += s[i++]; i++; ws(); i++; o.set(k.c_str(), value()); ws(); if (s[i] == ',') { i++; continue; } i++; return o; } }
    if (s[i] == '[') { i++; val a = val::array(); ws(); if (s[i] == ']') { i++; return a; }
      for (unsigned n = 0;; n++) { a.set(n, value()); ws(); if (s[i] == ',') { i++; continue; } i++; return a; } }
    if (s[i] == '"') { i++; std::string k; while (s[i] != '"') k += s[i++]; i++; return val(k); }
    if (!s.compare(i, 4, "true")) { i += 4; return val(true); }
    if (!s.compare(i, 5, "false")) { i += 5; return val(false); }
    if (!s.compare(i, 4, "null")) { i += 4; return val::null(); }
    size_t e = i; double d = std::stod(s.substr(i), &e); i += e; return val(d);
  };
  return value();
}
}  // namespace emscripten
