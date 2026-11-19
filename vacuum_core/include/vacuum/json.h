// json.h — minimal JSON reader/writer for flat configuration objects.
//
// Why not a library: vacuum_core must stay dependency-light (Eigen is the
// only third-party header allowed) and every JSON it ever sees is a flat
// object of numbers / bools / strings (RobotConfig, task specs). This parses
// exactly that and nothing more; anything richer fails loudly instead of
// being half-understood.

#pragma once

#include <cctype>
#include <cstdlib>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

namespace vacuum {

class FlatJson {
 public:
  // Parses {"key": value, ...} where value is number, true/false, "string".
  static FlatJson parse(const std::string& text) {
    FlatJson out;
    const char* p = text.c_str();
    skipWs(p);
    expect(p, '{');
    skipWs(p);
    if (*p == '}') return out;
    for (;;) {
      skipWs(p);
      const std::string key = parseString(p);
      skipWs(p);
      expect(p, ':');
      skipWs(p);
      if (*p == '"') {
        out.strings_[key] = parseString(p);
      } else if (std::strncmp(p, "true", 4) == 0) {
        out.bools_[key] = true;
        p += 4;
      } else if (std::strncmp(p, "false", 5) == 0) {
        out.bools_[key] = false;
        p += 5;
      } else {
        char* end = nullptr;
        const double v = std::strtod(p, &end);
        if (end == p) throw std::runtime_error("FlatJson: bad value for key '" + key + "'");
        out.numbers_[key] = v;
        p = end;
      }
      skipWs(p);
      if (*p == ',') { ++p; continue; }
      expect(p, '}');
      break;
    }
    skipWs(p);
    if (*p != '\0') throw std::runtime_error("FlatJson: trailing characters");
    return out;
  }

  double num(const std::string& key, double fallback) const {
    auto it = numbers_.find(key);
    return it == numbers_.end() ? fallback : it->second;
  }
  bool boolean(const std::string& key, bool fallback) const {
    auto it = bools_.find(key);
    return it == bools_.end() ? fallback : it->second;
  }
  std::string str(const std::string& key, const std::string& fallback) const {
    auto it = strings_.find(key);
    return it == strings_.end() ? fallback : it->second;
  }
  bool has(const std::string& key) const {
    return numbers_.count(key) || bools_.count(key) || strings_.count(key);
  }

  // --- writing (used for RobotConfig::toJson round-trip) ---
  void set(const std::string& key, double v) { numbers_[key] = v; }
  // int overload: without it, set(key, someInt) is ambiguous between double
  // and bool. Stored as a number.
  void set(const std::string& key, int v) { numbers_[key] = v; }
  void set(const std::string& key, bool v) { bools_[key] = v; }
  void set(const std::string& key, const std::string& v) { strings_[key] = v; }

  std::string dump() const {
    std::ostringstream os;
    os << '{';
    bool first = true;
    auto sep = [&] { if (!first) os << ','; first = false; };
    for (const auto& kv : numbers_) {
      sep();
      os << '"' << kv.first << "\":" << kv.second;
    }
    for (const auto& kv : bools_) {
      sep();
      os << '"' << kv.first << "\":" << (kv.second ? "true" : "false");
    }
    for (const auto& kv : strings_) {
      sep();
      os << '"' << kv.first << "\":\"" << escape(kv.second) << '"';
    }
    os << '}';
    return os.str();
  }

 private:
  std::map<std::string, double> numbers_;
  std::map<std::string, bool> bools_;
  std::map<std::string, std::string> strings_;

  static void skipWs(const char*& p) {
    while (std::isspace(static_cast<unsigned char>(*p))) ++p;
  }
  static void expect(const char*& p, char c) {
    if (*p != c) throw std::runtime_error(std::string("FlatJson: expected '") + c + "'");
    ++p;
  }
  static std::string parseString(const char*& p) {
    expect(p, '"');
    std::string out;
    while (*p && *p != '"') {
      if (*p == '\\' && p[1]) {
        ++p;
        switch (*p) {
          case 'n': out += '\n'; break;
          case 't': out += '\t'; break;
          // covers escaped quote and escaped backslash
          default: out += *p; break;
        }
      } else {
        out += *p;
      }
      ++p;
    }
    expect(p, '"');
    return out;
  }
  static std::string escape(const std::string& s) {
    std::string out;
    for (char c : s) {
      if (c == '"' || c == '\\') out += '\\';
      out += c;
    }
    return out;
  }
};

}  // namespace vacuum
