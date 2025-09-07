#pragma once
/*
===============================================================================
UartConsole.h — single-header UART console + key=value registry (Arduino/ESP-IDF)
===============================================================================
QUICK START (Arduino-style)
1) Include:
      #include "nvsSettings_definition.h"   // your controlSettings struct
      #include "UartRouter.h"
      #include "UartConsole.h"

2) In setup():
      // Optional hooks for immediate side effects when keys change
      UartConsole::Hooks hooks;
      hooks.onGainsChanged        = [&]{ pid.reset(); };
      hooks.onEnablePrintChanged  = [&](bool en){ reporter_setEnabled(en); };
      hooks.onSetSpeedChanged     = [&](int v){ motorB_setTargetTicks(v); };
      hooks.onSave                = [&]{ ** save to NVS ** return std::string("OK saved"); };
      hooks.onLoad                = [&]{ ** load from NVS ** pid.reset(); return std::string("OK loaded"); };

      UartConsole::init(settings, hooks **, UART_NUM_0, 115200 **);

3) (Optional) Software pattern watchers (arbitrary substrings):
      UartConsole::router().watchPattern("ALARM", [](const std::string& buf, size_t pos)
      {
          UartConsole::router().write("SW-PATTERN: ALARM detected\r\n");
      }, true); // consume so it won't retrigger on same bytes

      UartConsole::router().watchPattern("<STOP>", [](const std::string& buf, size_t pos)
      {
          auto r = UartConsole::kv().applyLine("setSpeed=0");
          for (auto& s : r) { UartConsole::router().write(s); UartConsole::router().write("\r\n"); }
      }, false); // don't consume; keep context

4) (Optional) Hardware pattern '+++':
      UartConsole::router().enableHWPattern('+', 3, 9, 0, 0, 20,
          [](const std::string& before, int patlen)
          {
              UartConsole::router().write("HW (+++) detected\r\n");
          });

SERIAL COMMAND CHEAT-SHEET (PC → ESP)
- Assign:   Ki=0.5  enablePrint=1  setSpeed=500
- Query:    Ki?     setSpeed?
- List all: list
- Commands: save    load      (wired via hooks)
- Booleans accepted: 1/0, true/false, on/off, yes/no (case-insensitive)

NOTES
- Keys are case-insensitive. Values keep case (except booleans).
- You can send multiple assignments in one line:  Kp=1.2 Ki=0.3 setSpeed=800
- Pattern watchers run regardless of line parsing and can span chunks/lines.
===============================================================================
*/

#include <string>
#include <vector>
#include <map>
#include <functional>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <cstdio>

#include "UartRouter.h"

// Forward-declare your controlSettings before including this, e.g.:
//   #include "nvsSettings_definition.h"

class UartConsole
{
public:
    // ----------------- KeyValueRegistry (header-only) -----------------
    class KeyValueRegistry
    {
    public:
        enum class Type
        {
            kInt,
            kFloat,
            kBool,
            kString
        };

        struct Entry
        {
            Type type;
            void* ptr;
            bool has_bounds = false;
            double min_v = 0.0;
            double max_v = 0.0;
            std::function<void()> on_change;
        };

        // Bindings
        inline void bind(const char* name, int* p)
        {
            Entry e{ Type::kInt, p };
            entries_[normalize(name)] = e;
        }

        inline void bind(const char* name, int* p, int min_v, int max_v, std::function<void()> cb = {})
        {
            Entry e{ Type::kInt, p, true, (double)min_v, (double)max_v, cb };
            entries_[normalize(name)] = e;
        }

        inline void bind(const char* name, float* p)
        {
            Entry e{ Type::kFloat, p };
            entries_[normalize(name)] = e;
        }

        inline void bind(const char* name, float* p, float min_v, float max_v, std::function<void()> cb = {})
        {
            Entry e{ Type::kFloat, p, true, (double)min_v, (double)max_v, cb };
            entries_[normalize(name)] = e;
        }

        inline void bind(const char* name, bool* p, std::function<void()> cb = {})
        {
            Entry e{ Type::kBool, p, false, 0.0, 0.0, cb };
            entries_[normalize(name)] = e;
        }

        inline void bind(const char* name, std::string* p, std::function<void()> cb = {})
        {
            Entry e{ Type::kString, p, false, 0.0, 0.0, cb };
            entries_[normalize(name)] = e;
        }

        // Commands (no '=' and no '?')
        inline void setCommand(const char* name, std::function<std::string()> cb)
        {
            commands_[normalize(name)] = cb;
        }

        // Parse a single line, return response lines
        inline std::vector<std::string> applyLine(const std::string& line)
        {
            std::vector<std::string> out;
            auto toks = tokenize(line);

            for (auto& t : toks)
            {
                std::string low = normalize(t);

                if (low == "list")
                {
                    listAll(out);
                    continue;
                }

                // command (no '=' and no trailing '?')
                if (t.find('=') == std::string::npos && (t.empty() || t.back() != '?'))
                {
                    auto itc = commands_.find(low);
                    if (itc != commands_.end())
                    {
                        out.push_back(itc->second());
                    }
                    else
                    {
                        out.push_back(err("unknown command: " + t));
                    }
                    continue;
                }

                // query: "Ki?"
                if (!t.empty() && t.back() == '?')
                {
                    std::string key = t.substr(0, t.size() - 1);
                    queryOne(key, out);
                    continue;
                }

                // assignment: "Ki=0.5"
                auto eq = t.find('=');
                if (eq == std::string::npos)
                {
                    out.push_back(err("bad token: " + t));
                    continue;
                }

                std::string key = t.substr(0, eq);
                std::string val = t.substr(eq + 1);
                applyOne(key, val, out);
            }

            if (out.empty())
            {
                out.push_back(hint("Use key=value (e.g. Ki=0.5) or key? or list"));
            }
            return out;
        }

    private:
        std::map<std::string, Entry> entries_;
        std::map<std::string, std::function<std::string()>> commands_;

        // ----- helpers -----
        static inline std::string normalize(const std::string& s)
        {
            std::string r = s;
            std::transform(r.begin(), r.end(), r.begin(), [](unsigned char c){ return (char)std::tolower(c); });
            // trim
            size_t i = 0; while (i < r.size() && std::isspace((unsigned char)r[i])) i++;
            r.erase(0, i);
            if (!r.empty())
            {
                size_t j = r.size();
                while (j > 0 && std::isspace((unsigned char)r[j-1])) j--;
                r.erase(j);
            }
            return r;
        }

        static inline std::vector<std::string> tokenize(const std::string& line)
        {
            std::vector<std::string> out;
            std::string cur;
            bool in_quotes = false;
            char q = 0;

            for (char c : line)
            {
                if (!in_quotes && std::isspace((unsigned char)c))
                {
                    if (!cur.empty())
                    {
                        out.push_back(cur);
                        cur.clear();
                    }
                    continue;
                }
                if ((c == '"' || c == '\''))
                {
                    if (!in_quotes) { in_quotes = true; q = c; }
                    else if (q == c) { in_quotes = false; }
                }
                cur.push_back(c);
            }
            if (!cur.empty()) out.push_back(cur);
            return out;
        }

        static inline bool parseBool(const std::string& s, bool& out)
        {
            std::string n = normalize(s);
            if (n == "1" || n == "true" || n == "on" || n == "yes") { out = true;  return true; }
            if (n == "0" || n == "false"|| n == "off"|| n == "no")  { out = false; return true; }
            return false;
        }

        static inline std::string ok(const std::string& m)   { return "OK " + m; }
        static inline std::string err(const std::string& m)  { return "ERR " + m; }
        static inline std::string hint(const std::string& m) { return "HINT " + m; }

        static inline std::string unquote(const std::string& v)
        {
            if (v.size() >= 2 && ((v.front() == '"' && v.back() == '"') || (v.front() == '\'' && v.back() == '\'')))
            {
                return v.substr(1, v.size() - 2);
            }
            return v;
        }

        inline void listAll(std::vector<std::string>& out)
        {
            for (auto& kv : entries_)
            {
                const auto& name = kv.first;
                const auto& e = kv.second;
                char buf[96];

                switch (e.type)
                {
                    case Type::kInt:
                        std::snprintf(buf, sizeof(buf), "%s=%d", name.c_str(), *(int*)e.ptr);
                        break;
                    case Type::kFloat:
                        std::snprintf(buf, sizeof(buf), "%s=%.6g", name.c_str(), (double)*(float*)e.ptr);
                        break;
                    case Type::kBool:
                        std::snprintf(buf, sizeof(buf), "%s=%d", name.c_str(), (int)(*(bool*)e.ptr));
                        break;
                    case Type::kString:
                        std::snprintf(buf, sizeof(buf), "%s=\"%s\"", name.c_str(), ((std::string*)e.ptr)->c_str());
                        break;
                }
                out.emplace_back(buf);
            }
        }

        inline void listOne(const std::string& name, const Entry& e, std::vector<std::string>& out)
        {
            char buf[96];
            switch (e.type)
            {
                case Type::kInt:
                    std::snprintf(buf, sizeof(buf), "%s=%d", name.c_str(), *(int*)e.ptr);
                    break;
                case Type::kFloat:
                    std::snprintf(buf, sizeof(buf), "%s=%.6g", name.c_str(), (double)*(float*)e.ptr);
                    break;
                case Type::kBool:
                    std::snprintf(buf, sizeof(buf), "%s=%d", name.c_str(), (int)(*(bool*)e.ptr));
                    break;
                case Type::kString:
                    std::snprintf(buf, sizeof(buf), "%s=\"%s\"", name.c_str(), ((std::string*)e.ptr)->c_str());
                    break;
            }
            out.emplace_back(buf);
        }

        inline void queryOne(const std::string& keyRaw, std::vector<std::string>& out)
        {
            auto key = normalize(keyRaw);
            auto it = entries_.find(key);
            if (it == entries_.end())
            {
                out.push_back(err("unknown key: " + keyRaw));
                return;
            }
            listOne(key, it->second, out);
        }

        inline void applyOne(const std::string& keyRaw, const std::string& valRaw, std::vector<std::string>& out)
        {
            auto key = normalize(keyRaw);
            auto it = entries_.find(key);
            if (it == entries_.end())
            {
                out.push_back(err("unknown key: " + keyRaw));
                return;
            }

            auto& e = it->second;

            // Keep case for strings; only trim whitespace at ends
            std::string v = valRaw;
            size_t i = 0; while (i < v.size() && std::isspace((unsigned char)v[i])) i++;
            v.erase(0, i);
            if (!v.empty())
            {
                size_t j = v.size();
                while (j > 0 && std::isspace((unsigned char)v[j-1])) j--;
                v.erase(j);
            }
            std::string uv = unquote(v);

            switch (e.type)
            {
                case Type::kInt:
                {
                    char* endp = nullptr;
                    long vv = std::strtol(uv.c_str(), &endp, 10);
                    if (endp == uv.c_str() || *endp != '\0')
                    {
                        out.push_back(err(keyRaw + " expects int"));
                        return;
                    }
                    if (e.has_bounds)
                    {
                        vv = std::max<long>((long)e.min_v, std::min<long>((long)e.max_v, vv));
                    }
                    *(int*)e.ptr = (int)vv;
                    if (e.on_change) e.on_change();
                    out.push_back(ok(key + "=" + std::to_string((int)vv)));
                    break;
                }

                case Type::kFloat:
                {
                    char* endp = nullptr;
                    double vv = std::strtod(uv.c_str(), &endp);
                    if (endp == uv.c_str() || *endp != '\0')
                    {
                        out.push_back(err(keyRaw + " expects float"));
                        return;
                    }
                    if (e.has_bounds)
                    {
                        vv = std::max(e.min_v, std::min(e.max_v, vv));
                    }
                    *(float*)e.ptr = (float)vv;
                    if (e.on_change) e.on_change();

                    char buf[64];
                    std::snprintf(buf, sizeof(buf), "%s=%.6g", key.c_str(), vv);
                    out.push_back(ok(buf));
                    break;
                }

                case Type::kBool:
                {
                    bool b;
                    if (!parseBool(uv, b))
                    {
                        out.push_back(err(keyRaw + " expects bool (1/0,true/false,on/off)"));
                        return;
                    }
                    *(bool*)e.ptr = b;
                    if (e.on_change) e.on_change();
                    out.push_back(ok(key + "=" + std::string(b ? "true" : "false")));
                    break;
                }

                case Type::kString:
                {
                    *(std::string*)e.ptr = uv;
                    if (e.on_change) e.on_change();
                    out.push_back(ok(key + "=\"" + uv + "\""));
                    break;
                }
            }
        }
    };
    // ----------------- end KeyValueRegistry -----------------

    struct Hooks
    {
        std::function<void()>        onGainsChanged;
        std::function<void(bool)>    onEnablePrintChanged;
        std::function<void(int)>     onSetSpeedChanged;
        std::function<std::string()> onSave;
        std::function<std::string()> onLoad;
    };

public:
    // Bring up UART console once (safe to call multiple times)
    static inline void init(controlSettings& s,
                            const Hooks& hooks = Hooks{},
                            uart_port_t port = UART_NUM_0,
                            int baud = 115200)
    {
        static bool started = false;
        if (started) return;
        started = true;

        UartRouter::Config cfg;
        cfg.port       = port;
        cfg.baud       = baud;
        cfg.line_delim = '\n';

        router(cfg).begin();
        bindConsole(s, hooks);

        router().onLine([](const std::string& line)
        {
            auto responses = kv().applyLine(line);
            for (auto& ss : responses)
            {
                router().write(ss);
                router().write("\r\n");
            }
        });

        router().startTask();
        router().write("KV console ready. Try: Ki=0.5 enablePrint=1 setSpeed=500\r\n");
    }

    // Accessors
    static inline UartRouter& router()
    {
        static UartRouter* r = nullptr; // set by router(cfg)
        return *r;
    }

    static inline UartRouter& router(const UartRouter::Config& cfg)
    {
        static UartRouter r(cfg);
        static bool assigned = ([](UartRouter* rr){ static UartRouter*& slot = *([](){ static UartRouter* p=nullptr; return &p; })(); slot = rr; return true; })(&r);
        (void)assigned;
        return r;
    }

    static inline KeyValueRegistry& kv()
    {
        static KeyValueRegistry inst;
        return inst;
    }

    // Two app-level toggles exposed via CLI
    static inline bool& enablePrint()
    {
        static bool v = false;
        return v;
    }

    static inline int& setSpeed()
    {
        static int v = 0;
        return v;
    }

private:
    static inline void bindConsole(controlSettings& s, const Hooks& hooks)
    {
        // PID gains
        kv().bind("Kp", &s.Kp, -100.0f, 100.0f, [hooks]{ if (hooks.onGainsChanged) hooks.onGainsChanged(); });
        kv().bind("Ki", &s.Ki, -100.0f, 100.0f, [hooks]{ if (hooks.onGainsChanged) hooks.onGainsChanged(); });
        kv().bind("Kd", &s.Kd, -100.0f, 100.0f, [hooks]{ if (hooks.onGainsChanged) hooks.onGainsChanged(); });
        kv().bind("LPF_ALPHA", &s.LPF_ALPHA, 0.0f, 1.0f, [hooks]{ if (hooks.onGainsChanged) hooks.onGainsChanged(); });

        // Other fields (match your struct exactly)
        kv().bind("scaleA", &s.scaleA);
        kv().bind("scaleB", &s.scaleB);
        kv().bind("vA_full", &s.vA_full);
        kv().bind("vB_full", &s.vB_full);

        // needToCalibrate is int in your struct
        kv().bind("needToCalibrate", &s.needToCalibrate);

        // Expose Q (uint32_t) and restart_counter via int binding
        kv().bind("Q", (int*)&s.Q);
        kv().bind("restart_counter", (int*)&s.restart_counter);

        kv().bind("reservedForFutureUse_1", &s.reservedForFutureUse_1);
        kv().bind("reservedForFutureUse_2", &s.reservedForFutureUse_2);
        kv().bind("reservedForFutureUse_3", &s.reservedForFutureUse_3);
        kv().bind("reservedForFutureUse_4", &s.reservedForFutureUse_4);
        kv().bind("reservedForFutureUse_5", &s.reservedForFutureUse_5);
        kv().bind("reservedForFutureUse_6", &s.reservedForFutureUse_6);

        // App-level toggles
        kv().bind("enablePrint", &enablePrint(), [hooks]
        {
            if (hooks.onEnablePrintChanged) hooks.onEnablePrintChanged(enablePrint());
        });

        kv().bind("setSpeed", &setSpeed(), 0, 5000, [hooks]
        {
            if (hooks.onSetSpeedChanged) hooks.onSetSpeedChanged(setSpeed());
        });

        // Commands
        kv().setCommand("save", [hooks]() -> std::string
        {
            return hooks.onSave ? hooks.onSave() : std::string("OK (no save hook)");
        });

        kv().setCommand("load", [hooks]() -> std::string
        {
            return hooks.onLoad ? hooks.onLoad() : std::string("OK (no load hook)");
        });
    }
};
