/**
 * A graphical OSD overlay on top of the video.
 * It receives a ton of various data-points ("Facts") other parts of the system publish using
 * osd_publish_* and osd_add_* functions and uses this data to draw a graphical and/or textual
 * widgets on the screen.
 * The whole OSD is configured using config-file (JSON) which currently is basically a list of
 * widgets, their positions, additional options and "subscriptions" to the "Facts".
 *
 * OSD runs in a separate thread and receives all the facts via queue.
 *
 * We also have `ExternalSurfaceWidget` which is a bit special - it doesn't read any facts but
 * displays a surface that is provided via shm by external program. Right now it is used to display
 * MSP/Displayport OSD.
 */
#include <cmath>
#include <atomic>
extern "C" {
#include "drm.h"
#include "mavlink.h"
#include "menu.h"
#include "input.h"
}
#include "osd.h"
#include "osd.hpp"

#include <pthread.h>
#include <map>
#include <unordered_map>
#include <stack>
#include <vector>
#include <ranges>
#include <memory>
#include <variant>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <deque>
#include <cstdlib> //KILLME
#include <string>
#include <optional>
#include <regex>
#include <utility>
#include <filesystem>
#include <cairo.h>
#include <time.h>
#include <stdint.h>
#include <nlohmann/json.hpp>
#include "spdlog/spdlog.h"
#include <fmt/ranges.h>
#include "../lvgl/lvgl.h"
#include "osd_gl.hpp"

#ifdef BUILD_TESTS
#include <catch2/catch.hpp>
#endif

#define WFB_LINK_LOST 1
#define WFB_LINK_JAMMED 2

#define PATH_MAX	4096

using json = nlohmann::json;

int enable_osd = 0;
int osd_zpos = 2;
extern uint32_t refresh_frequency_ms;
extern uint32_t frames_received;
uint32_t stats_rx_bytes = 0;
struct timespec last_timestamp = {0, 0};
float rx_rate = 0;
int hours = 0, minutes = 0, seconds = 0, milliseconds = 0;
extern pthread_mutex_t video_mutex;
extern pthread_cond_t video_cond;
bool osd_update_ready = false;
bool menu_active = false;
bool gsmenu_enabled = false;
// Default = fully opaque menu (today's historical behavior when this
// setting isn't configured). Already in post-inversion-compensated form
// (the actual bg_opa value LVGL gets) -- see the inversion comment where
// the YAML value is parsed in main.cpp for why 0 here means opaque, not
// transparent.
int gsmenu_transparency = 0;
int gsmenu_error_timeout_ms = 4000; // auto-dismiss delay for the error toast; see show_error() in gsmenu/executor.c

// "NO SIGNAL" indicator for the multistream switcher (see
// StreamManager::is_active_stream_stale() / __DISPLAY_THREAD__ in main.cpp).
// Lives on lv_layer_top() like the gsmenu error toast -- always visible
// above whichever screen is active -- but only actually shown while the
// menu is closed (no_signal_label, lv_layer_top), since "stream gone" isn't
// relevant while the user is in settings.
static lv_obj_t * no_signal_label = nullptr;
std::atomic<bool> g_lvgl_ready{false}; // set true at the end of setup_lvgl(), once lv_init() has actually run
void set_no_signal_indicator(bool show) {
    if (!g_lvgl_ready.load()) return;
    lv_lock();
    bool show_now = show && !menu_active;
    if (show_now && !no_signal_label) {
        no_signal_label = lv_label_create(lv_layer_top());
        lv_obj_set_style_text_color(no_signal_label, lv_color_white(), LV_PART_MAIN);
        lv_label_set_text(no_signal_label, "NO SIGNAL");
        lv_obj_center(no_signal_label);
    }
    if (no_signal_label) {
        if (show_now) lv_obj_remove_flag(no_signal_label, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(no_signal_label, LV_OBJ_FLAG_HIDDEN);
    }
    lv_unlock();
}

OsdGl osd_gl;
extern bool enable_live_colortrans;
extern float live_colortrans_offset;
extern float live_colortrans_gain;

#include "frame_processor.h"
extern FrameProcessor *frame_proc;
extern bool dvr_osd;
extern bool display_reencode_osd;

osd_thread_params *p;

double getTimeInterval(struct timespec* timestamp, struct timespec* last_meansure_timestamp) {
  return (timestamp->tv_sec - last_meansure_timestamp->tv_sec) +
       (timestamp->tv_nsec - last_meansure_timestamp->tv_nsec) / 1000000000.;
}


//
// Evaluation of `convert` expressions on numerical facts
//

class ExpressionException : public std::exception {
public:
    enum ErrorType {
        MISMATCHED_PARENTHESES,
        DIVISION_BY_ZERO,
        UNKNOWN_OPERATOR,
        INVALID_EXPRESSION
    };

    ExpressionException(ErrorType type, const std::string& message)
        : type_(type), msg_(message) {}

    virtual const char* what() const noexcept override {
        return msg_.c_str();
    }

    ErrorType type() const { return type_; }

private:
    ErrorType type_;
    std::string msg_;
};

/**
 * This class can parse and evaluate math expressions with functions.
 * Supported tokens:
 * - '123', '12.34' - integer and float literals
 * - '+', '-', '/', '*' - arithmetic operators with standard precedence
 * - '(', ')' - grouping parentheses
 * - 'x' - the input variable passed to evaluate()
 * - 'name(a, b, ...)' - function calls; built-ins listed below; extensible via register_func()
 *
 * Always returns double. Functions: abs floor ceil round sqrt sin cos tan asin acos atan
 *                                   fmod min max pow atan2 clamp
 */
class ExpressionTree {
public:
    // --- Public function registration API ---

    struct FuncDef {
        int min_args;
        int max_args; // -1 = variadic
        std::function<double(const std::vector<double>&)> impl;
    };

    static std::unordered_map<std::string, FuncDef>& func_registry() {
        static std::unordered_map<std::string, FuncDef> r;
        return r;
    }

    static void register_func(std::string name, int min_args, int max_args,
                              std::function<double(const std::vector<double>&)> impl) {
        func_registry()[std::move(name)] = {min_args, max_args, std::move(impl)};
    }

private:
    static bool register_builtins() {
        using A = const std::vector<double>&;
        register_func("abs",   1, 1, [](A a){ return std::abs(a[0]); });
        register_func("floor", 1, 1, [](A a){ return std::floor(a[0]); });
        register_func("ceil",  1, 1, [](A a){ return std::ceil(a[0]); });
        register_func("round", 1, 1, [](A a){ return std::round(a[0]); });
        register_func("sqrt",  1, 1, [](A a){ return std::sqrt(a[0]); });
        register_func("sin",   1, 1, [](A a){ return std::sin(a[0]); });
        register_func("cos",   1, 1, [](A a){ return std::cos(a[0]); });
        register_func("tan",   1, 1, [](A a){ return std::tan(a[0]); });
        register_func("asin",  1, 1, [](A a){ return std::asin(a[0]); });
        register_func("acos",  1, 1, [](A a){ return std::acos(a[0]); });
        register_func("atan",  1, 1, [](A a){ return std::atan(a[0]); });
        register_func("min",   2, 2, [](A a){ return std::min(a[0], a[1]); });
        register_func("max",   2, 2, [](A a){ return std::max(a[0], a[1]); });
        register_func("pow",   2, 2, [](A a){ return std::pow(a[0], a[1]); });
        register_func("atan2", 2, 2, [](A a){ return std::atan2(a[0], a[1]); });
        register_func("clamp", 3, 3, [](A a){ return std::clamp(a[0], a[1], a[2]); });
        register_func("mod",   2, 2, [](A a){ return std::fmod(a[0], a[1]); });
        register_func("deg",   1, 1, [](A a){ return a[0] * 57.29577951308232; });
        register_func("norm",  2, 2, [](A a){ return std::fmod(a[0] + a[1], a[1]); });
        return true;
    }
    static inline bool _builtins_registered = register_builtins();

public:
	ExpressionTree() : root(nullptr) {}
	ExpressionTree(const std::string &expression) : root(nullptr) {
		parse(expression);
	}
    ExpressionTree(ExpressionTree&& other) noexcept : root(std::move(other.root)) {}
    ExpressionTree(const ExpressionTree& other) {
        root = other.root ? std::make_unique<Node>(*other.root) : nullptr;
    }

	std::vector<std::string> tokenize(const std::string& expression) {
		std::vector<std::string> tokens;
		std::string currentToken;

		for (size_t i = 0; i < expression.length(); ++i) {
			char c = expression[i];

			if (std::isdigit(c) || c == '.') {
				currentToken += c;
			} else if (std::isalpha(c) || c == '_') {
                // identifier: variable 'x' or function name
				currentToken += c;
			} else if (c == '+' || c == '-' || c == '*' || c == '/' ||
                       c == '(' || c == ')' || c == ',') {
				if (!currentToken.empty()) {
					tokens.push_back(currentToken);
					currentToken.clear();
				}
				tokens.push_back(std::string(1, c));
			} else if (std::isspace(c)) {
				if (!currentToken.empty()) {
					tokens.push_back(currentToken);
					currentToken.clear();
				}
			} else {
				throw ExpressionException(
    					  ExpressionException::INVALID_EXPRESSION,
						  "Unexpected symbol at " + std::to_string(i) + ": '" + c + "'");
			}
		}
		if (!currentToken.empty()) {
			tokens.push_back(currentToken);
		}
		return tokens;
	}

    void parseTokens(const std::vector<std::string>& tokens) {
        // Use unique_ptr wrappers to avoid leaks on exception paths.
        // Raw Node* aliases into the same memory — ownership stays in the vectors.
        std::vector<std::unique_ptr<Node>> output;
        std::vector<std::unique_ptr<Node>> operators;
        std::stack<bool> func_call;  // true if the matching '(' is a function call
        std::stack<int>  arg_count;  // argument count for the current function call

        auto popUntilParen = [&]() {
            while (!operators.empty() && operators.back()->op != '(') {
                processOperatorU(output, operators);
            }
            if (operators.empty())
                throw ExpressionException(ExpressionException::MISMATCHED_PARENTHESES,
                                          "Mismatched parentheses");
        };

        for (const auto& token : tokens) {
            if (isNumber(token)) {
                output.push_back(std::make_unique<Node>(std::stod(token)));
            } else if (token == "x") {
                output.push_back(std::make_unique<Node>('x'));
            } else if (func_registry().count(token)) {
                auto fn = std::make_unique<Node>('@');
                fn->func = token;
                operators.push_back(std::move(fn));
            } else if (token == "(") {
                bool is_func = !operators.empty() && operators.back()->op == '@';
                func_call.push(is_func);
                if (is_func) arg_count.push(1);
                operators.push_back(std::make_unique<Node>('('));
            } else if (token == ",") {
                popUntilParen();
                if (arg_count.empty())
                    throw ExpressionException(ExpressionException::INVALID_EXPRESSION,
                                              "Unexpected ','");
                arg_count.top()++;
            } else if (token == ")") {
                popUntilParen();
                operators.pop_back(); // remove '('
                bool was_func = !func_call.empty() && func_call.top();
                if (!func_call.empty()) func_call.pop();

                if (was_func) {
                    int n = arg_count.top(); arg_count.pop();
                    if (operators.empty() || operators.back()->op != '@')
                        throw ExpressionException(ExpressionException::INVALID_EXPRESSION,
                                                  "Function node missing");
                    auto fn = std::move(operators.back()); operators.pop_back();
                    auto it = func_registry().find(fn->func);
                    if (it == func_registry().end())
                        throw ExpressionException(ExpressionException::UNKNOWN_OPERATOR,
                                                  "Unknown function: " + fn->func);
                    auto& def = it->second;
                    if (n < def.min_args || (def.max_args != -1 && n > def.max_args))
                        throw ExpressionException(ExpressionException::INVALID_EXPRESSION,
                                                  "Wrong arg count for " + fn->func);
                    fn->args.resize(n);
                    size_t base = output.size() - n;
                    for (int i = 0; i < n; ++i)
                        fn->args[i] = std::move(output[base + i]);
                    output.resize(base);
                    output.push_back(std::move(fn));
                }
            } else {
                while (!operators.empty() &&
                       operators.back()->op != '(' &&
                       operators.back()->op != '@' &&
                       precedence(operators.back()->op) >= precedence(token[0])) {
                    processOperatorU(output, operators);
                }
                operators.push_back(std::make_unique<Node>(token[0]));
            }
        }

        while (!operators.empty()) {
            processOperatorU(output, operators);
        }

        if (output.empty())
            throw ExpressionException(ExpressionException::INVALID_EXPRESSION,
                                      "Empty expression");
        root = std::move(output.back());
    }

    void parse(const std::string &expression) {
		parseTokens(tokenize(expression));
	}

    double evaluate(double xValue) {
        return evaluateNode(root.get(), xValue);
    }

	std::string treeToString() const {
		if (!root.get()) return "null";
		return nodeToString(root.get());
	}

private:
    struct Node {
        char op;
        double value;
        std::string func;                        // populated when op=='@'
        std::vector<std::unique_ptr<Node>> args; // function arguments, when op=='@'
        std::unique_ptr<Node> left, right;       // arithmetic children

        Node(double val) : op(0), value(val) {}
        Node(char operation) : op(operation), value(0) {}
        Node(const Node& other)
            : op(other.op), value(other.value), func(other.func),
              left(other.left   ? std::make_unique<Node>(*other.left)  : nullptr),
              right(other.right ? std::make_unique<Node>(*other.right) : nullptr) {
            for (auto& a : other.args)
                args.push_back(std::make_unique<Node>(*a));
        }
        Node(Node&& other) noexcept
            : op(other.op), value(other.value), func(std::move(other.func)),
              args(std::move(other.args)),
              left(std::move(other.left)), right(std::move(other.right)) {}
	};

    std::unique_ptr<Node> root;

    bool isNumber(const std::string& s) {
        char* p;
        std::strtod(s.c_str(), &p);
        return *p == 0;
    }

    int precedence(char op) {
        if (op == '+' || op == '-') return 1;
        if (op == '*' || op == '/') return 2;
        return 0;
    }

    void processOperatorU(std::vector<std::unique_ptr<Node>>& output,
                          std::vector<std::unique_ptr<Node>>& operators) {
        auto right = std::move(output.back()); output.pop_back();
        auto left  = std::move(output.back()); output.pop_back();
        auto op    = std::move(operators.back()); operators.pop_back();
        op->left  = std::move(left);
        op->right = std::move(right);
        output.push_back(std::move(op));
    }

    double evaluateNode(Node* node, double xValue) const {
        if (!node) return 0;
        if (node->op == 0)   return node->value;
        if (node->op == 'x') return xValue;
        if (node->op == '@') {
            auto& def = func_registry().at(node->func);
            std::vector<double> vals;
            vals.reserve(node->args.size());
            for (auto& a : node->args)
                vals.push_back(evaluateNode(a.get(), xValue));
            return def.impl(vals);
        }
        double leftValue  = evaluateNode(node->left.get(),  xValue);
        double rightValue = evaluateNode(node->right.get(), xValue);
        switch (node->op) {
            case '+': return leftValue + rightValue;
            case '-': return leftValue - rightValue;
            case '*': return leftValue * rightValue;
            case '/':
                if (rightValue == 0)
                    throw ExpressionException(ExpressionException::DIVISION_BY_ZERO,
                                              "Division by zero");
                return leftValue / rightValue;
            default:
                throw ExpressionException(ExpressionException::UNKNOWN_OPERATOR,
                                          "Unknown operator");
        }
    }

	std::string nodeToString(Node* node) const {
		if (!node) return "null";
		std::ostringstream oss;
        if (node->op == '@') {
            oss << node->func << "(";
            for (size_t i = 0; i < node->args.size(); ++i) {
                if (i) oss << ", ";
                oss << nodeToString(node->args[i].get());
            }
            oss << ")";
        } else {
            oss << "Node(op=" << (node->op != 0 ? std::string(1, node->op) : std::to_string(node->value))
                << ", left=" << nodeToString(node->left.get())
                << ", right=" << nodeToString(node->right.get()) << ")";
        }
		return oss.str();
	}

};

//
// Facts
//

typedef std::map<std::string, std::string> FactTags;


class FactMeta {
public:
	FactMeta(): name(""), tags({}) {};
	FactMeta(std::string name): name(name), tags({}) {};
	FactMeta(std::string name, FactTags tags): name(name), tags(tags) {};
	

	std::string getName() const { return name; }
	FactTags getTags() const { return tags; }

private:
	std::string name;
	FactTags tags;
};


class Fact {
public:
	enum Type {
		T_UNDEF,
		T_BOOL,
		T_INT,
		T_UINT,
		T_DOUBLE,
		T_STRING
	};

	Fact(): meta(FactMeta("", {})), type(T_UNDEF) {};
	Fact(FactMeta meta, bool val): meta(meta), value(val), type(T_BOOL) {};
	Fact(FactMeta meta, long val): meta(meta), value(val), type(T_INT) {};
	Fact(FactMeta meta, ulong val): meta(meta), value(val), type(T_UINT) {};
	Fact(FactMeta meta, double val): meta(meta), value(val), type(T_DOUBLE) {};
	Fact(FactMeta meta, std::string val): meta(meta), value(val), type(T_STRING) {};

	bool isDefined() const {
		return type != T_UNDEF;
	}

    operator bool() {
        switch (type) {
        case T_BOOL:
            return getBoolValue();
        case T_UINT:
            return getUintValue() != 0;
        case T_INT:
            return getIntValue() != 0;
        case T_DOUBLE:
            return getDoubleValue() != 0.0;
        case T_STRING:
            return getStrValue() != "";
        }
    }

    operator long() {
        switch (type) {
        case T_BOOL:
            return getBoolValue() ? 1 : 0;
        case T_UINT:
            return (long)getUintValue();
        case T_INT:
            return getIntValue();
        case T_DOUBLE:
            return round(getDoubleValue());
        }
    }

    operator ulong() {
        switch (type) {
        case T_BOOL:
            return getBoolValue() ? 1 : 0;
        case T_UINT:
            return getUintValue();
        case T_INT:
            return (ulong)getIntValue();
        case T_DOUBLE:
            return round(getDoubleValue());
        }
    }

    operator double() {
        switch (type) {
        case T_BOOL:
            return getBoolValue() ? 1.0 : 0.0;
        case T_UINT:
            return getUintValue() * 1.0;
        case T_INT:
            return getIntValue() * 1.0;
        case T_DOUBLE:
            return getDoubleValue();
        }
    }

	// TODO: try to cast instead of crash
	bool getBoolValue() const {
		assertType(T_BOOL);
		return std::get<bool>(value);
	}

	long getIntValue() const {
		assertType(T_INT);
		return std::get<long>(value);
	}

	ulong getUintValue() const {
		assertType(T_UINT);
		return std::get<ulong>(value);
	}

	double getDoubleValue() const {
		assertType(T_DOUBLE);
		return std::get<double>(value);
	}

	std::string getStrValue() const {
		assertType(T_STRING);
		return std::get<std::string>(value);
	}

	std::string getTypeName() const {
		return typeName(type);
	}

	Type getType() const {
		return type;
	}

	std::string getName() const {
		return meta.getName();
	}

	FactTags getTags() const {
		return meta.getTags();
	}

	std::string asString() const {
		switch(type) {
		case T_UNDEF:
			return "(undefined)";
		case T_BOOL:
			if (getBoolValue()) {
				return "true";
			} else {
				return "false";
			};
		case T_INT:
			return std::to_string(getIntValue());
		case T_UINT:
			return std::to_string(getUintValue());
		case T_DOUBLE:
			return std::to_string(getDoubleValue());
		case T_STRING:
			return getStrValue();
		}
		return "(unknown)";
	}

	std::string asVerboseString() const {
		std::ostringstream oss;
		if (!isDefined()) {
			oss << "undef";
		} else {
			oss << getName() << " (" << getTypeName() << ") {";
			for (const auto &tag : getTags()) {
				oss << tag.first << "=>" << tag.second << ", ";
			}
			oss << "} = " << asString();
		}
		return oss.str();
	}
	
private:
	Type type = T_UNDEF;
	std::string typeName(Type t) const {
		switch(t) {
		case T_UNDEF:
			return "UNDEF";
		case T_BOOL:
			return "BOOL";
		case T_INT:
			return "INT";
		case T_UINT:
			return "UINT";
		case T_DOUBLE:
			return "DOUBLE";
		case T_STRING:
			return "STRING";
		}
		return "UNKNOWN";
	}

	void assertType(Type t) const {
		if (t != type) {
			throw std::runtime_error(
				std::string("'") + asVerboseString() + "': requested type of " +
				typeName(t) + ", but the actual type is " + typeName(type));
		}
	}
	FactMeta meta;
	// TODO: timestamp
	std::variant<
		bool,
		long,
		ulong,
		double,
		std::string
		> value;
};



class FactMatcher {
public:
	FactMatcher(std::string name, FactTags tags, std::string &convert_str)
		: name(name), tags(tags), converter(ExpressionTree(convert_str)) {};
	// FactMatcher(std::string name, FactTags tags, ExpressionTree converter)
	// 	: name(name), tags(tags), converter(std::move(converter)) {};
	FactMatcher(std::string name, FactTags tags): name(name), tags(tags) {};
	FactMatcher(std::string name): name(name), tags({}) {};

	// Returns a copy that matches the same fact but applies no conversion.
	// Used when registering container child matchers at the Osd routing level:
	// the container itself holds the full matchers with convert expressions.
	FactMatcher routing_copy() const { return FactMatcher(name, tags); }


	/**
	 * Returns true if names are equal and all match_tags are defined and have equal value
	 */
	bool matches(Fact fact) {
		if(fact.getName() != name) return false;
		FactTags fact_tags = fact.getTags();
		
		for (const auto& [key, match_value] : tags) {
			if (auto value = fact_tags.find(key); value != tags.end()) {
				if (value->second != match_value) return false;
			} else {
				return false;
			}
		}
		return true;
	}

	/**
	 * Applies 'convert' expression to the fact's value.
	 * On success, a new fact is returned with `.converted` appended to its name and value converted
	 */
	Fact convert(Fact fact_in) {
		if (converter.has_value()) {
			std::string name = fact_in.getName();
			FactTags tags = fact_in.getTags();
			FactMeta new_meta(name + ".converted", tags);
			double val = 0.0;

			switch (fact_in.getType()) {
			case Fact::T_BOOL:
				val = 0.0;
				if(fact_in.getBoolValue()) {
					val = 1.0;
				}
				break;
			case Fact::T_INT:
				val = static_cast<double>(fact_in.getIntValue());
				break;
			case Fact::T_UINT:
				val = static_cast<double>(fact_in.getUintValue());
				break;
			case Fact::T_DOUBLE:
				val = fact_in.getDoubleValue();
				break;
			default:
				spdlog::warn("Attempt to apply 'convert' to unexpected datatype. Ignoring");
				return fact_in;
			}
			return Fact(new_meta, converter->evaluate(val));
		} else {
			return fact_in;
		}
	}
	
	std::string name;
	FactTags tags;
protected:
	std::optional<ExpressionTree> converter = std::nullopt;
};


struct Bucket {
	long long timestamp;
	long sum;
	int count;
	long min_value;
	long max_value;

	Bucket(long long ts, long value)
		: timestamp(ts), sum(value), count(1), min_value(value), max_value(value) {}
};

// Struct to hold extended statistics
struct Stats {
	long min;
	long max;
	double average;
	long sum;
	int count;

	Stats(long min_value, long max_value, double avg, long total_sum, int total_count)
		: min(min_value), max(max_value), average(avg), sum(total_sum), count(total_count) {}
};

/**
 * Calculates the running average/rate-per-second/min/max over a sliding time window.
 * @param window_size_ms the size of the sliding window in milliseconds
 * @param bucket_size_ms the size of the bucket; structure uses amount of memory
 *		  of O(window_size_ms / bucket_size_ms), however large bucket size decreases the precision.
 * NOTE: the code was mostly generated by ChatGPT
 */
class RunningAverage {
public:
	RunningAverage(int window_size_ms, int bucket_size_ms)
		: window_size(window_size_ms), bucket_size(bucket_size_ms), sum(0), count(0) {
		assert(window_size_ms >= bucket_size_ms);
	}

	long add(long value) {
		auto now = std::chrono::steady_clock::now();
		auto current_time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

		// Remove outdated buckets
		while (!buckets.empty() && (current_time - buckets.front().timestamp > window_size)) {
			sum -= buckets.front().sum;
			count -= buckets.front().count;
			buckets.pop_front();
		}

		// Add the value to the current bucket
		if (!buckets.empty() && (current_time - buckets.back().timestamp < bucket_size)) {
			buckets.back().sum += value;
			buckets.back().count += 1;
			buckets.back().min_value = std::min(buckets.back().min_value, value);
			buckets.back().max_value = std::max(buckets.back().max_value, value);
		} else {
			buckets.emplace_back(current_time, value);
		}

		// Update the running sum and count
		sum += value;
		count++;

		return count > 0 ? sum / count : 0;
	}

	double average_over_last_ms(uint last_ms) const {
		long min = std::numeric_limits<long>::max();
		long max = std::numeric_limits<long>::min();
		long last_sum;
		int last_count;
		calculate_stats_in_window(last_ms, last_sum, last_count, min, max);

		return last_count > 0 ? static_cast<double>(last_sum) / last_count : 0.0;
	}

	double rate_per_second_over_last_ms(uint last_ms) const {
		long min = std::numeric_limits<long>::max();
		long max = std::numeric_limits<long>::min();
		long last_sum;
		int last_count;
		calculate_stats_in_window(last_ms, last_sum, last_count, min, max);

		double elapsed_seconds = static_cast<double>(last_ms) / 1000.0;
		return elapsed_seconds > 0 ? static_cast<double>(last_sum) / elapsed_seconds : 0.0;
	}

	void get_stats_over_last_ms(uint last_ms, long& min, long& max, double& average) const {
		long last_sum;
		int last_count;

		min = std::numeric_limits<long>::max();
		max = std::numeric_limits<long>::min();

		calculate_stats_in_window(last_ms, last_sum, last_count, min, max);

		average = last_count > 0 ? static_cast<double>(last_sum) / last_count : 0.0;
	}

	// New method to return Stats struct with sum and count
	Stats get_stats_over_last_ms_result(uint last_ms) const {
		long min = std::numeric_limits<long>::max();
		long max = std::numeric_limits<long>::min();
		long last_sum = 0;
		int last_count = 0;

		calculate_stats_in_window(last_ms, last_sum, last_count, min, max);

		double average = last_count > 0 ? static_cast<double>(last_sum) / last_count : 0.0;
		return Stats(min, max, average, last_sum, last_count);
	}

	std::vector<long> get_bucket_sums() const {
		std::vector<long> sums;
		sums.reserve(buckets.size());
		for (const auto& bucket : buckets) {
			sums.push_back(bucket.sum);
		}
		return sums;
	}

	std::vector<Stats> get_bucket_stats() const {
		std::vector<Stats> stats;
		stats.reserve(buckets.size());
		for (const auto& bucket : buckets) {
			double average = bucket.count > 0 ? static_cast<double>(bucket.sum) / bucket.count : 0.0;
			stats.push_back(Stats(bucket.min_value, bucket.max_value, average, bucket.sum, bucket.count));
		}
		return stats;
	}

private:
	void calculate_stats_in_window(uint last_ms, long& sum_out, int& count_out, long& min_out, long& max_out) const {
		auto now = std::chrono::steady_clock::now();
		auto current_time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

		sum_out = 0;
		count_out = 0;

		for (auto it = buckets.rbegin(); it != buckets.rend(); ++it) {
			if (current_time - it->timestamp <= last_ms) {
				sum_out += it->sum;
				count_out += it->count;
				min_out = std::min(min_out, it->min_value);
				max_out = std::max(max_out, it->max_value);
			} else {
				break;	// Exit loop once we're outside the time window
			}
		}
	}

	int window_size;
	int bucket_size;
	std::deque<Bucket> buckets;
	long sum;
	int count;
};

//
// Widgets
//

class Widget {
public:
	Widget(int pos_x, int pos_y): pos_x(pos_x), pos_y(pos_y) {};
	Widget(int pos_x, int pos_y, uint num_args): pos_x(pos_x), pos_y(pos_y) {
		for (auto i=0; i < num_args; i++) {
			args.push_back(Fact());
		}
	};

	virtual void draw(cairo_t *cr) {};

	virtual void setFact(uint idx, Fact fact) {
        if (idx >= args.size()) throw std::out_of_range("setFact index out of range");
        args[idx] = fact;
	}

	int x(cairo_t *cr) {
		cairo_surface_t *target = cairo_get_target(cr);
		int w = cairo_image_surface_get_width(target);
		//int h = cairo_image_surface_get_height(target);
		return (w + pos_x) % w;
	}
	int y(cairo_t *cr) {
		cairo_surface_t *target = cairo_get_target(cr);
		//int w = cairo_image_surface_get_width(target);
		int h = cairo_image_surface_get_height(target);
		return (h + pos_y) % h;
	}
	std::pair<int, int> xy(cairo_t *cr) {
		cairo_surface_t *target = cairo_get_target(cr);
		int w = cairo_image_surface_get_width(target);
		int h = cairo_image_surface_get_height(target);
		int rx = center_x_ ? (w - own_width_) / 2 : (w + pos_x) % w;
		return std::pair(rx, (h + pos_y) % h);
	}

	void set_center_x(int widget_width) { center_x_ = true; own_width_ = widget_width; }

protected:
	int pos_x, pos_y;
	bool center_x_ = false;
	int own_width_ = 0;
	std::vector<Fact> args;
};


class TextWidget: public Widget {
public:
	TextWidget(int pos_x, int pos_y, std::string text): Widget(pos_x, pos_y), text(text) {};

	virtual void draw(cairo_t *cr) {
		auto [x, y] = xy(cr);
		cairo_set_source_rgba(cr, 255.0, 255.0, 255.0, 1);
		cairo_move_to(cr, x, y);
		cairo_show_text(cr, text.c_str());
	}
protected:
	std::string text;
};


class IconTextWidget: public Widget {
public:
	IconTextWidget(int pos_x, int pos_y, cairo_surface_t *icon, std::string text):
		Widget(pos_x, pos_y), text(text), icon(icon) {};

	virtual void draw(cairo_t *cr) {
		auto [x, y] = xy(cr);
		cairo_set_source_surface(cr, icon, x, y - 20);
		cairo_paint(cr);
		cairo_set_source_rgba(cr, 255.0, 255.0, 255.0, 1);
		cairo_move_to(cr, x + 40, y);
		cairo_show_text(cr, text.c_str());
	}

protected:
	std::string text;
	cairo_surface_t *icon;
};


class TplTextWidget: public Widget {
public:
    TplTextWidget(int pos_x, int pos_y, std::string tpl, uint num_args):
        Widget(pos_x, pos_y, num_args), tpl(tpl), num_args(num_args) {
        _tokens = tokenize(tpl);
    };

    virtual void draw(cairo_t *cr) {
        auto [x, y] = xy(cr);
        std::unique_ptr<std::string> msg = render_tpl();
        cairo_set_source_rgba(cr, 255.0, 255.0, 255.0, 1);
        cairo_move_to(cr, x, y);
        cairo_show_text(cr, msg->c_str());
    }

    std::unique_ptr<std::string> render_tpl() {
        return render_tokens(_tokens, args);
    }

    uint default_precision = 2;

protected:
    enum class TokenType {
        Literal,
        Boolean,
        Int,
        Uint,
        Float,
        String
    };

    struct Token {
        TokenType type;
        std::optional<std::string> value; // Used to hold literal
        uint precision;    // Precision for float placeholders if applicable
        bool show_sign;    // %+f — always print sign for positive values

        Token(TokenType t, std::string v) // literal
            : type(t), value(std::move(v)), precision(0), show_sign(false) {}
        Token(TokenType t, uint p, bool sign = false) // float
            : type(t), value(std::nullopt), precision(p), show_sign(sign) {}
        Token(TokenType t) // other
            : type(t), value(std::nullopt), precision(0), show_sign(false) {}
    };

    std::unique_ptr<std::string> render_tpl(const std::string& tpl, const std::vector<Fact>& facts) {
        auto tokens = tokenize(tpl);
        return render_tokens(tokens, facts);
    }

    std::unique_ptr<std::string> render_tokens(const std::vector<Token>& tokens,
                                               const std::vector<Fact>& facts) {
        std::ostringstream msg;
        size_t fact_i = 0; // To track the current index in the facts vector

        for (const Token& token : tokens) {
            if (token.type == TokenType::Literal) {
                msg << *token.value; // Append literal directly, dereference std::optional
            } else {
                // Check if we have enough facts and if the current fact is defined
                if (fact_i >= facts.size() || !facts[fact_i].isDefined()) {
                    msg << "--"; // Append '--' for undefined/stale facts (standard FPV OSD convention)
                } else {
                    switch (token.type) {
                    case TokenType::Boolean:
                        msg << (facts[fact_i].getBoolValue() ? 't' : 'f');
                        break;
                    case TokenType::Int:
                        msg << facts[fact_i].getIntValue();
                        break;
                    case TokenType::Uint:
                        msg << facts[fact_i].getUintValue();
                        break;
                    case TokenType::Float: {
                        double v = facts[fact_i].getDoubleValue();
                        if (token.show_sign && v >= 0.0) msg << '+';
                        msg << std::fixed << std::setprecision(token.precision) << v;
                        break;
                    }
                    case TokenType::String:
                        msg << facts[fact_i].asString();
                        break;
                    }
                }
                fact_i++; // Move to the next fact for the next placeholder
            }
        }
        return std::make_unique<std::string>(msg.str());
    }

    std::vector<Token> tokenize(const std::string& tpl) {
        std::vector<Token> tokens;
        // 'd' must be in this character class too -- the switch below already
        // treats %d as a synonym for %i (both -> TokenType::Int), but if the
        // tokenizing regex doesn't recognize %d as a placeholder at all, it
        // gets swallowed into a literal text token instead, silently
        // shifting every subsequent placeholder onto the wrong fact index
        // (e.g. an %i/%d-bound INT fact ends up read by the next %u token,
        // which calls getUintValue() on it and crashes).
        std::regex token_regex(R"(%%|%[bisud]|%\+?(\.\d+)?f|[^%]+)"); // Match placeholders and literals
        std::sregex_iterator iter(tpl.begin(), tpl.end(), token_regex);
        std::sregex_iterator end;

        while (iter != end) {
            std::string match = iter->str();
            if (match == "%%") {
                tokens.emplace_back(TokenType::Literal, "%");
            } else if (match[0] == '%') {
                if (match.size() == 2) { // Simple placeholder like %b, %i, %u, %s, %f
                    if (match[1] == 'b') {
                        tokens.emplace_back(TokenType::Boolean);
                    } else if (match[1] == 'i' || match[1] == 'd') {
                        tokens.emplace_back(TokenType::Int);
                    } else if (match[1] == 'u') {
                        tokens.emplace_back(TokenType::Uint);
                    } else if (match[1] == 's') {
                        tokens.emplace_back(TokenType::String);
                    } else if (match[1] == 'f') {
                        tokens.emplace_back(TokenType::Float, default_precision);
                    }
                } else if (match.back() == 'f') { // Float with optional sign flag and/or precision
                    bool show_sign = (match.size() > 2 && match[1] == '+');
                    uint precision = default_precision;
                    auto dot_pos = match.find('.');
                    if (dot_pos != std::string::npos) {
                        precision = std::stoi(match.substr(dot_pos + 1, match.size() - dot_pos - 2));
                    }
                    tokens.emplace_back(TokenType::Float, precision, show_sign);
                }
            } else {
                tokens.emplace_back(TokenType::Literal, match); // Accumulate literal
            }
            ++iter;
        }

        return tokens;
    }

    std::string tpl;
    std::vector<Token> _tokens;
    uint num_args;
};


class IconTplTextWidget: public TplTextWidget {
public:
	IconTplTextWidget(int pos_x, int pos_y, cairo_surface_t *icon, std::string tpl, uint num_args):
		TplTextWidget(pos_x, pos_y, tpl, num_args), icon(icon) {};

	virtual void draw(cairo_t *cr) {
		auto [x, y] = xy(cr);
		std::unique_ptr<std::string> msg = render_tpl();
		cairo_set_source_surface(cr, icon, x, y - 20);
		cairo_paint(cr);
		cairo_set_source_rgba(cr, 255.0, 255.0, 255.0, 1);
		cairo_move_to(cr, x + 40, y);
		cairo_show_text(cr, msg->c_str());
	}

protected:
	cairo_surface_t *icon;
};

class BoxWidget: public Widget {
public:
	BoxWidget(int pos_x, int pos_y, uint w, uint h, double r, double g, double b, double a):
		Widget(pos_x, pos_y), w(w), h(h), r(r), g(g), b(b), a(a) {};

	virtual void draw(cairo_t *cr) {
		auto [x, y] = xy(cr);
		cairo_set_source_rgba(cr, r, g, b, a);
		cairo_rectangle(cr, x, y, w, h);
		cairo_fill(cr);
	}

private:
	uint w, h;
	double r, g, b, a;
};

class BoxWidgetContainer : public Widget {
public:
	BoxWidgetContainer(int pos_x, int pos_y, uint w, uint h, double r, double g, double b, double a)
		: Widget(pos_x, pos_y), w(w), h(h), r(r), g(g), b(b), a(a) {}

	~BoxWidgetContainer() {
		for (auto* c : children) delete c;
	}

	void addChild(Widget* child, std::vector<FactMatcher> child_fact_matchers) {
		children.push_back(child);
		uint arg_idx = 0;
		for (auto& m : child_fact_matchers)
			child_matchers.push_back({m, child, arg_idx++});
	}

	void setFact(uint /*idx*/, Fact fact) override {
		for (auto& [matcher, child, arg_idx] : child_matchers) {
			if (matcher.matches(fact)) {
				child->setFact(arg_idx, matcher.convert(fact));
			}
		}
	}

	void draw(cairo_t* cr) override {
		auto [cx, cy] = xy(cr);
		cairo_set_source_rgba(cr, r, g, b, a);
		cairo_rectangle(cr, cx, cy, w, h);
		cairo_fill(cr);
		cairo_save(cr);
		cairo_translate(cr, cx, cy);
		for (auto* child : children) {
			try { child->draw(cr); }
			catch (const std::exception& e) {
				spdlog::warn("BoxWidgetContainer child draw error: {}", e.what());
			}
		}
		cairo_restore(cr);
	}

private:
	uint w, h;
	double r, g, b, a;
	std::vector<Widget*> children;
	std::vector<std::tuple<FactMatcher, Widget*, uint>> child_matchers;
};

class BarChartWidget: public Widget {
public:
	enum StatsField {
		STATS_MIN,
		STATS_MAX,
		STATS_SUM,
		STATS_COUNT,
		STATS_AVG
	};

	BarChartWidget(int pos_x, int pos_y, uint w, uint h, uint window_s, uint num_buckets, BarChartWidget::StatsField stats_field):
		Widget(pos_x, pos_y, 0), w(w), h(h), window_ms(window_s * 1000), num_buckets(num_buckets), stats_field(stats_field),
		stats(window_s * 1000, window_s * 1000 / num_buckets) {};

	virtual void setFact(uint idx, Fact fact) {
		if (idx != 0) { spdlog::error("BarChartWidget: unexpected setFact idx {}", idx); return; }
		switch (fact.getType()) {
		case Fact::T_INT:
			stats.add(fact.getIntValue());
			break;
		case Fact::T_UINT:
			stats.add(static_cast<long>(fact.getUintValue()));
		}
	}

	virtual void draw(cairo_t *cr) {
		auto [x, y] = xy(cr);
		// box
		cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, 0.4);
		cairo_rectangle(cr, x, y, w, h);
		cairo_fill(cr);

		std::vector<Stats> all_stats = stats.get_bucket_stats();
		if (all_stats.size() < 3) {
			SPDLOG_DEBUG("Can't draw bar chart - too few values");
			return;
		}
		all_stats.pop_back(); // drop last bucket, because it is usually still not full
		std::vector<double> stats = select_stats(all_stats);
		double min = *std::min_element(stats.begin(), stats.end());
		double max = *std::max_element(stats.begin(), stats.end());

		// legend
		cairo_set_source_rgba(cr, 255.0, 255.0, 255.0, 1);
		cairo_move_to(cr, x + 2, y + 15);
		cairo_show_text(cr, shorten(max).c_str());

		cairo_move_to(cr, x + 2, y + h);
		cairo_show_text(cr, shorten(min).c_str());

		// bars
		cairo_set_source_rgba(cr, 200.0, 200.0, 200.0, 0.8);

		double scale = max - min;
		SPDLOG_TRACE("Scale: {}, min {}, max {}", scale, min, max);
		uint legend_w = 65;
		uint chart_w = w - legend_w;

		uint bar_pad = 4;
		uint bar_w = (chart_w - (bar_pad * num_buckets)) / num_buckets;
		uint bar_x = x + legend_w;
		SPDLOG_TRACE(
					 "chart_w {} bar_w {}, bar_x {}",
					 chart_w, bar_w, bar_x
                    );
		for (auto val : stats) {
			double normalized = val - min;
			double bar_h = -1.0 * (normalized * (h - 10)) / scale;
			// h -> max-min
			// ? -> normalized
			SPDLOG_TRACE("val {}, cairo_rectangle(cr, {}, {}, {}, {})",
						 val, bar_x, y + h, bar_w, bar_h);
			cairo_rectangle(cr, bar_x, y + h, bar_w, bar_h - 2);
			cairo_fill(cr);
			bar_x += (bar_pad + bar_w);
		}
	}

private:
	/**
	 * function that takes ulong and returns string with short form of the number:
	 * up to 3 digits and "giga" / "mega" / "kilo" suffix
	 * made by ChatGPT
	 */
	std::string shorten(long num) {
		double value = num;
		std::string suffix;

		if (num >= 1'000'000'000) {  // Giga
			value = num / 1'000'000'000.0;
			suffix = "G";
		} else if (num >= 1'000'000) {  // Mega
			value = num / 1'000'000.0;
			suffix = "M";
		} else if (num >= 1'000) {  // Kilo
			value = num / 1'000.0;
			suffix = "K";
		} else {
			suffix = "";  // No suffix needed
		}

		// Format to 3 significant digits
		std::ostringstream oss;
		oss << std::fixed << std::setprecision(3 - static_cast<int>(std::log10(value) + 1)) << value;
		return oss.str() + " " + suffix;
	}
	std::vector<double> select_stats(std::vector<Stats> stats) {
		std::vector<double> res;
		res.reserve(stats.size());
		for (auto stat : stats) {
			switch(stats_field) {
			case STATS_MIN:
				res.push_back(static_cast<double>(stat.min));
				break;
			case STATS_MAX:
				res.push_back(static_cast<double>(stat.max));
				break;
			case STATS_SUM:
				res.push_back(static_cast<double>(stat.sum));
				break;
			case STATS_COUNT:
				res.push_back(static_cast<double>(stat.count));
				break;
			case STATS_AVG:
				res.push_back(stat.average);
				break;
			}
		}
		return res;
	}
	uint w, h;
	uint window_ms, num_buckets;
	StatsField stats_field = STATS_SUM;
	RunningAverage stats;
};

/**
 * Displays text facts for a period of time, stacking them one after another; fading-out opacity.
 * Convenient for warnings, custom messages and pop-ups.
 *
 * @param timeout_ms stop displaying the fact after this many milliseconds since it was received
 */
class PopupWidget: public Widget {
public:
	PopupWidget(int pos_x, int pos_y, uint timeout_ms, uint num_args) :
		Widget(pos_x, pos_y, num_args), timeout(timeout_ms) {};

	virtual void setFact(uint _idx, Fact fact) {
		auto now = std::chrono::steady_clock::now();
		std::string msg = fact.getStrValue();
		msgs.push_back(std::pair(now, msg));
	}

	void draw(cairo_t *cr) {
		auto [x, y] = xy(cr);
		auto now = std::chrono::steady_clock::now();

		// Remove outdated messages
		while (!msgs.empty() && (now - msgs.front().first > timeout)) {
			msgs.pop_front();
		}
		uint y_offset = y;
		for (auto [time, msg] : msgs) {
			auto past = std::chrono::duration_cast<std::chrono::milliseconds>(now - time);
			double fade_fraction = 1.0 - static_cast<double>(past.count()) / static_cast<double>(timeout.count());

			// Cairo's `cairo_show_text` does not honour `\n`, so split the
			// message on newlines and render each line on its own row.
			std::vector<std::string> lines;
			{
				size_t start = 0;
				while (start <= msg.size()) {
					size_t nl = msg.find('\n', start);
					if (nl == std::string::npos) {
						lines.push_back(msg.substr(start));
						break;
					}
					lines.push_back(msg.substr(start, nl - start));
					start = nl + 1;
				}
			}

			// Compute the bounding box for the whole multi-line message so
			// the background sits behind every line.
			double padding = 5.0;
			double max_width = 0.0;
			double total_height = 0.0;
			double line_spacing = 2.0;
			std::vector<cairo_text_extents_t> extents_per_line(lines.size());
			for (size_t i = 0; i < lines.size(); ++i) {
				cairo_text_extents(cr, lines[i].c_str(), &extents_per_line[i]);
				if (extents_per_line[i].width > max_width) {
					max_width = extents_per_line[i].width;
				}
				total_height += extents_per_line[i].height;
				if (i + 1 < lines.size()) {
					total_height += line_spacing;
				}
			}

			// Draw popup box
			cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, fade_fraction / 3.0);
			cairo_rectangle(cr,
							x - padding,
							y_offset + padding,
							max_width + (padding * 2), -(total_height + (padding * 2)));
			cairo_fill(cr);

			// Draw popup text, line by line
			cairo_set_source_rgba(cr, 255.0, 255.0, 255.0, fade_fraction);
			uint line_y = y_offset;
			for (size_t i = lines.size(); i-- > 0; ) {
				cairo_move_to(cr, x, line_y);
				cairo_show_text(cr, lines[i].c_str());
				if (i > 0) {
					line_y -= extents_per_line[i].height + line_spacing;
				}
			}
			y_offset += total_height + (padding * 2) + 2;
		}
	}

private:
	std::deque<std::pair<
				   std::chrono::time_point<std::chrono::steady_clock>,
				   std::string
				   >> msgs;
	std::chrono::milliseconds timeout;
};

//
// Specific widgets
//

class DvrStatusWidget: public IconTextWidget {
public:
	DvrStatusWidget(int pos_x, int pos_y, cairo_surface_t *icon, std::string text) :
		IconTextWidget(pos_x, pos_y, icon, text) {
		args.push_back(Fact());
	};

	void draw(cairo_t *cr) {
		if(args[0].isDefined() && args[0].getBoolValue()) {
			auto [x, y] = xy(cr);
			cairo_save(cr);
			cairo_set_source_surface(cr, icon, x, y - 20);
			cairo_paint(cr);
			cairo_set_source_rgba(cr, 255.0, 0.0, 0.0, 1);
			cairo_move_to(cr, x + 40, y);
			cairo_show_text(cr, text.c_str());
			cairo_restore(cr);
		}
	}
};

class VideoWidget: public IconTplTextWidget {
public:
  VideoWidget(int pos_x, int pos_y, uint window_size_ms, uint bucket_size_ms,
              cairo_surface_t *icon, std::string tpl, uint num_args, uint frame_idx) :
		IconTplTextWidget(pos_x, pos_y, icon, tpl, num_args),
		fps(window_size_ms, bucket_size_ms), frame_idx_(frame_idx) {};

	virtual void setFact(uint idx, Fact fact) {
		if (idx == frame_idx_) {
			ulong num_frames = fact.getUintValue(); // should be always '1'
			fps.add(num_frames);
			args[idx] = Fact(FactMeta("video_fps"), (ulong)fps.rate_per_second_over_last_ms(1000));
		} else {
			args[idx] = fact;
		}
	}

private:
	RunningAverage fps;
	uint frame_idx_;
};

class VideoBitrateWidget: public IconTplTextWidget {
public:
  VideoBitrateWidget(int pos_x, int pos_y, uint window_size_ms, uint bucket_size_ms,
					 cairo_surface_t *icon, std::string tpl, uint num_args) :
		IconTplTextWidget(pos_x, pos_y, icon, tpl, num_args),
		bps(window_size_ms, bucket_size_ms) {
	  if (num_args != 1) throw std::invalid_argument("VideoBitrateWidget: expected 1 fact, got " + std::to_string(num_args));
  };

	virtual void setFact(uint idx, Fact fact) {
		if (idx != 0) { spdlog::error("VideoBitrateWidget: unexpected setFact idx {}", idx); return; }
		// replace the value with its increment rate per-second
		ulong num_bytes = fact.getUintValue();
		bps.add(num_bytes);
		// 125000 is 1_000_000 / 8 (megabits, not megabytes)
		args[idx] = Fact(FactMeta("video_mbps"), bps.rate_per_second_over_last_ms(1000) / 125000.0);
	}

private:
	RunningAverage bps;
};

class VideoDecodeLatencyWidget: public IconTplTextWidget {
public:
  VideoDecodeLatencyWidget(int pos_x, int pos_y, uint window_size_ms, uint bucket_size_ms,
					 cairo_surface_t *icon, std::string tpl, uint num_args) :
		IconTplTextWidget(pos_x, pos_y, icon, tpl, 3),  // 3 args, because we calculate min/max/avg
		timing(window_size_ms, bucket_size_ms) {
	  if (num_args != 1) throw std::invalid_argument("VideoDecodeLatencyWidget: expected 1 fact, got " + std::to_string(num_args));
  };

	virtual void setFact(uint idx, Fact fact) {
		if (idx != 0) { spdlog::error("VideoDecodeLatencyWidget: unexpected setFact idx {}", idx); return; }
		ulong decode_time = fact.getUintValue();
		timing.add(decode_time);
		Stats stats = timing.get_stats_over_last_ms_result(1000);
		args[0] = Fact(FactMeta("video_avg"), stats.average);
		args[1] = Fact(FactMeta("video_min"), stats.min);
		args[2] = Fact(FactMeta("video_max"), stats.max);
	}

private:
	RunningAverage timing;
};


class GPSWidget: public Widget {
public:
	GPSWidget(int pos_x, int pos_y, uint num_args) :
		Widget(pos_x, pos_y, num_args) {
		if (num_args != 3) throw std::invalid_argument("GPSWidget: expected 3 facts, got " + std::to_string(num_args));
	};

	void draw(cairo_t *cr) {
		if( !(args[0].isDefined() && args[1].isDefined() && args[2].isDefined()) ) return;
		auto [x, y] = xy(cr);
		std::string fix_type = "undef";
		char buf[64];
		switch (args[0].getUintValue()) {
		case 0:
			fix_type = "no GPS";
			break;
		case 1:
			fix_type = "no fix";
			break;
		case 2:
			fix_type = "2D fix";
			break;
		case 3:
			fix_type = "3D fix";
			break;
		case 4:
			fix_type = "DGPS/SBAS 3D";
			break;
		case 5:
			fix_type = "RTK float 3D";
			break;
		case 6:
			fix_type = "RTK Fixed 3D";
			break;
		case 7:
			fix_type = "Static fixed";
			break;
		case 8:
			fix_type = "PPP 3D";
			break;
		}
		double lat = args[1].getIntValue() * 1.0e-7;
		double lon = args[2].getIntValue() * 1.0e-7;
		snprintf(buf, sizeof(buf), "%s Lat:%f, Lon:%f", fix_type.c_str(), lat, lon);
		cairo_set_source_rgba(cr, 255.0, 255.0, 255.0, 1);
		cairo_move_to(cr, x, y);
		cairo_show_text(cr, buf);
	}
};


/**
 * Widget that shows approximate voltage of a battery cell.
 * If the number of cells is 0, it estimates it from the pack voltage based on max_voltage_mv;
 * If the number of cells is -1, it estimates only even cell numbers (2, 4, 6, 8, ...) - this
 * fixes the situation when, eg, discharged to 20v 6s LiIon would be recognized as 5s.
 *
 * Widget's text is drawn in white when battery is above 20% from critical. And below 20% it
 * gradually transitions from yellow through orange to red.
 */
class BatteryCellWidget: public TplTextWidget {
public:
    float warn_percentage = 0.2;

    BatteryCellWidget(int pos_x, int pos_y,
                      int critical_voltage_mv, int max_voltage_mv, int num_cells,
                      std::string tpl, uint num_args) :
        TplTextWidget(pos_x, pos_y, tpl, num_args), critical_voltage_mv(critical_voltage_mv),
        max_voltage_mv(max_voltage_mv), num_cells(num_cells) {
        if (num_args != 1) throw std::invalid_argument("BatteryCellWidget: expected 1 fact, got " + std::to_string(num_args));
    };

    virtual void setFact(uint idx, Fact fact) {
        if (idx != 0) { spdlog::error("BatteryCellWidget: unexpected setFact idx {}", idx); return; }
        // replace the pack value with per-cell value
        long voltage_mv = (long)fact;
        int cells;
        if (num_cells > 0) {
            cells = num_cells;
        } else if (num_cells == 0) {
            // estimate any number of cells
            cells = (voltage_mv / max_voltage_mv) + 1;
        } else {
            // estimate even number of cells
            cells = (voltage_mv / max_voltage_mv) + 1;
            if (cells % 2 != 0) {
                cells++;
            }
        }
        long cell_voltage_mv = voltage_mv / cells;
        args[0] = Fact(FactMeta("volts"), (double)cell_voltage_mv / 1000.0);
    }


    virtual void draw(cairo_t *cr) {
        auto [x, y] = xy(cr);
        const Fact& fact = args[0];
        if (!fact.isDefined()) return;
        auto cell_voltage = fact.getDoubleValue();
        auto cell_voltage_mv = cell_voltage * 1000;

        std::unique_ptr<std::string> msg = render_tpl();

        if (cell_voltage_mv <= critical_voltage_mv) {
            // Draw in red
            cairo_set_source_rgba(cr, 255.0, 0, 0, 1);
        } else {
            // Now we know voltage is above critical
            float remaining_percentage =
                (float)(cell_voltage_mv - critical_voltage_mv) /
                (max_voltage_mv - critical_voltage_mv);

            if (remaining_percentage < warn_percentage) {
                // Calculate green based on remaining percentage (0--warn_percentage% range)
                double green_value = 255.0 * (remaining_percentage / warn_percentage);
                // Transition from yellow through orange to red
                cairo_set_source_rgba(cr, 255.0, green_value, 0, 1);
            } else {
                // White when above 20%
                cairo_set_source_rgba(cr, 255.0, 255.0, 255.0, 1);
            }
        }
        cairo_move_to(cr, x, y);
        cairo_show_text(cr, msg->c_str());
    }
protected:
    int critical_voltage_mv;
    int max_voltage_mv;
    int num_cells;
};

class DebugWidget: public Widget {
public:
	DebugWidget(int pos_x, int pos_y, uint num_args) :
		Widget(pos_x, pos_y, num_args) {};

	void draw(cairo_t *cr) {
		auto [x, y] = xy(cr);
		auto y_offset = y;
		for (Fact &fact : args) {
			std::string text = fact.asVerboseString();
			cairo_set_source_rgba(cr, 255.0, 50.0, 50.0, 1);
			cairo_move_to(cr, x, y_offset);
			cairo_show_text(cr, text.c_str());
			y_offset += 20;
			SPDLOG_INFO("dbg draw {}", text);
		}
	}
};

class ExternalSurfaceWidget: public Widget {
public:
	ExternalSurfaceWidget(int pos_x, int pos_y, std::string shm_name ): Widget(pos_x, pos_y), shm_name(shm_name)  {};

	virtual void init_shm(cairo_t *cr) {
		SPDLOG_INFO("creating shm region {}", shm_name);

		cairo_surface_t *target = cairo_get_target(cr);
		int width = cairo_image_surface_get_width(target);
		int height = cairo_image_surface_get_height(target);

		// Calculate total shared memory size
		shm_size = sizeof(SharedMemoryRegion) + (width * height * 4); // Metadata + Image data

		// Create shared memory region
		int shm_fd = shm_open(shm_name.c_str(), O_CREAT | O_RDWR, 0666);
		if (shm_fd == -1) {
			perror("Failed to create shared memory");
			return;
		}

		if (ftruncate(shm_fd, shm_size) == -1) {
			perror("Failed to set shared memory size");
			shm_unlink(shm_name.c_str());
			return;
		}

		// Map shared memory to process address space
		auto *shm_region = static_cast<SharedMemoryRegion*>(
			mmap(0, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0)
		);
		if (shm_region == MAP_FAILED) {
			perror("Failed to map shared memory");
			shm_unlink(shm_name.c_str());
			return;
		}

		// Write metadata
		shm_region->width = width;
		shm_region->height = height;

		// Create Cairo surface for the image data
		shm_surface = cairo_image_surface_create_for_data(
			shm_region->data, CAIRO_FORMAT_ARGB32, width, height, width * 4
		);

		// Store pointer for cleanup
		shm_data = reinterpret_cast<unsigned char*>(shm_region);
	}


	virtual void draw(cairo_t *cr) {

		if (! shm_surface) 
			init_shm(cr);
		auto [x, y] = xy(cr);
		cairo_set_source_surface(cr, shm_surface, x, y); // Position at (0, 0)
    	cairo_paint(cr); // Paint shm_surface onto base_surface
	}

	~ExternalSurfaceWidget() {
		SPDLOG_INFO("bye, bye, shm region {}", shm_name);
		if (shm_surface) {
			cairo_surface_destroy(shm_surface);
		}
		if (shm_data) {
			munmap(shm_data, shm_size);
		}
		shm_unlink(shm_name.c_str());
	}

protected:
	cairo_surface_t *shm_surface = nullptr;
	unsigned char *shm_data = nullptr;
	size_t shm_size;
	std::string shm_name;
};

class IconSelectorWidget : public Widget {
public:
    IconSelectorWidget(int pos_x, int pos_y, const std::vector<std::pair<std::pair<int, int>, std::filesystem::path>>& ranges_and_icons, const std::filesystem::path& assets_dir)
        : Widget(pos_x, pos_y), assets_dir(assets_dir) {
        args.push_back(Fact()); // Expect one fact as input

        // Load and cache all icons during initialization
        for (const auto& [range, icon_path] : ranges_and_icons) {
            cairo_surface_t* icon = openIcon(icon_path);
            if (icon) {
                icon_cache[range] = icon;
            }
        }
    }

    virtual ~IconSelectorWidget() {
        // Clean up cached icons
        for (auto& [range, icon] : icon_cache) {
            if (icon) {
                cairo_surface_destroy(icon);
            }
        }
    }

    virtual void setFact(uint idx, Fact fact) override {
        if (idx != 0) { spdlog::error("IconSelectorWidget: unexpected setFact idx {}", idx); return; }
        args[idx] = fact;
        current_icon = selectIcon(fact);
    }

    virtual void draw(cairo_t *cr) override {
        if (!current_icon) return;

        auto [x, y] = xy(cr);
        cairo_set_source_surface(cr, current_icon, x, y);
        cairo_paint(cr);
    }

private:
    cairo_surface_t* selectIcon(Fact& fact) {
        if (!fact.isDefined()) return nullptr;

        long value = 0;
        
        // Convert all fact types to comparable integer values
        switch (fact.getType()) {
            case Fact::T_BOOL:
                value = fact.getBoolValue() ? 1 : 0;
                break;
            case Fact::T_INT:
                value = fact.getIntValue();
                break;
            case Fact::T_UINT:
                value = static_cast<long>(fact.getUintValue());
                break;
            case Fact::T_DOUBLE:
                value = static_cast<long>(fact.getDoubleValue());
                break;
            case Fact::T_STRING:
                try {
                    value = std::stol(fact.getStrValue());
                } catch (...) {
                    // If string can't be converted to number, use 0
                    value = 0;
                }
                break;
            case Fact::T_UNDEF:
            default:
                return nullptr;
        }

        // Iterate through the configured ranges and select the appropriate icon
        for (const auto& [range, icon] : icon_cache) {
            if (value >= range.first && value <= range.second) {
                return icon;
            }
        }

        return nullptr; // No icon selected
    }

    cairo_surface_t* openIcon(const std::filesystem::path& icon_path) {
        std::filesystem::path full_path = assets_dir / icon_path;
        cairo_surface_t* icon = cairo_image_surface_create_from_png(full_path.c_str());
        if (cairo_surface_status(icon) != CAIRO_STATUS_SUCCESS) {
            spdlog::error("Failed to open icon: {}", full_path.string());
            return nullptr;
        }
        return icon;
    }

    std::map<std::pair<int, int>, cairo_surface_t*> icon_cache; // Cache of loaded icons
    std::filesystem::path assets_dir;
    cairo_surface_t* current_icon = nullptr; // Currently selected icon
};

class HeadingTapeWidget : public Widget {
public:
    // label_position: "top" | "middle" | "bottom" — where labels sit inside the tape
    // show_degrees: also label non-cardinal/intercardinal ticks with degree numbers
    // degree_interval: spacing of degree labels (only when show_degrees is true)
    // tick_*_h: pixel height of each tick category
    HeadingTapeWidget(int pos_x, int pos_y, int width, int height, int range_deg,
                      const std::string& label_pos, bool show_degrees, int degree_interval,
                      int tick_cardinal_h, int tick_intercardinal_h,
                      int tick_medium_h, int tick_small_h,
                      double center_r, double center_g, double center_b,
                      bool show_home, double home_r, double home_g, double home_b)
        : Widget(pos_x, pos_y, 0), width_(width), height_(height), range_deg_(range_deg),
          label_pos_(label_pos), show_degrees_(show_degrees), degree_interval_(degree_interval),
          tick_cardinal_h_(tick_cardinal_h), tick_intercardinal_h_(tick_intercardinal_h),
          tick_medium_h_(tick_medium_h), tick_small_h_(tick_small_h),
          center_r_(center_r), center_g_(center_g), center_b_(center_b),
          show_home_(show_home), home_r_(home_r), home_g_(home_g), home_b_(home_b) {}

    void setFact(uint idx, Fact fact) override {
        if (idx == 0) {
            if (fact.getType() == Fact::T_INT)
                heading_ = (int)((fact.getIntValue() % 360 + 360) % 360);
            else if (fact.getType() == Fact::T_UINT)
                heading_ = (int)(fact.getUintValue() % 360);
        } else if (idx == 1) {
            // Home bearing relative to drone heading (degrees, any range)
            if (fact.getType() == Fact::T_DOUBLE)      home_bearing_ = fact.getDoubleValue();
            else if (fact.getType() == Fact::T_INT)    home_bearing_ = (double)fact.getIntValue();
            else if (fact.getType() == Fact::T_UINT)   home_bearing_ = (double)fact.getUintValue();
        } else {
            spdlog::error("HeadingTapeWidget: unexpected setFact idx {}", idx);
        }
    }

    void draw(cairo_t *cr) override {
        auto [ox, oy] = xy(cr);
        double px_per_deg = (double)width_ / range_deg_;
        double mid_x = ox + width_ / 2.0;

        // Label baseline Y
        const double approx_ascent  = 12.0;
        const double approx_descent =  3.0;
        const double label_gap      =  3.0;  // pixel gap between tick end and letter edge
        double label_y;
        if (label_pos_ == "bottom")
            label_y = oy + height_ - 4.0;
        else if (label_pos_ == "middle")
            label_y = oy + height_ / 2.0 + approx_ascent / 2.0;
        else  // "top"
            label_y = oy + approx_ascent + 2.0;

        // Exclusion zone: ticks must not enter [letter_top, letter_bottom]
        double letter_top    = label_y - approx_ascent - label_gap;
        double letter_bottom = label_y + approx_descent + label_gap;

        cairo_save(cr);
        cairo_select_font_face(cr, "Roboto", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);

        int half = range_deg_ / 2 + 5;

        // Pass 1: tick lines — clipped to tape, separated from letter area
        cairo_save(cr);
        cairo_rectangle(cr, ox, oy, width_, height_);
        cairo_clip(cr);

        for (int offset = -half; offset <= half; offset++) {
            int norm = ((heading_ + offset) % 360 + 360) % 360;
            bool cardinal      = (norm % 90 == 0);
            bool intercardinal = (norm % 45 == 0 && !cardinal);
            bool at_minor      = (norm % 10 == 0);
            if (!cardinal && !intercardinal && !at_minor) continue;

            double px = mid_x + offset * px_per_deg;
            if (px < ox || px > ox + width_) continue;

            int tick_h;
            if (cardinal)             tick_h = tick_cardinal_h_;
            else if (intercardinal)   tick_h = tick_intercardinal_h_;
            else if (norm % 30 == 0)  tick_h = tick_medium_h_;
            else                      tick_h = tick_small_h_;

            double alpha = cardinal ? 1.0 : (intercardinal ? 0.85 : 0.55);
            cairo_set_source_rgba(cr, 1, 1, 1, alpha);
            cairo_set_line_width(cr, cardinal ? 2.0 : 1.0);

            if (label_pos_ == "top") {
                // Labels at top → ticks grow downward from just below letter descenders
                cairo_move_to(cr, px, letter_bottom);
                cairo_line_to(cr, px, letter_bottom + tick_h);
            } else if (label_pos_ == "bottom") {
                // Labels at bottom → ticks grow upward from just above letter ascenders
                cairo_move_to(cr, px, letter_top);
                cairo_line_to(cr, px, letter_top - tick_h);
            } else {  // middle: two half-ticks above and below the letter, each with a gap
                double half_h = tick_h / 2.0;
                cairo_move_to(cr, px, letter_top);
                cairo_line_to(cr, px, letter_top - half_h);
                cairo_stroke(cr);
                cairo_move_to(cr, px, letter_bottom);
                cairo_line_to(cr, px, letter_bottom + half_h);
            }
            cairo_stroke(cr);
        }
        cairo_restore(cr);  // remove clip so labels aren't cut off

        // Pass 2: labels — drawn without clip so text is never half-cut at tape edges
        for (int offset = -half; offset <= half; offset++) {
            int norm = ((heading_ + offset) % 360 + 360) % 360;
            bool cardinal      = (norm % 90 == 0);
            bool intercardinal = (norm % 45 == 0 && !cardinal);
            bool show_deg      = show_degrees_ && (norm % degree_interval_ == 0)
                                 && !cardinal && !intercardinal;
            if (!cardinal && !intercardinal && !show_deg) continue;

            double px = mid_x + offset * px_per_deg;
            if (px < ox || px > ox + width_) continue;

            if (cardinal || intercardinal) {
                const char *label = compass_label(norm);
                double font_sz = cardinal ? 14.0 : 11.0;
                cairo_select_font_face(cr, "Roboto", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
                cairo_set_font_size(cr, font_sz);
                cairo_text_extents_t ext;
                cairo_text_extents(cr, label, &ext);
                cairo_set_source_rgba(cr, 1, 1, 1, cardinal ? 1.0 : 0.85);
                cairo_move_to(cr, px - ext.width / 2.0 - ext.x_bearing, label_y);
                cairo_show_text(cr, label);
                cairo_select_font_face(cr, "Roboto", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
            } else {  // degree number
                char dbuf[8];
                snprintf(dbuf, sizeof(dbuf), "%d", norm);
                cairo_set_font_size(cr, 10.0);
                cairo_text_extents_t ext;
                cairo_text_extents(cr, dbuf, &ext);
                cairo_set_source_rgba(cr, 0.75, 0.75, 0.75, 0.75);
                cairo_move_to(cr, px - ext.width / 2.0 - ext.x_bearing, label_y);
                cairo_show_text(cr, dbuf);
            }
        }

        // Center indicator: full-height line + downward triangle at top (configurable color)
        cairo_set_source_rgba(cr, center_r_, center_g_, center_b_, 1.0);
        cairo_set_line_width(cr, 2.0);
        cairo_move_to(cr, mid_x, oy);
        cairo_line_to(cr, mid_x, oy + height_);
        cairo_stroke(cr);
        cairo_move_to(cr, mid_x - 6, oy);
        cairo_line_to(cr, mid_x + 6, oy);
        cairo_line_to(cr, mid_x,     oy + 9);
        cairo_close_path(cr);
        cairo_fill(cr);

        // Home direction triangle: upward-pointing triangle at tape bottom
        if (show_home_ && home_bearing_ > -990.0) {
            // Normalize bearing to [-180, +180)
            double hb = home_bearing_;
            while (hb >= 180.0)  hb -= 360.0;
            while (hb < -180.0)  hb += 360.0;
            double home_px = mid_x + hb * px_per_deg;

            cairo_set_source_rgba(cr, home_r_, home_g_, home_b_, 1.0);
            cairo_select_font_face(cr, "Roboto", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
            cairo_set_font_size(cr, 14.0);
            if (home_px >= ox && home_px <= ox + width_) {
                // Fully visible: upward triangle at tape bottom + "H" label below
                double ty = oy + height_;
                cairo_move_to(cr, home_px,     ty - 9);
                cairo_line_to(cr, home_px - 6, ty);
                cairo_line_to(cr, home_px + 6, ty);
                cairo_close_path(cr);
                cairo_fill(cr);
                cairo_text_extents_t he;
                cairo_text_extents(cr, "H", &he);
                cairo_move_to(cr, home_px - he.width / 2.0 - he.x_bearing, ty + he.height + 2.0);
                cairo_show_text(cr, "H");
            } else {
                // Off-screen: arrow at edge + "H" beside it
                double edge_x = (home_px < ox) ? ox + 4 : ox + width_ - 4;
                double ey = oy + height_ - 6;
                double dir = (home_px < ox) ? -1.0 : 1.0;
                cairo_move_to(cr, edge_x + dir * 6, ey);
                cairo_line_to(cr, edge_x - dir * 3, ey - 5);
                cairo_line_to(cr, edge_x - dir * 3, ey + 5);
                cairo_close_path(cr);
                cairo_fill(cr);
                cairo_text_extents_t he;
                cairo_text_extents(cr, "H", &he);
                double label_x = (home_px < ox) ? edge_x + 8 : edge_x - 8 - he.width - he.x_bearing;
                cairo_move_to(cr, label_x, ey + he.height / 2.0);
                cairo_show_text(cr, "H");
            }
        }

        // Heading value above the tape
        char buf[8];
        snprintf(buf, sizeof(buf), "%3d", heading_);
        cairo_select_font_face(cr, "Roboto", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
        cairo_set_font_size(cr, 15.0);
        cairo_text_extents_t ext;
        cairo_text_extents(cr, buf, &ext);
        cairo_set_source_rgba(cr, 1, 1, 1, 1);
        cairo_move_to(cr, mid_x - ext.width / 2.0 - ext.x_bearing, oy - 4);
        cairo_show_text(cr, buf);

        cairo_restore(cr);
    }

private:
    static const char* compass_label(int d) {
        switch (d) {
        case 0:   return "N";
        case 45:  return "NE";
        case 90:  return "E";
        case 135: return "SE";
        case 180: return "S";
        case 225: return "SW";
        case 270: return "W";
        case 315: return "NW";
        default:  return "";
        }
    }

    int width_, height_, range_deg_;
    std::string label_pos_;
    bool show_degrees_;
    int degree_interval_;
    int tick_cardinal_h_, tick_intercardinal_h_, tick_medium_h_, tick_small_h_;
    double center_r_, center_g_, center_b_;
    bool show_home_;
    double home_r_, home_g_, home_b_;
    int heading_ = 0;
    double home_bearing_ = -999.0;  // sentinel: not yet received
};

// ----------------------------------------------------------------------------
// CompassWidget — round compass rose with heading, home direction, wind
// Facts:
//   0: mavlink.vfr_hud.heading        (required)
//   1: mavlink.home.bearing_relative  (optional, degrees relative to heading)
//   2: mavlink.wind.direction         (optional, degrees from N where wind comes FROM)
//   3: mavlink.wind.speed             (optional, m/s)
// Modes:
//   "north_up"  — ring fixed (N at top), center icon rotates with heading
//   "track_up"  — center icon always points forward, ring rotates under it
// Icons (built-in): "arrow" (default), "drone", "plane"
//   or a path to a PNG file
// ----------------------------------------------------------------------------
class CompassWidget : public Widget {
public:
    CompassWidget(int pos_x, int pos_y, int radius,
                  const std::string& mode, const std::string& icon,
                  double icon_r, double icon_g, double icon_b,
                  bool show_home, double home_r, double home_g, double home_b,
                  bool show_wind, double wind_r, double wind_g, double wind_b,
                  double bg_alpha,
                  const std::filesystem::path& assets_dir)
        : Widget(pos_x, pos_y, 0),
          radius_(radius), mode_(mode), icon_name_(icon),
          icon_r_(icon_r), icon_g_(icon_g), icon_b_(icon_b),
          show_home_(show_home), home_r_(home_r), home_g_(home_g), home_b_(home_b),
          show_wind_(show_wind), wind_r_(wind_r), wind_g_(wind_g), wind_b_(wind_b),
          bg_alpha_(bg_alpha),
          assets_dir_(assets_dir) {}

    ~CompassWidget() {
        if (icon_surface_) cairo_surface_destroy(icon_surface_);
    }

    void setFact(uint idx, Fact fact) override {
        if (idx == 0) {
            if (fact.getType() == Fact::T_INT)
                heading_ = (int)((fact.getIntValue() % 360 + 360) % 360);
            else if (fact.getType() == Fact::T_UINT)
                heading_ = (int)(fact.getUintValue() % 360);
        } else if (idx == 1) {
            if (fact.getType() == Fact::T_DOUBLE)     home_bearing_ = fact.getDoubleValue();
            else if (fact.getType() == Fact::T_INT)   home_bearing_ = (double)fact.getIntValue();
            else if (fact.getType() == Fact::T_UINT)  home_bearing_ = (double)fact.getUintValue();
        } else if (idx == 2) {
            if (fact.getType() == Fact::T_DOUBLE)     wind_dir_ = fact.getDoubleValue();
            else if (fact.getType() == Fact::T_INT)   wind_dir_ = (double)fact.getIntValue();
            else if (fact.getType() == Fact::T_UINT)  wind_dir_ = (double)fact.getUintValue();
        } else if (idx == 3) {
            if (fact.getType() == Fact::T_DOUBLE)     wind_speed_ = fact.getDoubleValue();
            else if (fact.getType() == Fact::T_INT)   wind_speed_ = (double)fact.getIntValue();
            else if (fact.getType() == Fact::T_UINT)  wind_speed_ = (double)fact.getUintValue();
        } else {
            spdlog::error("CompassWidget: unexpected setFact idx {}", idx);
        }
    }

    void draw(cairo_t *cr) override {
        auto [ox, oy] = xy(cr);
        double cx = ox + radius_;
        double cy = oy + radius_;
        double r  = (double)radius_;
        double ring_w = r * 0.18;
        double r_inner = r - ring_w;
        double heading_rad = heading_ * M_PI / 180.0;

        cairo_save(cr);

        // --- Background circle ---
        cairo_arc(cr, cx, cy, r, 0, 2 * M_PI);
        cairo_set_source_rgba(cr, 0.0, 0.0, 0.0, bg_alpha_);
        cairo_fill(cr);

        // --- Ring + labels + markers (rotated for track_up) ---
        cairo_save(cr);
        if (mode_ == "track_up") {
            cairo_translate(cr, cx, cy);
            cairo_rotate(cr, -heading_rad);
            cairo_translate(cr, -cx, -cy);
        }

        // Tick marks every 5°, major at every 45°, medium at every 10°
        for (int b = 0; b < 360; b += 5) {
            bool major  = (b % 45 == 0);
            bool medium = (b % 10 == 0);
            double tick_len = major ? ring_w * 0.65 : (medium ? ring_w * 0.4 : ring_w * 0.22);
            double alpha = major ? 1.0 : (medium ? 0.7 : 0.45);
            double lw    = major ? 2.0 : 1.0;
            double bx = bearing_x(cx, r - 1.0, b);
            double by = bearing_y(cy, r - 1.0, b);
            double ex = bearing_x(cx, r - tick_len, b);
            double ey = bearing_y(cy, r - tick_len, b);
            cairo_set_source_rgba(cr, 1, 1, 1, alpha);
            cairo_set_line_width(cr, lw);
            cairo_move_to(cr, bx, by);
            cairo_line_to(cr, ex, ey);
            cairo_stroke(cr);
        }

        // Cardinal labels: N (white), others (light grey)
        struct { int b; const char *lbl; bool cardinal; } labels[] = {
            {0,   "N",  true},
            {45,  "NE", false},
            {90,  "E",  true},
            {135, "SE", false},
            {180, "S",  true},
            {225, "SW", false},
            {270, "W",  true},
            {315, "NW", false},
        };
        double label_r = r_inner - 2.0;
        for (auto &lb : labels) {
            double fs   = lb.cardinal ? 14.0 : 10.0;
            double alpha = lb.cardinal ? 1.0 : 0.75;
            double lx = bearing_x(cx, label_r, lb.b);
            double ly = bearing_y(cy, label_r, lb.b);
            cairo_select_font_face(cr, "Roboto", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
            cairo_set_font_size(cr, fs);
            cairo_text_extents_t ext;
            cairo_text_extents(cr, lb.lbl, &ext);
            // "N" label shifted slightly inward so it doesn't overlap the edge ticks
            double push_in = lb.cardinal ? 6.0 : 3.0;
            double dx = -sin(lb.b * M_PI / 180.0) * push_in;
            double dy =  cos(lb.b * M_PI / 180.0) * push_in;
            cairo_set_source_rgba(cr, 1, 1, 1, alpha);
            cairo_move_to(cr, lx - ext.width / 2.0 - ext.x_bearing + dx,
                              ly + ext.height / 2.0 - ext.height / 2.0 * 0.35 + dy);
            cairo_show_text(cr, lb.lbl);
        }

        // Home marker — filled triangle on the inner ring edge pointing inward
        if (show_home_ && home_bearing_ > -990.0) {
            double abs_home = fmod(heading_ + home_bearing_ + 3600.0, 360.0);
            draw_ring_marker(cr, cx, cy, r_inner, abs_home, home_r_, home_g_, home_b_, "H");
        }

        // Wind marker — elongated inward arrow at the "from" direction
        if (show_wind_ && wind_dir_ > -990.0) {
            draw_ring_marker(cr, cx, cy, r_inner, wind_dir_, wind_r_, wind_g_, wind_b_, "~");
            // Wind speed label next to marker
            if (wind_speed_ > -0.5) {
                char wbuf[16];
                snprintf(wbuf, sizeof(wbuf), "%.1fm/s", wind_speed_);
                double wx = bearing_x(cx, r_inner - 18.0, wind_dir_);
                double wy = bearing_y(cy, r_inner - 18.0, wind_dir_);
                cairo_select_font_face(cr, "Roboto", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
                cairo_set_font_size(cr, 9.0);
                cairo_text_extents_t wext;
                cairo_text_extents(cr, wbuf, &wext);
                cairo_set_source_rgba(cr, wind_r_, wind_g_, wind_b_, 0.9);
                cairo_move_to(cr, wx - wext.width / 2.0 - wext.x_bearing, wy + 4.0);
                cairo_show_text(cr, wbuf);
            }
        }

        cairo_restore(cr);  // undo ring rotation

        // --- Center icon ---
        cairo_save(cr);
        cairo_translate(cr, cx, cy);
        if (mode_ == "north_up") cairo_rotate(cr, heading_rad);
        draw_icon(cr, r_inner * 0.52);
        cairo_restore(cr);

        // --- Heading text (fixed, always below center) ---
        char buf[8];
        snprintf(buf, sizeof(buf), "%d\xc2\xb0", heading_);  // UTF-8 degree sign
        cairo_select_font_face(cr, "Roboto", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
        cairo_set_font_size(cr, 13.0);
        cairo_text_extents_t ext;
        cairo_text_extents(cr, buf, &ext);
        cairo_set_source_rgba(cr, 1, 1, 1, 1.0);
        cairo_move_to(cr, cx - ext.width / 2.0 - ext.x_bearing, cy + r_inner * 0.72);
        cairo_show_text(cr, buf);

        cairo_restore(cr);
    }

private:
    // Convert compass bearing (degrees, 0=N, clockwise) to screen x/y on a circle
    static double bearing_x(double cx, double r, double b) {
        return cx + r * sin(b * M_PI / 180.0);
    }
    static double bearing_y(double cy, double r, double b) {
        return cy - r * cos(b * M_PI / 180.0);
    }

    // Small triangle marker pointing inward from the ring at bearing b, with optional label
    void draw_ring_marker(cairo_t *cr, double cx, double cy, double r_inner,
                          double b, double mr, double mg, double mb, const char* label = nullptr) {
        double br = b * M_PI / 180.0;
        double sin_b = sin(br), cos_b = cos(br);
        double tip_r  = r_inner - 5.0;
        double base_r = r_inner + 4.0;
        double half_w = 6.0;

        // Tip point
        double tx = cx + tip_r  * sin_b;
        double ty = cy - tip_r  * cos_b;
        // Base center
        double bx = cx + base_r * sin_b;
        double by = cy - base_r * cos_b;
        // Perpendicular (tangent) direction
        double px =  cos_b;
        double py =  sin_b;

        cairo_set_source_rgba(cr, mr, mg, mb, 1.0);
        cairo_move_to(cr, tx, ty);
        cairo_line_to(cr, bx + px * half_w, by + py * half_w);
        cairo_line_to(cr, bx - px * half_w, by - py * half_w);
        cairo_close_path(cr);
        cairo_fill(cr);

        if (label) {
            // Small label just inside the triangle tip, centered at bearing
            double lx = cx + (tip_r - 8.0) * sin_b;
            double ly = cy - (tip_r - 8.0) * cos_b;
            cairo_select_font_face(cr, "Roboto", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
            cairo_set_font_size(cr, 8.0);
            cairo_text_extents_t ext;
            cairo_text_extents(cr, label, &ext);
            cairo_set_source_rgba(cr, mr, mg, mb, 1.0);
            cairo_move_to(cr, lx - ext.width / 2.0 - ext.x_bearing, ly + ext.height / 2.0);
            cairo_show_text(cr, label);
        }
    }

    // Draw center icon centered at (0,0), pointing "up" (toward -Y), size = half-span
    void draw_icon(cairo_t *cr, double size) {
        if (!icon_surface_loaded_) {
            icon_surface_loaded_ = true;
            // Try to load PNG if icon_name_ looks like a file path
            if (icon_name_.find('/') != std::string::npos || icon_name_.find('.') != std::string::npos) {
                std::filesystem::path p = icon_name_;
                if (p.is_relative()) p = assets_dir_ / p;
                icon_surface_ = cairo_image_surface_create_from_png(p.c_str());
                if (cairo_surface_status(icon_surface_) != CAIRO_STATUS_SUCCESS) {
                    spdlog::warn("CompassWidget: failed to load icon PNG '{}', using built-in arrow", p.string());
                    cairo_surface_destroy(icon_surface_);
                    icon_surface_ = nullptr;
                }
            }
        }

        if (icon_surface_) {
            // Render loaded PNG, scaled to fit 2*size box, centered
            int iw = cairo_image_surface_get_width(icon_surface_);
            int ih = cairo_image_surface_get_height(icon_surface_);
            double scale = (2.0 * size) / std::max(iw, ih);
            cairo_save(cr);
            cairo_scale(cr, scale, scale);
            cairo_set_source_surface(cr, icon_surface_, -iw / 2.0, -ih / 2.0);
            cairo_paint(cr);
            cairo_restore(cr);
            return;
        }

        cairo_set_source_rgba(cr, icon_r_, icon_g_, icon_b_, 1.0);

        if (icon_name_ == "drone") {
            draw_icon_drone(cr, size);
        } else if (icon_name_ == "plane") {
            draw_icon_plane(cr, size);
        } else {
            draw_icon_arrow(cr, size);
        }
    }

    // QGC-style navigation arrow: filled chevron, tip at top
    void draw_icon_arrow(cairo_t *cr, double s) {
        cairo_move_to(cr,  0,       -s);         // tip
        cairo_line_to(cr,  s * 0.5,  s * 0.55); // bottom-right
        cairo_line_to(cr,  0,        s * 0.15);  // center notch
        cairo_line_to(cr, -s * 0.5,  s * 0.55); // bottom-left
        cairo_close_path(cr);
        cairo_fill(cr);
    }

    // Quadcopter X-frame viewed from above
    void draw_icon_drone(cairo_t *cr, double s) {
        // Body
        cairo_arc(cr, 0, 0, s * 0.22, 0, 2 * M_PI);
        cairo_fill(cr);
        // Arms at 45°, 135°, 225°, 315°
        double arm_angles[] = {45, 135, 225, 315};
        for (double a : arm_angles) {
            double ax = s * 0.65 * sin(a * M_PI / 180.0);
            double ay = -s * 0.65 * cos(a * M_PI / 180.0);
            cairo_set_line_width(cr, 2.5);
            cairo_move_to(cr, 0, 0);
            cairo_line_to(cr, ax, ay);
            cairo_stroke(cr);
            // Propeller circle
            cairo_arc(cr, ax, ay, s * 0.2, 0, 2 * M_PI);
            cairo_set_line_width(cr, 1.5);
            cairo_stroke(cr);
        }
        // Heading arrow between front arms (pointing forward = up)
        cairo_move_to(cr,  0,        -s * 0.72);  // tip
        cairo_line_to(cr,  s * 0.14, -s * 0.44);  // base right
        cairo_line_to(cr, -s * 0.14, -s * 0.44);  // base left
        cairo_close_path(cr);
        cairo_fill(cr);
    }

    // Fixed-wing plane viewed from above
    void draw_icon_plane(cairo_t *cr, double s) {
        // Fuselage
        cairo_set_line_width(cr, 3.5);
        cairo_move_to(cr, 0, -s * 0.9);
        cairo_line_to(cr, 0,  s * 0.5);
        cairo_stroke(cr);
        // Main wings
        cairo_set_line_width(cr, 2.5);
        cairo_move_to(cr, -s * 0.95, -s * 0.1);
        cairo_line_to(cr,  s * 0.95, -s * 0.1);
        cairo_stroke(cr);
        // Tail
        cairo_set_line_width(cr, 2.0);
        cairo_move_to(cr, -s * 0.42, s * 0.38);
        cairo_line_to(cr,  s * 0.42, s * 0.38);
        cairo_stroke(cr);
    }

    int radius_;
    std::string mode_;
    std::string icon_name_;
    double icon_r_, icon_g_, icon_b_;
    bool show_home_;
    double home_r_, home_g_, home_b_;
    bool show_wind_;
    double wind_r_, wind_g_, wind_b_;
    double bg_alpha_;
    std::filesystem::path assets_dir_;

    int    heading_    = 0;
    double home_bearing_ = -999.0;
    double wind_dir_   = -999.0;
    double wind_speed_ = -1.0;

    cairo_surface_t *icon_surface_ = nullptr;
    bool icon_surface_loaded_ = false;
};

class Osd {
public:
	void loadConfig(json cfg) {
		json obj;
		if (cfg.contains("format")) {
			auto cfg_format = cfg.at("format").template get<std::string>();
			if (cfg_format != "0.0.1" && cfg_format != "0.0.2") {
				spdlog::warn("Unexpected OSD config format: {}. OSD may look wrong", cfg_format);
			}
		} else {
			spdlog::error("OSD config doesn't have 'format' key");
			return;
		}
		if (!cfg.contains("widgets")) {
			//|| cfg["widgets"].type() != json::value_t::array)
			spdlog::error("OSD config doesn't have 'widgets' key");
			return;
		}
		std::filesystem::path assets_dir(".");
		if (cfg.contains("assets_dir")) {
			assets_dir = cfg.at("assets_dir").template get<std::filesystem::path>();
		}
		json widgets_j = cfg.at("widgets");
		for (json widget_j : widgets_j) {
			auto [widget, wmatchers] = parseOneWidget(widget_j, assets_dir);
			if (widget) addWidget(widget, wmatchers);
		}
	}

	Osd *addWidget(Widget *widget, std::vector<FactMatcher> param_matchers) {
		uint arg_idx = 0;
		widgets.push_back(widget);
		for (auto matcher : param_matchers) {
			matchers.push_back(std::make_tuple(matcher, widget, arg_idx));
			arg_idx++;
		}
		return this;
	};

	void draw(cairo_t *cr) {
		for(auto &widget : widgets) {
			try {
				widget->draw(cr);
			} catch (const std::exception& e) {
				spdlog::warn("OSD widget draw error: {}", e.what());
			}
		}
	};

	void setFact(Fact fact) {
		for (auto& [matcher, widget, arg_idx] : matchers) {
			if (matcher.matches(fact)) {
				try {
					Fact converted_fact = matcher.convert(fact);
					widget->setFact(arg_idx, converted_fact);
				} catch (const std::exception& e) {
					spdlog::error("Failed to set fact {}: {}",
								  fact.asVerboseString(), e.what());
				}
			}
		}
	};

private:

	cairo_surface_t *openIcon(std::string widget_name, std::filesystem::path base_path,
							  std::filesystem::path icon_path) {
		if (icon_path.is_relative()) {
			icon_path = base_path / icon_path;
		}
		cairo_surface_t *icon = cairo_image_surface_create_from_png(icon_path.c_str());
		if (cairo_surface_status(icon) != CAIRO_STATUS_SUCCESS) {
			std::string status("OTHER_ERROR");
			switch (cairo_surface_status(icon)) {
			case CAIRO_STATUS_NULL_POINTER:
				status = "NULL_POINTER";
				break;
			case CAIRO_STATUS_NO_MEMORY:
				status = "NO_MEMORY";
				break;
			case CAIRO_STATUS_READ_ERROR:
				status = "READ_ERROR";
				break;
			case CAIRO_STATUS_INVALID_CONTENT:
				status = "INVALID_CONTENT";
				break;
			case CAIRO_STATUS_INVALID_FORMAT:
				status = "INVALID_FORMAT";
				break;
			case CAIRO_STATUS_INVALID_VISUAL:
				status = "INVALID_VISUAL";
				break;
			};
			spdlog::error("Widget '{}': Can't open icon '{}': {}",
						  widget_name, icon_path.string(), status);
			return NULL;
		}
		return icon;
	}

	std::pair<Widget*, std::vector<FactMatcher>> parseOneWidget(const json& widget_j, const std::filesystem::path& assets_dir) {
		if(!(widget_j.contains("name") || widget_j.contains("type") || widget_j.contains("x") ||
			 widget_j.contains("y") || widget_j.contains("facts"))) {
			spdlog::error("Missing required key name/type/x/y/facts");
			return {nullptr, {}};
		}
		auto name = widget_j.at("name").template get<std::string>();
		auto type = widget_j.at("type").template get<std::string>();
		auto x = widget_j.at("x").template get<int>();
		auto y = widget_j.at("y").template get<int>();
		std::vector<FactMatcher> matchers;
		for(json matcher_j : widget_j.at("facts")) {
			auto matcher_name = matcher_j.at("name").template get<std::string>();
			FactTags tags;
			if (matcher_j.contains("tags")) {
				for (auto& [key, value] : matcher_j.at("tags").items()) {
					tags.insert({key, value});
				}
			}
			if (matcher_j.contains("convert")) {
				auto expression_str = matcher_j.at("convert").template get<std::string>();
				try {
					matchers.push_back(FactMatcher(matcher_name, tags, expression_str));
				} catch (const ExpressionException& e) {
					spdlog::error("Invalid convert expression {}: {}",
								  expression_str, e.what());
				}
			} else {
				matchers.push_back(FactMatcher(matcher_name, tags));
			}
		}
		if (type == "TextWidget") {
			return {new TextWidget(x, y, widget_j.at("text").template get<std::string>()), matchers};
		} else if (type == "ExternalSurfaceWidget") {
			return {new ExternalSurfaceWidget(x, y, name), matchers};
		} else if (type == "IconSelectorWidget") {
			std::vector<std::pair<std::pair<int, int>, std::filesystem::path>> ranges_and_icons;
			for (const auto& range_icon : widget_j.at("ranges_and_icons")) {
				int range_start = range_icon.at("range")[0];
				int range_end = range_icon.at("range")[1];
				std::filesystem::path icon_path = range_icon.at("icon_path");
				ranges_and_icons.push_back({{range_start, range_end}, icon_path});
			}
			return {new IconSelectorWidget(x, y, ranges_and_icons, assets_dir), matchers};
		} else if (type == "TplTextWidget") {
			auto tpl = widget_j.at("template").template get<std::string>();
			return {new TplTextWidget(x, y, tpl, (uint)matchers.size()), matchers};
		} else if(type == "IconTplTextWidget") {
			auto tpl = widget_j.at("template").template get<std::string>();
			auto icon_path = widget_j.at("icon_path").template get<std::filesystem::path>();
			cairo_surface_t *icon = openIcon(name, assets_dir, icon_path);
			if (icon == NULL) return {nullptr, {}};
			return {new IconTplTextWidget(x, y, icon, tpl, (uint)matchers.size()), matchers};
		} else if(type == "DvrStatusWidget") {
			auto text = widget_j.at("text").template get<std::string>();
			auto icon_path = widget_j.at("icon_path").template get<std::filesystem::path>();
			cairo_surface_t *icon = openIcon(name, assets_dir, icon_path);
			if (icon == NULL) return {nullptr, {}};
			return {new DvrStatusWidget(x, y, icon, text), matchers};
		} else if(type == "VideoWidget") {
			auto tpl = widget_j.at("template").template get<std::string>();
			auto icon_path = widget_j.at("icon_path").template get<std::filesystem::path>();
			uint window_size_s = widget_j.at("per_second_window_s").template get<uint>();
			uint bucket_size_ms = widget_j.at("per_second_bucket_ms").template get<uint>();
			cairo_surface_t *icon = openIcon(name, assets_dir, icon_path);
			if (icon == NULL) return {nullptr, {}};
			uint frame_idx = 0;
			for (uint i = 0; i < matchers.size(); i++) {
				if (matchers[i].name == "video.displayed_frame") { frame_idx = i; break; }
			}
			return {new VideoWidget(x, y, window_size_s * 1000, bucket_size_ms, icon, tpl, (uint)matchers.size(), frame_idx), matchers};
		} else if(type == "VideoBitrateWidget") {
			auto tpl = widget_j.at("template").template get<std::string>();
			auto icon_path = widget_j.at("icon_path").template get<std::filesystem::path>();
			uint window_size_s = widget_j.at("per_second_window_s").template get<uint>();
			uint bucket_size_ms = widget_j.at("per_second_bucket_ms").template get<uint>();
			cairo_surface_t *icon = openIcon(name, assets_dir, icon_path);
			if (icon == NULL) return {nullptr, {}};
			return {new VideoBitrateWidget(x, y, window_size_s * 1000, bucket_size_ms, icon, tpl, (uint)matchers.size()), matchers};
		} else if(type == "VideoDecodeLatencyWidget") {
			auto tpl = widget_j.at("template").template get<std::string>();
			auto icon_path = widget_j.at("icon_path").template get<std::filesystem::path>();
			uint window_size_s = widget_j.at("per_second_window_s").template get<uint>();
			uint bucket_size_ms = widget_j.at("per_second_bucket_ms").template get<uint>();
			cairo_surface_t *icon = openIcon(name, assets_dir, icon_path);
			if (icon == NULL) return {nullptr, {}};
			return {new VideoDecodeLatencyWidget(x, y, window_size_s * 1000, bucket_size_ms, icon, tpl, 1), matchers};
		} else if(type == "BoxWidget") {
			auto width = widget_j.at("width").template get<uint>();
			auto height = widget_j.at("height").template get<uint>();
			json color_j = widget_j.at("color");
			auto r = color_j.at("r").template get<double>();
			auto g = color_j.at("g").template get<double>();
			auto b = color_j.at("b").template get<double>();
			auto a = color_j.at("alpha").template get<double>();
			return {new BoxWidget(x, y, width, height, r, g, b, a), matchers};
		} else if(type == "BoxWidgetContainer") {
			auto width = widget_j.at("width").template get<uint>();
			auto height = widget_j.at("height").template get<uint>();
			json color_j = widget_j.at("color");
			auto r = color_j.at("r").template get<double>();
			auto g = color_j.at("g").template get<double>();
			auto b = color_j.at("b").template get<double>();
			auto a = color_j.at("alpha").template get<double>();
			auto* container = new BoxWidgetContainer(x, y, width, height, r, g, b, a);
			if (widget_j.value("center_x", false))
				container->set_center_x(width);
			std::vector<FactMatcher> all_child_matchers;
			if (widget_j.contains("widgets")) {
				for (const json& child_j : widget_j.at("widgets")) {
					auto [child, child_matchers] = parseOneWidget(child_j, assets_dir);
					if (child) {
						container->addChild(child, child_matchers);
						for (auto& m : child_matchers)
							all_child_matchers.push_back(m.routing_copy());
					}
				}
			}
			return {container, all_child_matchers};
		} else if(type == "BarChartWidget") {
			auto width = widget_j.at("width").template get<uint>();
			auto height = widget_j.at("height").template get<uint>();
			auto window_s = widget_j.at("window_s").template get<uint>();
			auto num_buckets = widget_j.at("num_buckets").template get<uint>();
			auto stats_kind_str = widget_j.at("stats_kind").template get<std::string>();
			BarChartWidget::StatsField stats_kind;
			if (stats_kind_str == "sum") {
				stats_kind = BarChartWidget::STATS_SUM;
			} else if (stats_kind_str == "min") {
				stats_kind = BarChartWidget::STATS_MIN;
			} else if (stats_kind_str == "max") {
				stats_kind = BarChartWidget::STATS_MAX;
			} else if (stats_kind_str == "count") {
				stats_kind = BarChartWidget::STATS_COUNT;
			} else if (stats_kind_str == "avg") {
				stats_kind = BarChartWidget::STATS_AVG;
			} else {
				SPDLOG_WARN("{}: invalid stats_kind {}", name, stats_kind_str);
				return {nullptr, {}};
			}
			return {new BarChartWidget(x, y, width, height, window_s, num_buckets, stats_kind), matchers};
		} else if (type == "GPSWidget") {
			return {new GPSWidget(x, y, (uint)matchers.size()), matchers};
		} else if (type == "BatteryCellWidget") {
			int critical_mv = 3500;
			int max_mv = 4200;
			int num_cells = -1;
			auto tpl = widget_j.at("template").template get<std::string>();
			if (widget_j.contains("critical_voltage")) {
				critical_mv = (int)(widget_j.at("critical_voltage").template get<float>() * 1000);
			}
			if (widget_j.contains("max_voltage")) {
				max_mv = (int)(widget_j.at("max_voltage").template get<float>() * 1000);
			}
			if (widget_j.contains("num_cells")) {
				std::string cells = widget_j["num_cells"];
				if (cells == "auto") {
					num_cells = 0;
				} else if (cells == "even") {
					num_cells = -1;
				} else {
					num_cells = widget_j["num_cells"].get<int>();
				}
			}
			assert(critical_mv < max_mv);
			return {new BatteryCellWidget(x, y, critical_mv, max_mv, num_cells, tpl, (uint)matchers.size()), matchers};
		} else if (type == "PopupWidget") {
			auto timeout_ms = widget_j.at("timeout_ms").template get<uint>();
			return {new PopupWidget(x, y, timeout_ms, (uint)matchers.size()), matchers};
		} else if (type == "DebugWidget") {
			return {new DebugWidget(x, y, (uint)matchers.size()), matchers};
		} else if (type == "HeadingTapeWidget") {
			int w    = widget_j.contains("width")            ? widget_j.at("width").get<int>()            : 400;
			int h    = widget_j.contains("height")           ? widget_j.at("height").get<int>()           : 50;
			int r    = widget_j.contains("range")            ? widget_j.at("range").get<int>()            : 120;
			std::string lpos = widget_j.contains("label_position") ? widget_j.at("label_position").get<std::string>() : "top";
			bool sdeg = widget_j.contains("show_degrees")    ? widget_j.at("show_degrees").get<bool>()   : false;
			int dint = widget_j.contains("degree_interval")  ? widget_j.at("degree_interval").get<int>() : 30;
			int tch  = widget_j.contains("tick_cardinal_h")       ? widget_j.at("tick_cardinal_h").get<int>()       : 30;
			int tih  = widget_j.contains("tick_intercardinal_h")  ? widget_j.at("tick_intercardinal_h").get<int>()  : 20;
			int tmh  = widget_j.contains("tick_medium_h")         ? widget_j.at("tick_medium_h").get<int>()         : 12;
			int tsh  = widget_j.contains("tick_small_h")          ? widget_j.at("tick_small_h").get<int>()          : 6;
			// Center indicator color (default: gold)
			double cr_ = 1.0, cg_ = 0.85, cb_ = 0.0;
			if (widget_j.contains("center_color")) {
				auto cc = widget_j.at("center_color");
				cr_ = cc.value("r", 1.0); cg_ = cc.value("g", 0.85); cb_ = cc.value("b", 0.0);
			}
			// Home direction triangle (default: green, disabled)
			bool show_home = widget_j.value("show_home", false);
			double hr_ = 0.0, hg_ = 0.85, hb_ = 0.2;
			if (widget_j.contains("home_color")) {
				auto hc = widget_j.at("home_color");
				hr_ = hc.value("r", 0.0); hg_ = hc.value("g", 0.85); hb_ = hc.value("b", 0.2);
			}
			return {new HeadingTapeWidget(x, y, w, h, r, lpos, sdeg, dint, tch, tih, tmh, tsh,
			                             cr_, cg_, cb_, show_home, hr_, hg_, hb_), matchers};
		} else if (type == "CompassWidget") {
			int rad = widget_j.value("radius", 80);
			std::string cmode = widget_j.value("mode", "north_up");
			std::string icon  = widget_j.value("icon", "arrow");
			// Icon color (default: red)
			double ir = 0.9, ig = 0.15, ib = 0.15;
			if (widget_j.contains("icon_color")) {
				auto ic = widget_j.at("icon_color");
				ir = ic.value("r", 0.9); ig = ic.value("g", 0.15); ib = ic.value("b", 0.15);
			}
			// Home marker (default: green)
			bool sh = widget_j.value("show_home", true);
			double hr2 = 0.0, hg2 = 0.85, hb2 = 0.2;
			if (widget_j.contains("home_color")) {
				auto hc = widget_j.at("home_color");
				hr2 = hc.value("r", 0.0); hg2 = hc.value("g", 0.85); hb2 = hc.value("b", 0.2);
			}
			// Wind marker (default: cyan)
			bool sw = widget_j.value("show_wind", true);
			double wr = 0.2, wg = 0.75, wb = 1.0;
			if (widget_j.contains("wind_color")) {
				auto wc = widget_j.at("wind_color");
				wr = wc.value("r", 0.2); wg = wc.value("g", 0.75); wb = wc.value("b", 1.0);
			}
			double bg_alpha = widget_j.value("bg_alpha", 0.75);
			return {new CompassWidget(x, y, rad, cmode, icon,
			                         ir, ig, ib, sh, hr2, hg2, hb2, sw, wr, wg, wb,
			                         bg_alpha, assets_dir), matchers};
		} else {
			spdlog::warn("Widget '{}': unknown type: {}", name, type);
			return {nullptr, {}};
		}
	}

	std::vector<Widget *> widgets;
	std::vector<std::tuple<FactMatcher, Widget *, uint>> matchers;
};


std::queue<Fact> fact_queue;
std::mutex mtx;
std::condition_variable cv;
pthread_mutex_t osd_mutex;

void modeset_paint_buffer(struct modeset_buf *buf, Osd *osd) {
	unsigned int j,k,off;
	cairo_t* cr;
	cairo_surface_t *surface;

	int osd_x = buf->width - 300;
	surface = cairo_image_surface_create_for_data(buf->map, CAIRO_FORMAT_ARGB32, buf->width, buf->height, buf->stride);
	cr = cairo_create (surface);

	// https://www.cairographics.org/FAQ/#clear_a_surface
	cairo_save(cr);
	cairo_set_operator(cr, CAIRO_OPERATOR_CLEAR);
	cairo_paint(cr);
	cairo_restore(cr);

	cairo_select_font_face (cr, "Roboto", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
	cairo_set_font_size (cr, 20);

	osd->draw(cr);

	cairo_fill(cr);
	cairo_destroy(cr);
	cairo_surface_destroy(surface);
}

int osd_thread_signal;

typedef struct png_closure
{
	unsigned char * iter;
	unsigned int bytes_left;
} png_closure_t;

cairo_status_t on_read_png_stream(png_closure_t * closure, unsigned char * data, unsigned int length)
{
	if(length > closure->bytes_left) return CAIRO_STATUS_READ_ERROR;
	
	memcpy(data, closure->iter, length);
	closure->iter += length;
	closure->bytes_left -= length;
	return CAIRO_STATUS_SUCCESS;
}
cairo_surface_t * surface_from_embedded_png(const char * png, size_t length)
{
	int rc = -1;
	png_closure_t closure[1] = {{
		.iter = (unsigned char *)png,
		.bytes_left = (unsigned int)length,
	}};
	return cairo_image_surface_create_from_png_stream(
		(cairo_read_func_t)on_read_png_stream,
		closure);
}


void my_flush_cb(lv_display_t * display, const lv_area_t * area, uint8_t * px_map)
{

    struct modeset_buf *buf1 = &p->out->osd_bufs[0];
    struct modeset_buf *buf2 = &p->out->osd_bufs[1];
	int ret = pthread_mutex_lock(&osd_mutex);
	assert(!ret);	
    if (px_map == buf1->map) {
		p->out->osd_buf_switch = 0;
    } else if (px_map == buf2->map) {
		p->out->osd_buf_switch = 1;
    } else {
        spdlog::error("Unknown buffer being flushed");
    }

	if (enable_live_colortrans) {
		p->out->osd_bufs[p->out->osd_buf_switch].gl_fb_id = osd_gl_process(&p->out->osd_bufs[p->out->osd_buf_switch], false); // LVGL: straight alpha
	}

	ret = pthread_mutex_unlock(&osd_mutex);
	assert(!ret);

	{
		struct modeset_buf *osd_buf = &p->out->osd_bufs[p->out->osd_buf_switch];
		if ((dvr_osd || display_reencode_osd) && frame_proc)
			frame_proc->set_osd_blend(osd_buf->prime_fd, osd_buf->width, osd_buf->height,
			                         osd_buf->stride / 4);
	}

	// tell the display thread that we have a update
	ret = pthread_mutex_lock(&video_mutex);
	assert(!ret);
	osd_update_ready = true;
	ret = pthread_cond_signal(&video_cond);
	assert(!ret);
	ret = pthread_mutex_unlock(&video_mutex);
	assert(!ret);

    /* IMPORTANT!!!
     * Inform LVGL that flushing is complete so buffer can be modified again. */
    lv_display_flush_ready(display);
}

uint32_t my_get_milliseconds() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

lv_display_t * display;
static lv_draw_buf_t lvgl_draw_buf1, lvgl_draw_buf2;

void setup_lvgl(osd_thread_params *p) {

	/* Initialize LVGL. */
    lv_init();

	// Get the first two buffers from the OSD buffers
	struct modeset_buf *buf1 = &p->out->osd_bufs[0];
	struct modeset_buf *buf2 = &p->out->osd_bufs[1];

	display = lv_display_create(buf1->width, buf1->height);
	lv_display_set_color_format(display, LV_COLOR_FORMAT_ARGB8888);

	// Pass the actual DRM pitch (buf->stride) so LVGL's row offsets match the
	// hardware-aligned scanline stride. lv_display_set_buffers() would recompute
	// stride as width*bpp and misalign on non-power-of-2 widths like 1366px.
	lv_draw_buf_init(&lvgl_draw_buf1, buf1->width, buf1->height,
	                 LV_COLOR_FORMAT_ARGB8888, buf1->stride, buf1->map, buf1->size);
	lv_draw_buf_init(&lvgl_draw_buf2, buf2->width, buf2->height,
	                 LV_COLOR_FORMAT_ARGB8888, buf2->stride, buf2->map, buf2->size);
	lv_display_set_draw_buffers(display, &lvgl_draw_buf1, &lvgl_draw_buf2);
	lv_display_set_render_mode(display, LV_DISPLAY_RENDER_MODE_DIRECT);

	lv_display_set_flush_cb(display, my_flush_cb);

	lv_tick_set_cb(my_get_milliseconds);

    lv_obj_set_style_bg_opa(lv_screen_active(), LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lv_layer_bottom(), LV_OPA_TRANSP, LV_PART_MAIN);

    // __DISPLAY_THREAD__ (main.cpp) starts well before this thread reaches
    // here (tid_display is created before tid_osd) and can call
    // set_no_signal_indicator() as soon as it detects a stale stream, which
    // can be on its very first iteration if no stream has decoded a frame
    // yet -- without this gate, that races lv_init() above and segfaults.
    g_lvgl_ready.store(true);
}

void *__OSD_THREAD__(void *param) {
	p = (osd_thread_params *)param;
	Osd *osd = new Osd;
	pthread_setname_np(pthread_self(), "__OSD");

	try {
		osd->loadConfig(p->config);
	} catch (const std::exception& e) {
		spdlog::error("OSD config load failed: {} — running with empty OSD", e.what());
	}
	auto last_display_at = std::chrono::steady_clock::now();

	int ret = pthread_mutex_init(&osd_mutex, NULL);
	assert(!ret);

	struct modeset_buf *buf = &p->out->osd_bufs[p->out->osd_buf_switch];
	ret = modeset_perform_modeset(p->fd, p->out, p->out->osd_request, &p->out->osd_plane,
								  buf->fb, buf->width, buf->height, osd_zpos);

	if (!osd_gl.init(p->fd, buf->width, buf->height,
						live_colortrans_gain, live_colortrans_offset)) {
		spdlog::warn("OSD GL: init failed");
	}

	if (gsmenu_enabled) {
		setup_lvgl(p);
		pp_menu_main();
	}

	while (!osd_thread_signal) {
		try {

		if (gsmenu_enabled) {
			handle_keyboard_input();
			lv_task_handler();
		}

		std::unique_lock<std::mutex> lock(mtx);
		std::vector<Fact> fact_buf;
		auto since_last_display = std::chrono::steady_clock::now() - last_display_at;
		auto wait = std::chrono::milliseconds(refresh_frequency_ms) - since_last_display;
		bool got_fact = cv.wait_for(
					lock,
					wait,
					[/*fact_queue*/] {
						return !fact_queue.empty();
					});
		if (got_fact) {
			// thread woke up because we got a new fact(s)
			// copy all the facts to the temporary buffer to unlock the queue ASAP
			for(; !fact_queue.empty(); fact_queue.pop()) {
				SPDLOG_DEBUG("got fact {}", fact_queue.front().asVerboseString());
				fact_buf.push_back(fact_queue.front());
			}
			lock.unlock();
			for (Fact fact : fact_buf) {
				osd->setFact(fact);
			}
			fact_buf.clear();
		} else {
			// thread woke up because of refresh timeout
			lock.unlock();

			if (! menu_active ) {
				SPDLOG_DEBUG("refresh OSD");
				int buf_idx = p->out->osd_buf_switch ^ 1;
				struct modeset_buf *buf = &p->out->osd_bufs[buf_idx];
				modeset_paint_buffer(buf, osd);

				if (enable_live_colortrans) {
					buf->gl_fb_id = osd_gl.process(buf, true); // Cairo: premultiplied alpha
				}

				int ret = pthread_mutex_lock(&osd_mutex);
				assert(!ret);
				p->out->osd_buf_switch = buf_idx;
				ret = pthread_mutex_unlock(&osd_mutex);
				assert(!ret);

				if ((dvr_osd || display_reencode_osd) && frame_proc)
					frame_proc->set_osd_blend(buf->prime_fd, buf->width, buf->height,
					                         buf->stride / 4);

				// tell the display thread that we have a update
				ret = pthread_mutex_lock(&video_mutex);
				assert(!ret);
				osd_update_ready = true;
				ret = pthread_cond_signal(&video_cond);
				assert(!ret);
				ret = pthread_mutex_unlock(&video_mutex);
				assert(!ret);

				last_display_at = std::chrono::steady_clock::now();
			} else {
				usleep(5000);
			}
		}

		} catch (const std::exception& e) {
			spdlog::error("OSD thread exception (continuing): {}", e.what());
		} catch (...) {
			spdlog::error("OSD thread unknown exception (continuing)");
		}
    }
	spdlog::info("OSD thread done.");
	return nullptr;
}

void mk_tags(osd_tag *tags, int n_tags, FactTags *fact_tags) {
	osd_tag tag;
	for (int i = 0; i < n_tags; i++) {
		tag = *tags++;
		fact_tags->emplace(tag.key, tag.val);
	}
}

void publish(Fact fact) {
	if (!enable_osd) return;
	//SPDLOG_DEBUG("post fact {}({})", fact.getName(), fact.getTags());
	{
		std::lock_guard<std::mutex> lock(mtx);
		fact_queue.push(fact);
	}
	cv.notify_one();
}

#ifdef __cplusplus
extern "C" {
#endif

// Batch APIs

void *osd_batch_init(uint n) {
	auto batch = new std::vector<Fact>;
	batch->reserve(n);
	return batch;
}
void osd_publish_batch(void *batch) {
	std::vector<Fact> *facts = static_cast<std::vector<Fact> *>(batch);
	if (enable_osd) {
		{
			std::lock_guard<std::mutex> lock(mtx);
			for (const Fact& fact : *facts) {
				// SPDLOG_DEBUG("batch post fact {}({})", fact.getName(), fact.getTags());
				fact_queue.push(fact);
			}
		}
		cv.notify_one();
	}
	delete facts;
};

void osd_add_bool_fact(void *batch, char const *name, osd_tag *tags, int n_tags, bool value) {
	std::vector<Fact> *facts = static_cast<std::vector<Fact> *>(batch);
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	facts->push_back(Fact(FactMeta(std::string(name), fact_tags), value));
};

void osd_add_int_fact(void *batch, char const *name, osd_tag *tags, int n_tags, long value) {
	std::vector<Fact> *facts = static_cast<std::vector<Fact> *>(batch);
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	facts->push_back(Fact(FactMeta(std::string(name), fact_tags), value));
};

void osd_add_uint_fact(void *batch, char const *name, osd_tag *tags, int n_tags, ulong value) {
	std::vector<Fact> *facts = static_cast<std::vector<Fact> *>(batch);
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	facts->push_back(Fact(FactMeta(std::string(name), fact_tags), value));
};

void osd_add_double_fact(void *batch, char const *name, osd_tag *tags, int n_tags, double value) {
	std::vector<Fact> *facts = static_cast<std::vector<Fact> *>(batch);
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	facts->push_back(Fact(FactMeta(std::string(name), fact_tags), value));
};

void osd_add_str_fact(void *batch, char const *name, osd_tag *tags, int n_tags, const char *value) {
	std::vector<Fact> *facts = static_cast<std::vector<Fact> *>(batch);
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	facts->push_back(Fact(FactMeta(std::string(name), fact_tags), std::string(value)));
};


// Individual APIs

void osd_publish_bool_fact(char const *name, osd_tag *tags, int n_tags, bool value) {
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	publish(Fact(FactMeta(std::string(name), fact_tags), value));
};

void osd_publish_int_fact(char const *name, osd_tag *tags, int n_tags, long value) {
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	publish(Fact(FactMeta(std::string(name), fact_tags), value));
};

void osd_publish_uint_fact(char const *name, osd_tag *tags, int n_tags, ulong value) {
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	publish(Fact(FactMeta(std::string(name), fact_tags), value));
};

void osd_publish_double_fact(char const *name, osd_tag *tags, int n_tags, double value) {
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	publish(Fact(FactMeta(std::string(name), fact_tags), value));
};

void osd_publish_str_fact(char const *name, osd_tag *tags, int n_tags, const char *value) {
	FactTags fact_tags;
	mk_tags(tags, n_tags, &fact_tags);
	publish(Fact(FactMeta(std::string(name), fact_tags), std::string(value)));
};

uint32_t osd_gl_process(struct modeset_buf* buf, bool premultiplied){
	return osd_gl.process(buf, premultiplied);
}

#ifdef __cplusplus
}
#endif


//
// Code below is only for unit-tests!
//
#ifdef TEST

TestExpressionTree::TestExpressionTree() {
    tree = new ExpressionTree();
}

TestExpressionTree::TestExpressionTree(const std::string& expression) {
    tree = new ExpressionTree(expression);
}

TestExpressionTree::~TestExpressionTree() {
    delete tree;
}

std::vector<std::string> TestExpressionTree::tokenize(const std::string& input) {
    return tree->tokenize(input);
}

void TestExpressionTree::parse(const std::string &expression) {
    tree->parse(expression);
}

double TestExpressionTree::evaluate(double xValue) {
    return tree->evaluate(xValue);
}



TestTplTextWidget::TestTplTextWidget(int pos_x, int pos_y, std::string tpl, uint n_args) {
    widget = new TplTextWidget(pos_x, pos_y, tpl, n_args);
}
TestTplTextWidget::~TestTplTextWidget() {
    delete widget;
}
void TestTplTextWidget::setBoolFact(uint idx, bool v) {
    Fact fact = Fact(FactMeta("bool"), v);
    widget->setFact(idx, fact);
};
void TestTplTextWidget::setLongFact(uint idx, long v) {
    Fact fact = Fact(FactMeta("long"), v);
    widget->setFact(idx, fact);
};
void TestTplTextWidget::setUlongFact(uint idx, ulong v) {
    Fact fact = Fact(FactMeta("ulong"), v);
    widget->setFact(idx, fact);
};
void TestTplTextWidget::setDoubleFact(uint idx, double v) {
    Fact fact = Fact(FactMeta("double"), v);
    widget->setFact(idx, fact);
};
void TestTplTextWidget::setStringFact(uint idx, std::string v) {
    Fact fact = Fact(FactMeta("string"), v);
    widget->setFact(idx, fact);
};

void TestTplTextWidget::draw(void *cr) {
    widget->draw((cairo_t *) cr);
}

std::unique_ptr<std::string> TestTplTextWidget::render_tpl() {
    return widget->render_tpl();
}

#endif
