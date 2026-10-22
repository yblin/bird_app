//
// Copyright 2014-2025 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_BASE_MESSAGE_H_
#define CODELIBRARY_BASE_MESSAGE_H_

#include <algorithm>
#include <cassert>
#include <set>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "codelibrary/base/format.h"

// The width of the screen (the maximum characters in one line).
// MESSAGE_MAX_CHARACTERS_PER_LINE <= 0 means that the screen width is infinity.
#ifndef MESSAGE_MAX_CHARACTERS_PER_LINE
#define MESSAGE_MAX_CHARACTERS_PER_LINE 120
#endif

// For a one-dimensional array, the default maximum number of elements that
// Message will output.
// MESSAGE_MAX_ELEMENTS_PER_ARRAY = 0 means that all elements will be output.
#ifndef MESSAGE_MAX_ELEMENTS_PER_ARRAY
#define MESSAGE_MAX_ELEMENTS_PER_ARRAY 10
#endif

namespace cl {

/**
 * Message class stream information for various objects.
 *
 * Why need Message?
 *  1. STL formating library do not support pointers and array.
 *  2. It is easier to use.
 *  3. It supports aligned formating for matrices or tensors.
 *
 * Sample usage;
 *
 *   bool flag = true;
 *   Message foo;
 *   foo << "flag", " = ", flag;
 *   std::cout << foo;
 *
 * The above code will print "flag = true".
 */
class Message {
    struct MaxElementsFlag {
        explicit MaxElementsFlag(int n) : max_elements(n) {}
        int max_elements;
    };

    template <typename T, typename = void>
    struct HasStreamOperator : std::false_type {};

    template <typename T>
    struct HasStreamOperator<T, decltype(void(std::declval<std::ostream>() <<
                                              std::declval<T>()), void())>
                             : std::true_type {};

public:
    /**
     * Constructs an empty Message.
     */
    Message() = default;

    /**
     * Construct a message from sequence [first, last) in aligned manner.
     *
     * This function is especially useful for printing matrices and vectors.
     * It can greatly improve human readability.
     *
     * For example,
     *   [ 1,  2,  3,
     *     4,  5,  6,
     *     7,  8,  9,
     *    10, 11, 12]
     *
     * is much better than
     *   [1, 2, 3,
     *   4, 5, 6,
     *   7, 8, 9,
     *   10, 11, 12]
     */
    template <typename Iterator,
        typename = typename std::enable_if<std::is_convertible<
                   typename std::iterator_traits<Iterator>::iterator_category,
                            std::input_iterator_tag>::value>::type>
    Message(Iterator first, Iterator last) {
        Append(first, last);
    }

    /**
     * Construct a message from N-D array in aligned manner.
     */
    template <typename Iterator,
          typename = typename std::enable_if<std::is_convertible<
                     typename std::iterator_traits<Iterator>::iterator_category,
                              std::input_iterator_tag>::value>::type>
    Message(Iterator first, Iterator last, const std::vector<int>& shape) {
        Append(first, last, shape);
    }

    /**
     * Construct a message of an object.
     */
    template <typename T>
    explicit Message(const T& object) {
        Append(object);
    }

    /**
     * Message of STL set.
     */
    template <typename T>
    explicit Message(const std::set<T>& s)
        : Message(s.begin(), s.end()) {}

    /**
     * Construct a message of a list.
     */
    template <typename T, typename... Args>
    Message(const T& t1, const Args&... t2) {
        Append(t1, t2...);
    }

    /**
     * Check if the message is multi-line or single line.
     */
    bool IsMultiLine() const {
        return text_.find('\n') != std::string::npos;
    }

    /**
     * Append STL pair to the message.
     */
    template <class T1, class T2>
    Message& Append(const std::pair<T1, T2>& p) {
        Append(p.first, p.second);
        return *this;
    }

    /**
     * Append the object that support ostream (operator<<).
     */
    template <typename T>
    typename std::enable_if<HasStreamOperator<T>::value, Message&>::type
    Append(const T& object) {
        return AppendWithStreamOperator(object);
    }

    /**
     * Append the object that do not support ostream (operator<<).
     */
    template <typename T>
    typename std::enable_if<!HasStreamOperator<T>::value, Message&>::type
    Append(const T& object) {
        return AppendWithoutStreamOperator(object);
    }

    /**
     * Instead of 1/0, we want to see true/false for bool values.
     */
    Message& Append(bool b) {
        text_ += (b ? "true" : "false");
        return *this;
    }

    Message& Append(uint8_t v) {
        text_ += ToString(v);
        return *this;
    }
    Message& Append(int8_t v) {
        text_ += ToString(v);
        return *this;
    }
    Message& Append(uint16_t v) {
        text_ += ToString(v);
        return *this;
    }
    Message& Append(int16_t v) {
        text_ += ToString(v);
        return *this;
    }
    Message& Append(int32_t v) {
        text_ += ToString(v);
        return *this;
    }
    Message& Append(uint32_t v) {
        text_ += ToString(v);
        return *this;
    }
    Message& Append(int64_t v) {
        text_ += ToString(v);
        return *this;
    }
    Message& Append(uint64_t v) {
        text_ += ToString(v);
        return *this;
    }
    Message& Append(float v) {
        text_ += ToString(v);
        return *this;
    }
    Message& Append(double v) {
        text_ += ToString(v);
        return *this;
    }

    /**
     * Return the richer information for pointer.
     */
    template <typename T>
    Message& Append(const T* pointer) {
        if (pointer == nullptr) {
            text_ += "(nullptr)";
        } else {
            text_ += Format("{:#x}", reinterpret_cast<size_t>(pointer));
        }
        return *this;
    }
    template <typename T>
    Message& Append(T* pointer) {
        if (pointer == nullptr) {
            text_ += "(nullptr)";
        } else {
            text_ += Format("{:#x}", reinterpret_cast<size_t>(pointer));
        }
        return *this;
    }

    /**
     * When the input value is a std::string or std::wstring object, each NULL
     * character in it is replaced with "\\0".
     */
    Message& Append(const std::string& data) {
        std::string str;
        for (size_t i = 0; i < data.size(); ++i) {
            if (data[i] == '\0') {
                str += "\\0";
            } else {
                str += data[i];
            }
        }
        text_ += str;
        return *this;
    }
    Message& Append(char* data) {
        return Append(std::string(data));
    }
    Message& Append(const char* data) {
        return Append(std::string(data));
    }

    /**
     * Convert a wide string to a narrow string in UTF-8 encoding.
     * The wide string is assumed to have the following encoding:
     *   UTF-16 if sizeof(wchar_t) == 2 (on Windows, Cygwin, Symbian OS)
     *   UTF-32 if sizeof(wchar_t) == 4 (on Linux)
     *
     * Note If the string contains code points that are not valid Unicode code
     * points (i.e. outside of Unicode range U+0 to U+10FFFF) they will be
     * output as '(Invalid Unicode 0xXXXXXXXX)'.
     * If the string is in UTF16 encoding and contains invalid UTF-16 surrogate
     * pairs, values in those pairs will be encoded as individual Unicode
     * characters from Basic Normal Plane.
     */
    Message& Append(const std::wstring& str) {
        size_t len = str.size();

        std::string result;
        for (size_t i = 0; i < len; ++i) {
            uint32_t code_point;

            if (str[i] == L'\0') break;

            if (i + 1 < len && sizeof(wchar_t) == 2 &&
                (str[i] & 0xFC00) == 0xD800 &&
                (str[i + 1] & 0xFC00) == 0xDC00) {
                const uint32_t mask = (1 << 10) - 1;
                code_point = (((str[i] & mask) << 10) |
                              (str[i + 1] & mask)) + 0x10000;
                ++i;
            } else {
                code_point = static_cast<uint32_t>(str[i]);
            }

            result += UnicodeCodePointToUtf8(code_point);
        }

        text_ += result;
        return *this;
    }
    Message& Append(wchar_t* data) {
        return Append(std::wstring(data));
    }
    Message& Append(const wchar_t* data) {
        return Append(std::wstring(data));
    }
    Message& Append(const Message& msg) {
        return Append(msg.text());
    }

    /**
     * Append the sequence [first, last) to the message in an aligned manner.
     */
    template <typename Iterator,
        typename = typename std::enable_if<std::is_convertible<
                   typename std::iterator_traits<Iterator>::iterator_category,
                            std::input_iterator_tag>::value>::type>
    Message& Append(Iterator first, Iterator last) {
        std::vector<std::string> terms;
        size_t width = GetTerms(first, last, &terms);
        if (terms.empty()) {
            text_ += "[]";
            return *this;
        }

        // Check if the sequence can be shown in one line.
        size_t screen_w = MESSAGE_MAX_CHARACTERS_PER_LINE > 0 ?
                    MESSAGE_MAX_CHARACTERS_PER_LINE :
                    std::numeric_limits<size_t>::max();

        size_t length = 0;
        for (const std::string& term : terms)
            length += term.size() + 2;
        if (length <= screen_w) {
            text_ += Format("[{}]", Join(terms, ", "));
            return *this;
        }

        // Otherwise, we split terms into multi lines.
        std::vector<std::string> lines;
        Split(terms, width, screen_w, &lines);
        text_ += Format("{}", Join(lines, "\n"));

        return *this;
    }

    /**
     * Append an N-dimensional array to the message in aligned manner.
     */
    template <typename Iterator,
        typename = typename std::enable_if<std::is_convertible<
                   typename std::iterator_traits<Iterator>::iterator_category,
                            std::input_iterator_tag>::value>::type>
    Message& Append(Iterator first, Iterator last,
                    const std::vector<int>& shape) {
        if (shape.empty()) {
            text_ += "[]";
            return *this;
        }

        auto n = std::distance(first, last);
        for (int s : shape) {
            assert(s != 0);
            assert(n % s == 0);
            n /= s;
        }
        assert(n == 1 && "The shape is not matched to [first, last).");

        std::vector<int> strides(shape.size());
        strides.back() = 1;
        for (size_t i = shape.size() - 1; i > 0; --i) {
            strides[i - 1] = strides[i] * shape[i];
        }

        size_t width = ComputeWidth(first, shape, strides, 0);
        return AppendND(first, shape, strides, 0, width);
    }

    /**
     * Append a tuple to the message.
     */
    template <typename T, typename... Args>
    Message& Append(const T& t1, const Args&... t2) {
        Append("(");
        AppendUnpack(t1, t2...);
        Append(")");
        return *this;
    }

    /**
     * Allow that Message() << "fdsa", 12.
     */
    template <typename T>
    Message& operator,(const T& t1) {
        Append(" ");
        Append(t1);
        return *this;
    }

    /**
     * Using opeartor << Message::SetMaxElements(n) to set the max elements for
     * printing.
     */
    static MaxElementsFlag MaxElements(int n) {
        assert(n >= 0);

        return MaxElementsFlag(n);
    }

    /**
     * Clear the message.
     */
    void clear() {
        text_.clear();
    }

    bool empty() const {
        return text_.empty();
    }

    /**
     * Return the internal text data.
     */
    const std::string& text() const {
        return text_;
    }

    /**
     * Convert message to string.
     */
    const std::string& to_string() const {
        return text_;
    }

    /**
     * Return the maximum elements in the sequence will be recorded.
     */
    int max_elements() const {
        return max_elements_;
    }

    /**
     * Set the maximum number of elements in sequence that need to be recorded.
     *
     * max_elements = 0 means that all elements will be recorded. The default
     * value is 10.
     */
    void set_max_elements(int max_elements) {
        assert(max_elements >= 0);

        max_elements_ = max_elements;
    }

    template <typename T>
    Message& operator <<(const T& v)                 { return Append(v); }
    Message& operator <<(bool v)                     { return Append(v); }
    template <typename T>
    Message& operator <<(const T* v)                 { return Append(v); }
    Message& operator <<(const std::string& v)       { return Append(v); }
    Message& operator <<(char* v)                    { return Append(v); }
    Message& operator <<(const char* v)              { return Append(v); }
    Message& operator <<(const std::wstring& v)      { return Append(v); }
    Message& operator <<(wchar_t* v)                 { return Append(v); }
    Message& operator <<(const wchar_t* v)           { return Append(v); }
    template <typename T1, typename T2>
    Message& operator <<(const std::pair<T1, T2>& v) { return Append(v); }
    Message& operator <<(const Message& v)           { return Append(v); }

    Message& operator <<(const MaxElementsFlag& m) {
        max_elements_ = m.max_elements;
        return *this;
    }

    /**
     * Support other logging library.
     */
    friend std::ostream& operator <<(std::ostream& os, const Message& msg) {
        if (msg.IsMultiLine()) os << std::endl;
        os << msg.text();
        return os;
    }

    friend auto format_as(const Message& msg) {
        return Format("{}", msg.text_);
    }

private:
    /**
     * Handling for types that support operator<<
     */
    template <typename T>
    Message& AppendWithStreamOperator(const T& object) {
        std::stringstream ss;
        ss << object;      // Stream the object to a stringstream
        text_ += ss.str(); // Append the string to text_
        return *this;
    }

    /**
     * Handling for types that do not support operator<<.
     */
    template <typename T>
    Message& AppendWithoutStreamOperator(const T& object) {
        auto bytes = reinterpret_cast<const unsigned char*>(&object);

        int count = sizeof(object);

        // Tell the user how big the object is.
        text_ += Format("{}-byte object <", count);

        int i = 0;
        for (; i < std::min(128, count); ++i) {
            if (i != 0) {
                // Organize the bytes into groups of 2 for easy parsing by
                // human.
                if ((i % 2) == 0)
                    text_ += ' ';
                else
                    text_ += '-';
            }
            text_ += Format("{:02x}", bytes[i]);
        }
        if (i < count) text_ += " ... ";
        text_ += ">";
        return *this;
    }

    /**
     * Initialize the message of a ND array recursively.
     */
    template <typename Iterator>
    Message& AppendND(Iterator first,
                      const std::vector<int>& shape,
                      const std::vector<int>& strides,
                      size_t depth,
                      size_t width) {
        if (depth == shape.size()) return *this;

        int n = shape[depth];
        if (depth + 1 == shape.size()) {
            // Info of 1D sequence.
            size_t screen_width = MESSAGE_MAX_CHARACTERS_PER_LINE > 0 ?
                        MESSAGE_MAX_CHARACTERS_PER_LINE :
                        std::numeric_limits<size_t>::max();
            if (screen_width < depth + depth + 10) {
                screen_width = 10;
            } else {
                screen_width -= depth + depth;
            }

            std::vector<std::string> terms;
            Iterator p = first;
            std::advance(p, n);
            GetTerms(first, p, &terms);

            std::string space(depth, ' ');
            std::vector<std::string> lines;
            Split(terms, width, screen_width, &lines);
            for (size_t i = 1; i < lines.size(); ++i) {
                lines[i] = space + lines[i];
            }
            text_ += Format("{}", Join(lines, "\n"));
            return *this;
        }

        std::string space(depth + 1, ' ');
        text_ += "[";

        if (n <= max_elements_ || max_elements_ == 0) {
            Iterator p = first;
            for (int i = 0; i < n; ++i) {
                if (i != 0) text_ += space;
                AppendND(p, shape, strides, depth + 1, width);
                std::advance(p, strides[depth]);
                if (i + 1 < n) {
                    text_ += ",\n";
                    if (depth + 2 < shape.size()) text_ += "\n";
                }
            }
        } else {
            Iterator p = first;
            for (int i = 0; i < (max_elements_ + 1) / 2; ++i) {
                if (i != 0) text_ += space;
                AppendND(p, shape, strides, depth + 1, width);
                std::advance(p, strides[depth]);
                text_ += ",\n";
                if (depth + 2 < shape.size()) text_ += '\n';
            }

            text_ += space;
            text_ += "...,\n";
            if (depth + 2 < shape.size()) text_ += "\n";

            p = first;
            std::advance(p, strides[depth] * (n - max_elements_ / 2));
            for (int i = n - max_elements_ / 2; i < n; ++i) {
                text_ += space;
                AppendND(p, shape, strides, depth + 1, width);
                std::advance(p, strides[depth]);
                if (i + 1 < n) {
                    text_ += ",\n";
                    if (depth + 2 < shape.size()) text_ += '\n';
                }
            }
        }

        text_ += "]";
        return *this;
    }

    /**
     * Append a variable number of objects to the message.
     */
    template <typename T, typename... Args>
    Message& AppendUnpack(const T& first_arg, const Args&... args) {
        this->Append(first_arg);
        text_ += ", ";
        return AppendUnpack(args...);
    }
    template <typename T>
    Message& AppendUnpack(const T& last_arg) {
        this->Append(last_arg);
        return *this;
    }

    /**
     * Get the string messages of various objects.
     */
    template <typename T>
    std::string Info(const T& t) const {
        Message msg;
        msg.set_max_elements(max_elements_);
        msg.Append(t);
        return msg.text();
    }

    /**
     * Convert a sequence [first, last) to string terms.
     *
     * Return the max width in terms.
     */
    template <typename Iterator>
    size_t GetTerms(Iterator first, Iterator last,
                    std::vector<std::string>* terms) const {
        terms->clear();
        if (first == last) return 0;

        auto n = std::distance(first, last);

        size_t width = 0;
        if (max_elements_ >= n || max_elements_ == 0) {
            for (Iterator p = first; p != last; ++p) {
                terms->push_back(Info(*p));
                width = std::max(width, terms->back().size());
            }
        } else {
            Iterator p = first;
            for (int k = 0; k < (max_elements_ + 1) / 2; ++k, ++p) {
                terms->push_back(Info(*p));
                width = std::max(width, terms->back().size());
            }
            terms->push_back("...");
            std::advance(p, n - max_elements_);
            for (; p != last; ++p) {
                terms->push_back(Info(*p));
                width = std::max(width, terms->back().size());
            }
        }
        return width;
    }

    /**
     * Recursively find a suitable width to stream the N-D array .
     */
    template <typename Iterator>
    size_t ComputeWidth(Iterator first,
                        const std::vector<int>& shape,
                        const std::vector<int>& strides,
                        size_t depth) const {
        if (depth == shape.size()) return 0;

        int n = shape[depth];
        if (depth + 1 == shape.size()) {
            std::vector<std::string> terms;
            Iterator last = first;
            std::advance(last, n);
            return GetTerms(first, last, &terms);
        }

        size_t width = 0;
        if (n <= max_elements_ || max_elements_ == 0) {
            Iterator p = first;
            for (int i = 0; i < n; ++i) {
                size_t w = ComputeWidth(p, shape, strides, depth + 1);
                std::advance(p, strides[depth]);
                width = std::max(width, w);
            }
        } else {
            Iterator p = first;
            for (int i = 0; i < (max_elements_ + 1) / 2; ++i) {
                size_t w = ComputeWidth(p, shape, strides, depth + 1);
                std::advance(p, strides[depth]);
                width = std::max(width, w);
            }
            p = first;
            std::advance(p, strides[depth] * (n - max_elements_ / 2));
            for (int i = n - max_elements_ / 2; i < n; ++i) {
                size_t w = ComputeWidth(p, shape, strides, depth + 1);
                std::advance(p, strides[depth]);
                width = std::max(width, w);
            }
        }

        return width;
    }

    /**
     * Split terms into multi-lines with the specific width.
     */
    void Split(const std::vector<std::string>& terms,
               size_t width,
               size_t screen_width,
               std::vector<std::string>* lines) const {
        lines->clear();
        if (terms.empty()) {
            lines->push_back("[]");
            return;
        }

        std::string str = "[";
        size_t leading = 1;

        for (size_t i = 0; i < terms.size(); ++i) {
            std::string term = terms[i];
            if (term.size() < width) {
                std::string space(width - term.size(), ' ');
                term = space + term;
            } else if (term.size() > width) {
                size_t n = width;
                while (n < term.size()) n += 2 + width;
                n -= term.size();
                std::string left_space(n / 2, ' ');
                std::string right_space(n - n / 2, ' ');
                term = left_space + term + right_space;
            }

            if (leading + term.size() + 2 < screen_width) {
                str += term;
                leading += term.size();
            } else {
                if (!str.empty()) lines->push_back(str);
                str = " " + term;
                leading = term.size() + 1;
            }

            if (i + 1 < terms.size()) {
                str += ", ";
                leading += 2;
            } else {
                str += "]";
                lines->push_back(str);
            }
        }
    }

    /**
     * Convert a Unicode code point to a narrow string in UTF-8 encoding.
     *
     * code_point parameter is of unsigned 32-bit integer type because wchar_t
     * may not be wide enough to contain a code point.
     *
     * A Unicode code-point can have up to 21 bits and is encoded in UTF-8 like
     * this:
     *
     * Code-point length   Encoding
     *   0 -  7 bits       0xxxxxxx
     *   8 - 11 bits       110xxxxx 10xxxxxx
     *  12 - 16 bits       1110xxxx 10xxxxxx 10xxxxxx
     *  17 - 21 bits       11110xxx 10xxxxxx 10xxxxxx 10xxxxxx
     *
     * If the code_point is not a valid Unicode code point
     * (i.e. outside of Unicode range U+0 to U+10FFFF) it will be converted to
     * "(Invalid Unicode 0xXXXXXXXX)".
     */
    static std::string UnicodeCodePointToUtf8(uint32_t c) {
        // The maximum code-point a one-byte UTF-8 sequence can represent.
        const uint32_t max_code_point1 = (static_cast<uint32_t>(1) << 7) - 1;

        // The maximum code-point a two-byte UTF-8 sequence can represent.
        const uint32_t max_code_point2 = (static_cast<uint32_t>(1) << 11) - 1;

        // The maximum code-point a three-byte UTF-8 sequence can represent.
        const uint32_t max_code_point3 = (static_cast<uint32_t>(1) << 16) - 1;

        // The maximum code-point a four-byte UTF-8 sequence can represent.
        const uint32_t max_code_point4 = (static_cast<uint32_t>(1) << 21) - 1;

        if (c > max_code_point4) {
            return Format("Invalid Unicode 0x{:X}", c);
        }

        char str[5]; // Big enough for the largest valid code point.
        if (c <= max_code_point1) {
            str[1] = '\0';
            str[0] = static_cast<char>(c);                         // 0xxxxxxx
        } else if (c <= max_code_point2) {
            str[2] = '\0';
            str[1] = static_cast<char>(0x80 | (c & 0x3F));         // 10xxxxxx
            str[0] = static_cast<char>(0xC0 | (c >> 6));           // 110xxxxx
        } else if (c <= max_code_point3) {
            str[3] = '\0';
            str[2] = static_cast<char>(0x80 | (c & 0x3F));         // 10xxxxxx
            str[1] = static_cast<char>(0x80 | ((c >> 6) & 0x3F));  // 10xxxxxx
            str[0] = static_cast<char>(0xE0 | (c >> 12));          // 1110xxxx
        } else {
            str[4] = '\0';
            str[3] = static_cast<char>(0x80 | (c & 0x3F));         // 10xxxxxx
            str[2] = static_cast<char>(0x80 | ((c >> 6) & 0x3F));  // 10xxxxxx
            str[1] = static_cast<char>(0x80 | ((c >> 12) & 0x3F)); // 10xxxxxx
            str[0] = static_cast<char>(0xF0 | (c >> 18));          // 11110xxx
        }

        return str;
    }

    /**
     * Join std::vector<std::string> with delimiter (legacy C++ compatible:
     * C++11+).
     */
    static std::string Join(const std::vector<std::string>& vec,
                            const std::string& delimiter) {
        std::string result;
        for (size_t i = 0; i < vec.size(); ++i) {
          result += vec[i];
          if (i != vec.size() - 1) {
            result += delimiter;
          }
        }
        return result;
    }

    // The maximum number of elements in a sequence that need to be recorded in
    // the message.
    // max_elements = 0 means that all elements will be recorded.
    int max_elements_ = MESSAGE_MAX_ELEMENTS_PER_ARRAY;

    // String to hold the text data.
    std::string text_;
};

/**
 * Return a failure message for ASSERT_EQ and CHECK_EQ.
 *
 * For example, for ASSERT_EQ(foo, bar) where foo is 5 and bar is 6,
 * we have:
 *
 *   expected_expression: "foo"
 *   actual_expression:   "bar"
 *   expected_value:      "5"
 *   actual_value:        "6"
 */
template <typename T1, typename T2>
std::string CheckEQFailureMessage(const char* actual_expression,
                                  const char* expected_expression,
                                  const T1& actual_value,
                                  const T2& expected_value) {
    Message msg1(actual_value), msg2(expected_value);
    std::string actual_str = msg1.text();
    std::string expected_str = msg2.text();

    std::string message = Format("EQ({}, {}) failed!\n", actual_expression,
                                 expected_expression);

    if (actual_str == actual_expression &&
        expected_str == expected_expression) {
        message += Format("{} != {}", actual_expression, expected_expression);
        return message;
    }

    if (actual_str == actual_expression) {
        auto format1 = msg1.IsMultiLine() ? Format("\n{}\n", expected_str)
                                          : Format("{}, ", expected_str);
        auto format2 = msg2.IsMultiLine() ? Format("\n{}\n", actual_str)
                                          : Format("{}", actual_str);

        message += Format("{} = {}not equal to {}", expected_expression,
                          format1, format2);
    } else if (expected_str == expected_expression) {
        auto format1 = msg2.IsMultiLine() ? Format("\n{}\n", actual_str)
                                          : Format("{}, ", actual_str);
        auto format2 = msg1.IsMultiLine() ? Format("\n{}\n", expected_str)
                                          : Format("{}", expected_str);
        message += Format("{} = {}not equal to {}", actual_expression,
                          format1, format2);
    } else {
        auto format1 = msg1.IsMultiLine() ? Format("\n{}\n", actual_str)
                                          : Format("{}, ", actual_str);
        auto format2 = msg2.IsMultiLine() ? Format("\n{} = \n{}\n",
                                                   expected_expression,
                                                   expected_str)
                                          : Format("{} = {}",
                                                   expected_expression,
                                                   expected_str);
        message += Format("{} = {}but {}", actual_expression, format1, format2);
    }

    return message;
}

/**
 * Return a failure message for ASSERT_EQ_NEAR and CHECK_EQ_NEAR.
 */
template <typename T>
std::string CheckEQNearFailureMessage(const char* a_expression,
                                      const char* b_expression,
                                      const char* epsilon_expression,
                                      const T& a,
                                      const T& b,
                                      const T& epsilon) {
    return Format("EQ_NEAR({}, {}, {}) failed!\n"
                  "The difference between {} and {} is {}, "
                  "which exceeds {}, where\n"
                  "{} evaluates to {},\n"
                  "{} evaluates to {}, and\n"
                  "{} evaluates to {}.",
                  a_expression,
                  b_expression,
                  epsilon_expression,
                  a_expression,
                  b_expression,
                  std::abs(a - b),
                  epsilon_expression,
                  a_expression,
                  a,
                  b_expression,
                  b,
                  epsilon_expression,
                  epsilon);
}

/**
 * Return a failure message for equality check such as ASSERT_EQ_RANGE and
 * CHECK_EQ_RANGE.
 */
template <typename T1, typename T2>
std::string CheckEQRangeFailureMessage(const char* actual_expression_first,
                                       const char* actual_expression_last,
                                       const char* expected_expression_first,
                                       const char* expected_expression_last,
                                       T1 actual_first,
                                       T1 actual_last,
                                       T2 expected_first,
                                       T2 expected_last) {
    Message a(actual_first, actual_last);
    Message b(expected_first, expected_last);

    return Format("EQ_RANGE({}, {}, {}, {}) failed!\n"
                  "Values of [{}, {}):\n"
                  "{}\n"
                  "Expected [{}, {}):\n"
                  "{}",
                  actual_expression_first,
                  actual_expression_last,
                  expected_expression_first,
                  expected_expression_last,
                  actual_expression_first,
                  actual_expression_last,
                  a.to_string(),
                  expected_expression_first,
                  expected_expression_last,
                  b.to_string());
}

} // namespace cl

#endif // CODELIBRARY_BASE_MESSAGE_H_
