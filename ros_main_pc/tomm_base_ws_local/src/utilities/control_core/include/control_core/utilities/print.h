/*! \file
 *
 * \author Simon Armleder
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */


#ifndef CONTROL_CORE_PRINT_H
#define CONTROL_CORE_PRINT_H

#include <ros/console.h>
#include <iostream>

// color option enum
typedef enum {P_RED, P_GREEN, P_BLUE, P_CYAN, P_MAGENTA, P_YELLOW, P_NONE} P_COLORS;

// color codes for ros terminal (TEXT)
#define P_RED_CODE          "\033[1;31m"
#define P_GREEN_CODE        "\033[1;92m"
#define P_BLUE_CODE         "\033[1;94m"
#define P_CYAN_CODE         "\033[1;96m"
#define P_MAGENTA_CODE      "\033[1;95m"
#define P_YELLOW_CODE       "\033[1;93m"
#define P_WHITE_CODE        "\033[1;97m"
#define P_BLACK_CODE        "\033[1;90m"

// color codes for ros terminal (BACKGROUND)
#define P_RED_CODE_BG       "\033[1;41;97m"         // 1 - bold, 41 - red bg, 97 - white text
#define P_YELLOW_CODE_BG    "\033[1;43;97m"         // 1 - bold, 43 - yel bg, 97 - white text

// selection of color code depending on the upper enum
#define COLOR_HELPER(col)\
    (\
      col==P_RED?P_RED_CODE:\
      col==P_GREEN?P_GREEN_CODE:\
      col==P_BLUE?P_BLUE_CODE:\
      col==P_CYAN?P_CYAN_CODE:\
      col==P_MAGENTA?P_MAGENTA_CODE:\
      col==P_YELLOW?P_YELLOW_CODE:\
      ""\
    )

// prettier printing functions
namespace control_core
{
  /**
   * @brief extract the class method name
   * 
   * @param pretty_function 
   * @return std::string 
   */
  inline std::string class_method_name_to_str(const std::string& pretty_function)
  {
    size_t begin,end;
    end = pretty_function.find("(");
    begin = pretty_function.substr(0,end).rfind(" ") + 1;
    end -= begin;
    return pretty_function.substr(begin,end) + "()";
  }

  inline std::string module_name_to_str(const std::string& module_name, const std::string& function_name)
  {
    return module_name + "::" + function_name + "(): ";
  }

  template<typename ... Args>
  std::string string_format(const std::string& format, Args ... args )
  {
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1;
    auto size = static_cast<size_t>( size_s );
    std::vector<char> buf(size);
    std::snprintf( buf.data(), size, format.c_str(), args ... );
    return std::string( buf.data(), buf.data() + size - 1 );
  }
  template<>
  inline std::string string_format(const std::string& format)
  {
    return format;
  }

}

// normal prints for modules
#define PRINT_INFO(...)                         ROS_INFO_STREAM(control_core::module_name_to_str(this->name(), __FUNCTION__) << control_core::string_format(__VA_ARGS__))
#define PRINT_WARN(...)                         ROS_WARN_STREAM(control_core::module_name_to_str(this->name(), __FUNCTION__) << control_core::string_format(__VA_ARGS__))
#define PRINT_ERROR(...)                        ROS_ERROR_STREAM(control_core::module_name_to_str(this->name(), __FUNCTION__) << control_core::string_format(__VA_ARGS__))

#define PRINT_INFO_STREAM(str)                  ROS_INFO_STREAM(control_core::module_name_to_str(this->name(), __FUNCTION__)<<str)
#define PRINT_WARN_STREAM(str)                  ROS_WARN_STREAM(control_core::module_name_to_str(this->name(), __FUNCTION__)<<str)
#define PRINT_ERROR_STREAM(str)                 ROS_ERROR_STREAM(control_core::module_name_to_str(this->name(), __FUNCTION__)<<str)

// colored prints for modules
#define PRINT_INFO_STREAM_COL(str, col)         ROS_INFO_STREAM(COLOR_HELPER(col)<<control_core::module_name_to_str(this->name(), __FUNCTION__)<<str)
#define PRINT_WARN_STREAM_COL(str, col)         ROS_WARN_STREAM(P_YELLOW_CODE_BG<<(col!=P_YELLOW?COLOR_HELPER(col):"")<<control_core::module_name_to_str(this->name(), __FUNCTION__)<<str)
#define PRINT_ERROR_STREAM_COL(str, col)        ROS_ERROR_STREAM(P_RED_CODE_BG<<(col!=P_RED?COLOR_HELPER(col):"")<<control_core::module_name_to_str(this->name(), __FUNCTION__)<<str)

// assert function with message
#define ROS_ERROR_ASSERT(expr, str) \
    do { \
        if (!(expr)) { \
            ROS_ERROR_STREAM(str); \
            std::exit(EXIT_FAILURE); \
        } \
    } while (false)

#define PRINT_ASSERT(expr, str) \
    do { \
        if (!(expr)) { \
            PRINT_ERROR_STREAM(str); \
            std::exit(EXIT_FAILURE); \
        } \
    } while (false)

#endif