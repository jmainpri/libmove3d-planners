#ifndef NUMSANDSTRINGS_HPP
#define NUMSANDSTRINGS_HPP

#include <string>
#include <vector>
#include <sstream>

template <typename T>
std::string num_to_string ( T Number )
{
    std::ostringstream ss;
    ss << Number;
    return ss.str();
}

template <class T>
bool convert_text_to_num(T& t,
                 const std::string& s,
                 std::ios_base& (*f)(std::ios_base&))
{
  std::istringstream iss(s);
  return !(iss >> f >> t).fail();
}

#endif // NUMSANDSTRINGS_HPP
