#include <ros/exception.h>


class NegativeValueInvalid: public std::exception
{
public:
    NegativeValue(std::string msg) : m_message(msg) { }
    const char * what () const throw () {};
private:
    std::string m_message;
}