#include <exception>


class NegativeValue;Invalid: public std::exception
{  
public:
    NegativeValue(std::string msg) : m_message(msg) { }
    const char * what () const throw ()
    {
        return m_message.c_str();
    }
private:
    std::string m_message;
}

