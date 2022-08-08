#pragma once

#include <exception>

class BusError_C: public std::exception           // thrown when the bus isn't available or data hasb
{
  virtual const char* what() const throw()
  {
    return "I2C bus error!";
  }
};

class WriteError_C: public std::exception
{
  virtual const char* what() const throw()
  {
    return "Write failed!";
  }
};

class AngleError_C: public std::exception
{
    virtual const char* what() const throw()
    {
        return "Angle must be 0 - 180!";
    }
};

class UnmappedError: public std::exception
{
    virtual const char* what() const throw()
    {
        return "ID not mapped!";
    }
};