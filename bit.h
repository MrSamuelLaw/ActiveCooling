#ifndef BIT_H
#define BIT_H

#include <Arduino.h>

/*
* Basic ONS class
*/
class ONS {
  protected:
    bool _bLast;

  public:
    
    /*
    * @param input, sets the intial scan value
    */
    ONS(bool input)
      : _bLast{input} 
    {
      // empty constructor
    }

    /*
    * @param input, input signal that causes the ONS
    * call() to return true on a rising edge
    */
    bool call(bool input)
    {
      bool temp = input & !this->_bLast;
      this->_bLast = input;
      return temp;
    }
};

#endif