#ifndef TIMING_H
#define TIMING_H

#include <Arduino.h>

/*
* Base class for TON and TOF
*/
class PLCTimer {
  protected:
    uint32_t _now{0};
    uint32_t _start{0};
    unsigned long (*_timingFunc)(void){nullptr};
    static constexpr uint32_t MAX_VALUE{4294967295};

  public:
    bool EN{false};
    bool DN{false};
    bool TT{false};
    uint32_t PRE{0};
    uint32_t ACC{0};

    /**
     * Allows users to pass in their own timing function
    */
    PLCTimer(unsigned long (*timingFunc)(void))
    {
      this->_timingFunc = timingFunc;
    }

    /*
    * Resets the timer
    */
    PLCTimer& reset()
    {
      this->DN = false;
      this->TT = false;
      this->ACC = 0;
      this->_start = {0};
      return *this;
    }
    
    // pure virtual function for derived classes
    virtual PLCTimer& call(bool enabled) = 0;
};


/*
* Class that replicates the functionality of an RSLogix TON instruction
* // ton test
* if(Serial.available() || ton.TT)
* {
*   ton.update(!ton.DN);
*   Serial.read();
*   Serial.print("EN= "); Serial.print(ton.EN);
*   Serial.print(" ACC="); Serial.print(ton.ACC);
*   Serial.print(" DN="); Serial.println(ton.DN);
* }
*/
class TON : public PLCTimer{
  public:
    TON(unsigned long (*timingFunc)(void))
      : PLCTimer{timingFunc}
    {
      // pass
    }

    TON(unsigned long (*timingFunc)(void), uint32_t pre) 
      : PLCTimer(timingFunc)
    {
      // used for instiating in a PID constructor
      PRE = pre;
    }

    /*
    * Function to be called to update the timer in the event loop.
    * returns the ton instance.
    */
    TON& call(bool enabled)
    {
      this->EN = enabled;
      // check if the timer is enabled
      if(this->EN && !this->DN) {
        this->_now = this->_timingFunc();
        // set start = now on rising edge
        if(!this->TT){this->_start = this->_now;}
        // compute the time accumulated accounting for roll over
        this->ACC = this->_now >= this->_start ? this->_now - this->_start : this->_now + (this->MAX_VALUE - this->_start);
        this->DN = (this->ACC >= this->PRE);
      // reset the timer on falling edge of enabled
      } else if(!this->EN && (this->ACC || this->DN)) {
        this->reset();
      }
      // update the TT state
      this->TT = (this->EN && !this->DN);
      return *this;
    }
};


/*
* Class that replicates the functionality of an RSLogix TOF instruction
* // tof test
* if(Serial.available() || tof.DN)
* {
*   tof.update(!tof.DN);
*   Serial.read();
*   Serial.print("EN= "); Serial.print(tof.EN);
*   Serial.print(" ACC="); Serial.print(tof.ACC);
*   Serial.print(" DN="); Serial.println(tof.DN);
* }
*/
class TOF : public PLCTimer{
  public:
    TOF(unsigned long (*timingFunc)(void))
      : PLCTimer{timingFunc}
    {
      // pass
    }

     TOF(unsigned long (*timingFunc)(void), uint32_t pre)
      : PLCTimer{timingFunc}
    {
      PRE = pre;
    }

    /*
    * function to be called to update the timer in the event loop.
    * returns the ton instance.
    */
    TOF& call(bool enabled)
    {
      this->EN = enabled;
      this->DN = this->DN || this->EN;
      // check if the timer is enabled
      if(!this->EN && this->DN) {
        this->_now = this->_timingFunc();
        // set start = now on rising edge
        if(!this->TT){this->_start = this->_now;}
        // compute the time accumulated accounting for roll over
        this->ACC = this->_now >= this->_start ? this->_now - this->_start : this->_now + (this->MAX_VALUE - this->_start);
        this->DN = (this->ACC <= this->PRE);
      // reset the timer on falling edge of enabled
      } else if(this->EN && (this->ACC || this->DN)) {
        this->reset();
      }
      // update the TT state
      this->TT = (!this->EN && this->DN);
      return *this;
    }
};

#endif