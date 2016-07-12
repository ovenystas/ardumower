/*
 Ardumower (www.ardumower.de)
 Copyright (c) 2013-2014 by Alexander Grau
 Copyright (c) 2013-2014 by Sven Gennat

 Private-use only! (you need to ask for a commercial-use)

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 Private-use only! (you need to ask for a commercial-use)
 */

#ifndef PID_H
#define PID_H

/*
 digital PID controller
 */

class Pid
{
  public:
    Pid(void) {};
    Pid(float Kp, float Ki, float Kd) :
        Kp(Kp), Ki(Ki), Kd(Kd) {};
    void setup(const float Kp, const float Ki, const float Kd);
    void setup(const float Kp, const float Ki, const float Kd,
               const float y_min, const float y_max, const float max_output);
    float compute(float processValue);

    const float getErrorOld() const
    {
      return errorOld;
    }

    const float getSetpoint() const
    {
      return setPoint;
    }

    void setSetpoint(float setPoint)
    {
      this->setPoint = setPoint;
    }

    float getMaxOutput() const
    {
      return max_output;
    }

    void setMaxOutput(float maxOutput)
    {
      max_output = maxOutput;
    }

    float getYMax() const
    {
      return y_max;
    }

    void setYMax(float y_max)
    {
      this->y_max = y_max;
    }

    float getYMin() const
    {
      return y_min;
    }

    void setYMin(float y_min)
    {
      this->y_min = y_min;
    }

    float Kp {};         // proportional control
    float Ki {};         // integral control
    float Kd {};         // differential control

  private:
    float setPoint {};   // set value
    float y_min {};      // minimum control output
    float y_max {};      // maximum control output
    float max_output {}; // maximum output

    float errorOld {};   // last error
    float errorSum {};   // error sum
    unsigned long lastControlTime {};
};

#endif
