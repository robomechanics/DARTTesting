/**
 * @authors Garrett Wenger

  This file is part of koduino <https://github.com/avikde/koduino>

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation, either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
#ifndef RTC_H
#define RTC_H

#include "Arduino.h"
#include <time.h>

/**
 * @brief Class for getting the real clock time
 */
class RTCClass {
private:
	// boolean rtcCalibrated;

public:
	RTCClass(void);

	void setTime(time_t localTime);
	boolean isCalibrated();
	void getTimestruct(struct tm * timeStruct);

};

extern RTCClass Clock;

#endif