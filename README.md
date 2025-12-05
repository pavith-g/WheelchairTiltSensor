Wheelchair Tilt Sensor and Buzzer

Project requirement:

Patient requires a beep tone when the wheelchair begins to tilt back from 0 degrees. The beep must continue until a specified angle. Once reached, the beep stops. 
E.G. If required angle threshold is 15 degrees, beep must occur when 0 < angle <= 15. The threshold angle should be adjustable by the user. 


USAGE:

Once the board starts up (or when RESET button is pressed), the board will set the current tilt as its 0 value. IE, the wheelchair should be level with the ground when starting up. It will then measure an offset to this angle and compare the offset to the threshold value. The potentiometer can be used to adjust the threshold value, ranging from 5 - 20 degrees. 
