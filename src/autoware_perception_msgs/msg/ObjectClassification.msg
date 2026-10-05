# Semantic class of an object, with the confidence the perception stack has in it.
# An object carries an ARRAY of these: a detector that is torn between car and
# truck says so, instead of collapsing to one label at the message boundary.

uint8 UNKNOWN = 0
uint8 CAR = 1
uint8 TRUCK = 2
uint8 BUS = 3
uint8 TRAILER = 4
uint8 MOTORCYCLE = 5
uint8 BICYCLE = 6
uint8 PEDESTRIAN = 7

uint8 label
float32 probability
