#ifndef TOPIC_CUBIE_POS_H
#define TOPIC_CUBIE_POS_H

/* define the data structure that will be published where subscribers can see it */
struct cubie_pos_s {
	float x, y, z;
};

/* declare the topic */
ORB_DECLARE(cubie_position);

#endif
