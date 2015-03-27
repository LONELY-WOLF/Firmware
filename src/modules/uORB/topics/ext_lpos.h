#ifndef TOPIC_EXT_LPOS_H
#define TOPIC_EXT_LPOS_H

/* define the data structure that will be published where subscribers can see it */
struct ext_lpos_s {
	float x, y, z;
};

/* declare the topic */
ORB_DECLARE(ext_lpos_position);

#endif
