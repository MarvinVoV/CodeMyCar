/*
 * sys_utils.h
 *
 *  Created on: Apr 4, 2025
 *      Author: marvin
 */

#ifndef SYSTEM_UTILS_INC_SYS_UTILS_H_
#define SYSTEM_UTILS_INC_SYS_UTILS_H_

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef CLAMP
#define CLAMP(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))
#endif

#ifndef SIGN
#define SIGN(x) ((x > 0) ? 1 : ((x < 0) ? -1 : 0))
#endif


#endif /* SYSTEM_UTILS_INC_SYS_UTILS_H_ */
