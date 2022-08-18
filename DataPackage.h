/*
 * DataPackage.h
 *
 *  Created on: Aug 4, 2022
 *      Author: Justin
 */

#ifndef __DATAPACKAGE_H_
#define __DATAPACKAGE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

typedef struct
{
    void* addr; // address of the data
    uint32_t numBytes; // number of bytes of data

} DataPackage_t;

#ifdef __cplusplus
}
#endif

#endif /* DATAPACKAGE_H_ */
