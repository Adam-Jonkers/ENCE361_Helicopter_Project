#ifndef SIGNEDCIRCBUFT_H_
#define SIGNEDCIRCBUFT_H_

// *******************************************************
//
// signedCircBufT.h
//  based on circBuft.c
// Support for a circular buffer of int32_t values on the
//  Tiva processor.
//
// *******************************************************
#include <stdint.h>

// *******************************************************

typedef struct {
    uint32_t size;      // Number of entries in buffer
    uint32_t windex;    // Index for writing, mod(size)
    uint32_t rindex;    // Index for reading, mod(size)
    int32_t *data;      // Pointer to the data
} circBufSigned_t;

// *******************************************************
// initCircBuf: Initialise the circBuf instance. Reset both indices to
// the start of the buffer.  Dynamically allocate and clear the the
// memory and return a pointer for the data.  Return NULL if
// allocation fails.
int32_t *
sinitCircBuf (circBufSigned_t *buffer, uint32_t size);

// *******************************************************
// writeCircBuf: insert entry at the current windex location,
// advance windex, modulo (buffer size).
void
swriteCircBuf (circBufSigned_t *buffer, int32_t entry);

// *******************************************************
// readCircBuf: return entry at the current rindex location,
// advance rindex, modulo (buffer size). The function deos not check
// if reading has advanced ahead of writing.
int32_t
sreadCircBuf (circBufSigned_t *buffer);

// *******************************************************
// freeCircBuf: Releases the memory allocated to the buffer data,
// sets pointer to NULL and other fields to 0. The buffer can
// re initialised by another call to sinitCircBuf().
void
sfreeCircBuf (circBufSigned_t *buffer);



#endif /* SIGNEDCIRCBUFT_H_ */
