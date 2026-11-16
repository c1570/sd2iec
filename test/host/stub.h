/* stub.h: shared declarations for the host-side REL test harness */
#ifndef STUB_H
#define STUB_H

#include <stdint.h>
#include "dirent.h"
#include "buffers.h"
#include "errormsg.h"

/* in-memory image access */
void     image_load(const char *filename);
void     image_save(const char *filename);

#endif
