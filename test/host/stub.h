/* stub.h: shared declarations for the host-side REL test harness */
#ifndef STUB_H
#define STUB_H

#include <stdint.h>
#include "dirent.h"
#include "buffers.h"
#include "errormsg.h"

/* image access (write-through via the fatops image layer) */
void image_load(const char *filename);
void image_flush(void);

/* FatFs shim: base directory of the emulated FAT medium, an empty
   string passes host paths through unchanged */
void fatshim_set_base(const char *dir);
void fatshim_clear_base(void);

#endif
