/* fatshim.c: FatFs API implemented over POSIX I/O for the host tests.
   The firmware's ff.c is replaced by this shim so that fatops.c runs
   against plain host files inside a base directory, see
   fatshim_set_base(). An empty base directory passes host paths through
   unchanged, the harness uses this to open disk images.

   Limitations compared to real FAT:
   - one flat directory, no subdirectories or volumes
   - cluster numbers are shim-internal ids that map back to host paths
   - file attributes and timestamps are not tracked

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; version 2 of the License only. */

#define _DEFAULT_SOURCE
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

/* POSIX and FatFs both name their directory handle DIR */
#define DIR host_DIR
#include <dirent.h>
#undef DIR

#include "ff.h"

#define SHIM_MAXOPEN 8
#define SHIM_MAXDIR  4

static char base_dir[256] = "fatroot";

struct filslot {
  FIL   *fil;
  FILE  *fp;
  uint8_t used;
};
static struct filslot filtab[SHIM_MAXOPEN];

struct dirslot {
  DIR   *dir;
  struct dirent **entries;
  int    count;
  int    pos;
  char   path[512];
  uint8_t used;
};
static struct dirslot dirtab[SHIM_MAXDIR];

/* cluster table: finfo.clust values handed out by f_readdir/f_stat map
   back to host paths for l_opencluster (used by the x00 detection in
   fat_readdir) */
#define CLUSTERTAB_SIZE 128
struct clustertab_entry {
  DWORD id;
  char  path[300];
};
static struct clustertab_entry clustertab[CLUSTERTAB_SIZE];
static DWORD next_cluster_id = 100;

static const char *cluster_path(DWORD clust) {
  uint16_t i;
  for (i = 0; i < CLUSTERTAB_SIZE; i++)
    if (clustertab[i].id == clust && clustertab[i].path[0])
      return clustertab[i].path;
  return NULL;
}

static DWORD cluster_register(const char *hostpath) {
  static uint16_t rot;
  uint16_t i;
  DWORD id;

  /* reuse the id of a path that is already registered */
  for (i = 0; i < CLUSTERTAB_SIZE; i++)
    if (clustertab[i].id && !strcmp(clustertab[i].path, hostpath))
      return clustertab[i].id;

  id = next_cluster_id++;
  clustertab[rot].id = id;
  strncpy(clustertab[rot].path, hostpath, sizeof(clustertab[rot].path) - 1);
  clustertab[rot].path[sizeof(clustertab[rot].path) - 1] = 0;
  rot = (rot + 1) % CLUSTERTAB_SIZE;
  return id;
}

static void ensure_base(void) {
  mkdir(base_dir, 0755);
}

void fatshim_set_base(const char *dir) {
  strncpy(base_dir, dir, sizeof(base_dir) - 1);
  base_dir[sizeof(base_dir) - 1] = 0;
}

void fatshim_clear_base(void) {
  ensure_base();
  {
    struct dirent **entries;
    int count = scandir(base_dir, &entries, NULL, NULL);
    int i;
    if (count < 0)
      return;
    for (i = 0; i < count; i++) {
      char path[512];
      if (!strcmp(entries[i]->d_name, ".") ||
          !strcmp(entries[i]->d_name, "..")) {
        free(entries[i]);
        continue;
      }
      snprintf(path, sizeof(path), "%s/%s", base_dir, entries[i]->d_name);
      unlink(path);
      free(entries[i]);
    }
    free(entries);
  }
}

static FILE *lookup_fp(FIL *fil) {
  uint8_t i;
  for (i = 0; i < SHIM_MAXOPEN; i++)
    if (filtab[i].used && filtab[i].fil == fil)
      return filtab[i].fp;
  return NULL;
}

static int alloc_filslot(FIL *fil, FILE *fp) {
  uint8_t i;
  /* re-registering the same FIL replaces its entry (ff.c would just
     overwrite the FIL struct) */
  for (i = 0; i < SHIM_MAXOPEN; i++) {
    if (filtab[i].used && filtab[i].fil == fil) {
      fclose(filtab[i].fp);
      filtab[i].fp = fp;
      return 0;
    }
  }
  for (i = 0; i < SHIM_MAXOPEN; i++) {
    if (!filtab[i].used) {
      filtab[i].fil = fil;
      filtab[i].fp  = fp;
      filtab[i].used = 1;
      return 0;
    }
  }
  return -1;
}

static void free_dirslot_entries(struct dirslot *slot) {
  int i;
  for (i = 0; i < slot->count; i++)
    free(slot->entries[i]);
  free(slot->entries);
  slot->entries = NULL;
  slot->count = 0;
  slot->pos = 0;
}

static struct dirslot *alloc_dirslot(DIR *dir) {
  uint8_t i;
  /* re-registering the same DIR replaces its entry (fatops.c reuses
     the same dh_t for every scan) */
  for (i = 0; i < SHIM_MAXDIR; i++) {
    if (dirtab[i].used && dirtab[i].dir == dir) {
      free_dirslot_entries(&dirtab[i]);
      dirtab[i].count = 0;
      dirtab[i].pos   = 0;
      return &dirtab[i];
    }
  }
  for (i = 0; i < SHIM_MAXDIR; i++) {
    if (!dirtab[i].used) {
      dirtab[i].dir    = dir;
      dirtab[i].used   = 1;
      dirtab[i].count  = 0;
      dirtab[i].pos    = 0;
      dirtab[i].entries = NULL;
      return &dirtab[i];
    }
  }
  return NULL;
}

static struct dirslot *lookup_dirslot(DIR *dir) {
  uint8_t i;
  for (i = 0; i < SHIM_MAXDIR; i++)
    if (dirtab[i].used && dirtab[i].dir == dir)
      return &dirtab[i];
  return NULL;
}

/* strip an optional drive prefix and leading slash from a FatFs path */
static const char *fat_name(const UCHAR *name) {
  if (name[0] && name[1] == ':')
    name += 2;
  if (*name == '/')
    name++;
  return (const char *)name;
}

/* build a host path from a FatFs path */
static const char *shim_path(const UCHAR *name) {
  static char buf[2][512];
  static uint8_t rot;
  char *p = buf[rot & 1];
  rot++;

  if (!base_dir[0])
    snprintf(p, sizeof(buf[0]), "%s", (const char *)name);
  else
    snprintf(p, sizeof(buf[0]), "%s/%s", base_dir, fat_name(name));
  return p;
}


static FRESULT errno_to_fr(void) {
  switch (errno) {
  case EEXIST:    return FR_EXIST;
  case ENOENT:    return FR_NO_FILE;
  case ENOTDIR:   return FR_NOT_DIRECTORY;
  case EISDIR:    return FR_IS_DIRECTORY;
  case EACCES:    return FR_WRITE_PROTECTED;
  case ENOTEMPTY: return FR_DIR_NOT_EMPTY;
  default:        return FR_RW_ERROR;
  }
}

/* ------------------------------------------------------------------ */

FRESULT f_mount(BYTE drive, FATFS *fs) {
  (void)drive;
  if (fs == NULL)
    return FR_OK;
  memset(fs, 0, sizeof(FATFS));
  fs->fs_type = 3;   /* pretend FAT32 */
  fs->curr_dir = 2;  /* magic "cluster" of the base directory */
  ensure_base();
  return FR_OK;
}

/* FAT is case-insensitive; find the actual host file name matching
   name case-insensitively, or NULL */
static const char *shim_caseinsensitive(const UCHAR *name) {
  static char found[300];
  struct dirent **entries;
  int count, i;

  if (!base_dir[0])
    return NULL;
  count = scandir(base_dir, &entries, NULL, NULL);
  if (count < 0)
    return NULL;

  for (i = 0; i < count; i++) {
    if (!strcasecmp(entries[i]->d_name, fat_name(name))) {
      strncpy(found, entries[i]->d_name, sizeof(found) - 1);
      found[sizeof(found) - 1] = 0;
      free(entries[i]);
      while (++i < count)
        free(entries[i]);
      free(entries);
      return found;
    }
    free(entries[i]);
  }
  free(entries);
  return NULL;
}

/* host path of an existing file matching name case-insensitively, or
   the path for name itself */
static const char *resolve_path(const UCHAR *name) {
  const char *actual = shim_caseinsensitive(name);
  return shim_path(actual ? (const UCHAR *)actual : name);
}

FRESULT f_open(FATFS *fs, FIL *fil, const UCHAR *name, BYTE flags) {
  const char *path = resolve_path(name);
  int oflags = 0, fd;
  FILE *fp;
  struct stat st;

  if (fs == NULL)
    return FR_NO_FILESYSTEM;

  switch (flags & (FA_READ | FA_WRITE)) {
  case FA_READ:            oflags = O_RDONLY; break;
  case FA_WRITE:           oflags = O_WRONLY; break;
  default:                 oflags = O_RDWR;   break;
  }

  switch (flags & (FA_CREATE_NEW | FA_CREATE_ALWAYS | FA_OPEN_ALWAYS)) {
  case FA_CREATE_NEW:      oflags |= O_CREAT | O_EXCL; break;
  case FA_CREATE_ALWAYS:   oflags |= O_CREAT | O_TRUNC; break;
  case FA_OPEN_ALWAYS:     oflags |= O_CREAT; break;
  default:                 break; /* FA_OPEN_EXISTING */
  }

  fd = open(path, oflags, 0600);
  if (fd < 0)
    return errno_to_fr();

  switch (oflags & O_ACCMODE) {
  case O_RDONLY: fp = fdopen(fd, "rb");  break;
  case O_WRONLY: fp = fdopen(fd, "wb");  break;
  default:       fp = fdopen(fd, "r+b"); break;
  }
  if (fp == NULL) {
    close(fd);
    return FR_RW_ERROR;
  }

  memset(fil, 0, sizeof(FIL));
  fil->fs = fs;
  fil->flag = flags;
  if (fstat(fd, &st) == 0)
    fil->fsize = st.st_size;

  if (alloc_filslot(fil, fp) < 0) {
    fclose(fp);
    return FR_INVALID_OBJECT;
  }
  return FR_OK;
}

FRESULT f_close(FIL *fil) {
  FILE *fp = lookup_fp(fil);
  uint8_t i;

  if (fp == NULL)
    return FR_INVALID_OBJECT;
  for (i = 0; i < SHIM_MAXOPEN; i++)
    if (filtab[i].used && filtab[i].fil == fil) {
      filtab[i].used = 0;
      break;
    }
  if (fclose(fp) != 0)
    return FR_RW_ERROR;
  return FR_OK;
}

FRESULT f_sync(FIL *fil) {
  FILE *fp = lookup_fp(fil);
  if (fp == NULL)
    return FR_INVALID_OBJECT;
  if (fflush(fp) != 0)
    return FR_RW_ERROR;
  return FR_OK;
}

/* ISO C requires a positioning call when an update stream switches
   between reading and writing */
static int sync_stream(FILE *fp) {
  return fseek(fp, 0, SEEK_CUR);
}

FRESULT f_read(FIL *fil, void *buffer, UINT bytes, UINT *read) {
  FILE *fp = lookup_fp(fil);
  if (fp == NULL)
    return FR_INVALID_OBJECT;
  if (sync_stream(fp) != 0)
    return FR_RW_ERROR;
  *read = fread(buffer, 1, bytes, fp);
  fil->fptr = ftell(fp);
  if (*read != bytes && ferror(fp))
    return FR_RW_ERROR;
  return FR_OK;
}

FRESULT f_write(FIL *fil, const void *buffer, UINT bytes, UINT *written) {
  FILE *fp = lookup_fp(fil);
  UINT n;
  if (fp == NULL)
    return FR_INVALID_OBJECT;
  if (sync_stream(fp) != 0)
    return FR_RW_ERROR;
  n = fwrite(buffer, 1, bytes, fp);
  *written = n;
  if (n != bytes)
    return FR_RW_ERROR;
  fil->fptr = ftell(fp);
  if (fil->fptr > fil->fsize)
    fil->fsize = fil->fptr;
  return FR_OK;
}

FRESULT f_lseek(FIL *fil, DWORD ofs) {
  FILE *fp = lookup_fp(fil);
  if (fp == NULL)
    return FR_INVALID_OBJECT;
  if (fseek(fp, ofs, SEEK_SET) != 0)
    return FR_RW_ERROR;
  fil->fptr = ftell(fp);
  return FR_OK;
}

/* Copy at most max characters of a name part to a short name, in the
   upper case FatFs reports */
static size_t copy_short(UCHAR *dst, const char *src, size_t max) {
  size_t i;

  for (i = 0; i < max && src[i]; i++)
    dst[i] = toupper((unsigned char)src[i]);
  return i;
}

/* Fill finfo for the host file at path whose name is hostname, the way
   ff.c reports a directory entry: a short name in fname and, when the
   caller provided a buffer and wants it, the long name in lfn */
static void fill_finfo(FILINFO *finfo, const char *path, const char *hostname,
                       uint8_t with_lfn) {
  const char *dot = strrchr(hostname, '.');
  size_t baselen = dot ? (size_t)(dot - hostname) : strlen(hostname);
  const char *ext = dot ? dot + 1 : "";
  size_t extlen = strlen(ext);
  struct stat st;
  size_t o = 0;

  /* only the fields ff.c's get_fileinfo fills; finfo->lfn is left
     alone unless the caller asked for a long name */
  finfo->fname[0] = 0;
  finfo->fsize    = 0;
  finfo->fattrib  = 0;
  if (stat(path, &st) == 0) {
    finfo->fsize   = st.st_size;
    finfo->fattrib = S_ISDIR(st.st_mode) ? AM_DIR : AM_ARC;
  }
  finfo->clust = cluster_register(path);

  /* a plain 8.3 name is reported as is, anything else mangled like
     VFAT does; fatops.c's check_extension needs the extension to
     detect x00 files */
  if (baselen <= 8 && extlen <= 3) {
    o += copy_short(finfo->fname + o, hostname, baselen);
  } else {
    o += copy_short(finfo->fname + o, hostname, 6);
    finfo->fname[o++] = '~';
    finfo->fname[o++] = '1';
  }
  if (extlen) {
    finfo->fname[o++] = '.';
    o += copy_short(finfo->fname + o, ext, 3);
  }
  finfo->fname[o] = 0;

  if (with_lfn && finfo->lfn) {
    if (strlen(hostname) <= _MAX_LFN_LENGTH)
      strcpy((char *)finfo->lfn, hostname);
    else
      finfo->lfn[0] = 0;
  }
}

FRESULT f_stat(FATFS *fs, const UCHAR *name, FILINFO *finfo) {
  const char *path = resolve_path(name);
  const char *hostname = strrchr(path, '/');
  struct stat st;
  (void)fs;

  if (stat(path, &st) != 0)
    return errno_to_fr();
  /* f_stat only fills the short name, like ff.c's get_fileinfo; callers
     may pass a FILINFO with an uninitialized lfn */
  fill_finfo(finfo, path, hostname ? hostname + 1 : path, 0);
  return FR_OK;
}

FRESULT f_unlink(FATFS *fs, const UCHAR *name) {
  (void)fs;
  if (unlink(resolve_path(name)) != 0)
    return errno_to_fr();
  return FR_OK;
}

FRESULT f_rename(FATFS *fs, const UCHAR *oldname, const UCHAR *newname) {
  (void)fs;
  if (rename(shim_path(oldname), shim_path(newname)) != 0)
    return errno_to_fr();
  return FR_OK;
}

FRESULT f_mkdir(FATFS *fs, const UCHAR *name) {
  (void)fs;
  if (mkdir(shim_path(name), 0755) != 0)
    return errno_to_fr();
  return FR_OK;
}

FRESULT f_chmod(FATFS *fs, const UCHAR *name, BYTE value, BYTE mask) {
  (void)fs;
  (void)name;
  (void)value;
  (void)mask;
  return FR_OK;
}

FRESULT f_chdir(FATFS *fs, const UCHAR *name) {
  (void)fs;
  (void)name;
  return FR_OK;
}

FRESULT f_getfree(FATFS *fs, const UCHAR *name, DWORD *clusters) {
  (void)fs;
  (void)name;
  *clusters = 0x00ffffff;
  return FR_OK;
}

FRESULT l_getfree(FATFS *fs, const UCHAR *name, DWORD *clusters, DWORD limit) {
  (void)fs;
  (void)name;
  *clusters = 0x00ffffff;
  if (*clusters > limit)
    *clusters = limit;
  return FR_OK;
}

FRESULT f_mkfs(BYTE drive, BYTE partition, WORD mask) {
  (void)drive;
  (void)partition;
  (void)mask;
  ensure_base();
  return FR_OK;
}

static FRESULT open_dir_slot(FATFS *fs, DIR *dir, const char *path) {
  struct dirslot *slot = alloc_dirslot(dir);

  if (slot == NULL)
    return FR_INVALID_OBJECT;
  slot->count = scandir(path, &slot->entries, NULL, NULL);
  if (slot->count < 0) {
    slot->count = 0;
    slot->entries = NULL;
    slot->used = 0;
    return errno_to_fr();
  }
  snprintf(slot->path, sizeof(slot->path), "%s", path);
  slot->pos = 0;
  dir->fs = fs;
  dir->sect = 0;
  return FR_OK;
}

FRESULT f_opendir(FATFS *fs, DIR *dir, const UCHAR *name) {
  char path[512];
  const char *sub = fat_name(name);

  if (!base_dir[0])
    snprintf(path, sizeof(path), "%s", sub[0] ? sub : ".");
  else
    snprintf(path, sizeof(path), "%s%s%s", base_dir, *sub ? "/" : "", sub);

  return open_dir_slot(fs, dir, path);
}

FRESULT f_readdir(DIR *dir, FILINFO *finfo) {
  struct dirslot *slot = lookup_dirslot(dir);
  struct dirent *de;
  char fullpath[1024];

  if (slot == NULL)
    return FR_INVALID_OBJECT;

  do {
    if (slot->pos >= slot->count) {
      /* end of directory: ff.c returns an empty name */
      UCHAR *lfn = finfo->lfn;

      free_dirslot_entries(slot);
      memset(finfo, 0, sizeof(FILINFO));
      finfo->lfn = lfn;
      if (lfn)
        lfn[0] = 0;
      return FR_OK;
    }
    de = slot->entries[slot->pos++];
  } while (!strcmp(de->d_name, ".") || !strcmp(de->d_name, ".."));

  snprintf(fullpath, sizeof(fullpath), "%s/%s", slot->path, de->d_name);
  fill_finfo(finfo, fullpath, de->d_name, 1);
  return FR_OK;
}

/* internal ff.c helpers used by fatops.c */

FRESULT l_opendir(FATFS *fs, DWORD cluster, DIR *dir) {
  (void)cluster;
  return open_dir_slot(fs, dir, base_dir);
}

FRESULT l_opencluster(FATFS *fs, FIL *fil, DWORD clust) {
  const char *path = cluster_path(clust);
  struct stat st;
  FILE *fp;

  if (path == NULL)
    return FR_NO_FILE;
  fp = fopen(path, "rb");
  if (fp == NULL)
    return errno_to_fr();
  if (fstat(fileno(fp), &st) != 0) {
    fclose(fp);
    return FR_RW_ERROR;
  }

  memset(fil, 0, sizeof(FIL));
  fil->fs = fs;
  fil->flag = FA_READ | FA_OPEN_EXISTING;
  fil->fsize = st.st_size;

  if (alloc_filslot(fil, fp) < 0) {
    fclose(fp);
    return FR_INVALID_OBJECT;
  }
  return FR_OK;
}