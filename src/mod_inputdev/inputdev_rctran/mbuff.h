/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *   Copyright (C) 2000, 2001 - Jan Edward Kansky (original author)
 *   Copyright (C) 2004 - Jan Reucker
 *   Copyright (C) 2008 - Jens Wilhelm Wulf
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */
#ifndef __MBUFF_H
#define __MBUFF_H
#ifdef __cplusplus
extern "C" {
#endif


#define RTL_SHM_MISC_MINOR 254

/* max length of the name of the shared memory area */
#define MBUFF_NAME_LEN 32
/* max number of attached mmaps per one area */
#define MBUFF_MAX_MMAPS 16


#ifdef  SHM_DEMO
#define MBUFF_DEV_NAME "./mbuff"
#else
#define MBUFF_DEV_NAME "/dev/mbuff"
#endif

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/fs.h>
#else
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#endif


#define MBUFF_VERSION "0.7.3"

/*
   All ioctl()s are called with name filled in with the appropriate
   name for the mbuff to be referenced.  Calls to any ioctl() makes
   that mbuff "active", i.e., read(), write(), and mmap() use that
   mbuff.  I didn't do this yet.

   ioctl()s:

   ALLOCATE:
      Call with size=0 to just find out if the area exists; no
      mbuff will be allocated.  Otherwise, allocate an mbuff with
      that size.
   DEALLOCATE:
      Decrease reference count for an mbuff.

   issues:
      - using this method, it is *really* easy to get dangling
        mbuffs, i.e., mbuffs that nobody owns.  When you close
	/dev/mbuff, it would be a good idea to decrease the ref
	count of the active mbuff.
 */
#define IOCTL_MBUFF_INFO 0
#define IOCTL_MBUFF_ALLOCATE 1
#define IOCTL_MBUFF_DEALLOCATE 2
#define IOCTL_MBUFF_SELECT 3
#define IOCTL_MBUFF_LAST IOCTL_MBUFF_SELECT

struct mbuff_request_struct{
	unsigned int flags;

	char name[MBUFF_NAME_LEN+1];

	size_t size;

	unsigned int reserved[4];
};

#ifndef __KERNEL__

/* you can use mbuff_alloc several times, the buffer
will be deallocated when mbuff_free was called the same number of times
AND area is not mmaped anywhere anymore
AND it is not used in the kernel as well */
/* if you have a need to mmap the area at the specific address, use
 * mbuff_alloc_at */
static inline int mbuff_open_dev(const char *name, struct mbuff_request_struct  *req) {
	int fd;

	if(name) strncpy(req->name, name, sizeof(req->name));
	req->name[sizeof(req->name)-1]='\0';
	if(( fd = open(MBUFF_DEV_NAME, O_RDWR) ) < 0 ){
		perror("open " MBUFF_DEV_NAME " failed");
		return fd;
	}
	return fd;
}

static inline void * mbuff_alloc_at(const char *name, int size, void * addr) {
	int fd;
	struct mbuff_request_struct req={0,"default",0,{0}};
	void * mbuf;

	req.size = size;
	if((fd=mbuff_open_dev(name,&req))<0)
		return NULL;
	size=ioctl(fd,IOCTL_MBUFF_ALLOCATE,&req);
	if((size_t)size<req.size)
		return NULL;
/* the type of first mmap's argument depends on libc version? This really
 * drives me crazy. Man mmap says "void * start" */
	mbuf=mmap(addr, size,PROT_WRITE|PROT_READ,MAP_SHARED|MAP_FILE,fd, 0);
	if( mbuf == (void *) -1) 
		mbuf=NULL;
	close(fd);
	return mbuf;
}

static inline void * mbuff_alloc(const char *name, int size) {
	return mbuff_alloc_at(name, size, NULL);
}

static inline int mbuff_size(const char *name) {
	int fd;
	struct mbuff_request_struct req={0,"default",0,{0}};

	req.size = 0;
	if((fd=mbuff_open_dev(name,&req))<0)
		return -ENODEV;
	return ioctl(fd,IOCTL_MBUFF_INFO,&req);
}

static inline void mbuff_free(const char *name, void * mbuf) {
	int fd;
	struct mbuff_request_struct req={0,"default",0,{0}};
	int size;

	if((fd=mbuff_open_dev(name,&req))<0)
		return;
	size=ioctl(fd,IOCTL_MBUFF_DEALLOCATE,&req);
	if(size > 0) munmap( mbuf, size);
	close(fd);
	/* in general, it could return size, but typical "free" is void */
	return;
}

/* mbuff_attach and mbuff_detach do not change usage counters -
   area allocated using mbuff_attach will be deallocated on program exit/kill
   if nobody else uses it - mbuff_detach is not needed -
   the only lock keeping area allocated is mmap */

static inline void * mbuff_attach_at(const char *name, int size, void * addr) {
	int fd;
	struct mbuff_request_struct req={0, "default", 0, {0}};
	void * mbuf;

	if((fd=mbuff_open_dev(name,&req))<0)
		return NULL;
	req.size = size;
	ioctl(fd,IOCTL_MBUFF_ALLOCATE,&req);
	mbuf=mmap(addr, size, PROT_WRITE|PROT_READ, MAP_SHARED|MAP_FILE, fd, 0);
	/* area will be deallocated on the last munmap, not now */
	ioctl(fd, IOCTL_MBUFF_DEALLOCATE, &req);
	if( mbuf == (void *) -1) 
		mbuf=NULL;
	close(fd);
	return mbuf;
}

static inline void * mbuff_mmap_at(const char *name, int size, void * addr) {
	int fd;
	struct mbuff_request_struct req={0, "default", 0, {0}};
	void * mbuf;

	if((fd=mbuff_open_dev(name,&req))<0)
		return NULL;
	req.size = size;
	ioctl(fd,IOCTL_MBUFF_SELECT,&req);
	mbuf=mmap(addr, size, PROT_WRITE|PROT_READ, MAP_SHARED|MAP_FILE, fd, 0);	if( mbuf == (void *) -1)
		mbuf=NULL;
	close(fd);
	return mbuf;
}

static inline void * mbuff_attach(const char *name, int size) {
	return mbuff_attach_at(name, size, NULL);
}

static inline void * mbuff_mmap(const char *name, int size) {
	return mbuff_mmap_at(name, size, NULL);
}

static inline void mbuff_detach(const char *name, void * mbuf) {
	int fd;
	struct mbuff_request_struct req={0,"default",0,{0}};
	int size;

	if((fd=mbuff_open_dev(name,&req))<0)
		return;
	size=ioctl(fd,IOCTL_MBUFF_SELECT,&req);
	if(size > 0) munmap( mbuf, size);
	close(fd);
	/* in general, it could return size, but typical "free" is void */
	return;
}

#define mbuff_unmap mbuff_detach

#else

	
struct mbuff{
	struct mbuff *next;
	struct mbuff *prev;

	char name[MBUFF_NAME_LEN+1];

	struct vm_area_struct *(vm_area[MBUFF_MAX_MMAPS]);
	struct file *file;

	unsigned char *buf;
	unsigned long size;
	int count;  /* number of allocations from user space */
	int kcount; /* number of allocations from kernel space */
	int open_cnt;	/* #times opened */
	int open_mode;
};

extern struct mbuff * mbuff_list_lookup_name(const char *name,int priority);
extern struct mbuff * mbuff_list_lookup_buf(void *buf);
extern int shm_allocate(const char *name,unsigned int size, void **shm);
extern int shm_deallocate(void * shm);

static inline void * mbuff_alloc(const char *name, int size) {
	void *tmp=NULL;
	if( shm_allocate(name, size, &tmp) > 0 )
		return tmp;
	else
		return NULL;
}
static inline void mbuff_free(const char *name, void * mbuf) {
/* it would be no problem to deallocate using only name */
	shm_deallocate(mbuf);
}
/* in kernel space implementing "nonlocking" attach and detach
   would be very unsafe (deallocation from user space possible at any time) */
#define mbuff_attach(name,size) mbuff_alloc(name,size)
#define mbuff_detach(name,mbuf) mbuff_free(name,mbuf)

/** get kernel address of mbuff. It _will_ actually increment kernel usage 
 * counter. It will _not_ create new mbuff if not existing yet. */
static inline void * mbuff_mmap(const char *name, int size) {
	if(mbuff_list_lookup_name(name,0) == NULL)
		return NULL;
	return mbuff_attach(name,size);
}
#define mbuff_unmap mbuff_detach

static inline int mbuff_size(const char *name) {
	struct mbuff *mbuf;
	if( (mbuf=mbuff_list_lookup_name(name,0)) == NULL)
		return -EINVAL;
	return mbuf->size;
}

extern char mbuff_default_name[];
extern int mbuff_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	unsigned long arg);
#ifdef LINUX_V22
extern int mbuff_mmap_handler(struct file *file, struct vm_area_struct *vma);
#else
extern int mbuff_mmap_handler(struct inode *inode, struct file *file, 
	struct vm_area_struct *vma);
#endif
extern int mbuff_open_with_name( struct inode *inode, struct file *file,
	const char * name);


#endif
#ifdef __cplusplus
}
#endif
#endif
