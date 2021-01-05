/******************************************************************************
 * privcmd.h
 *
 * Interface to /dev/xen/privcmd.
 *
 * Copyright (c) 2003-2005, K A Fraser
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation; or, when distributed
 * separately from the Linux kernel or incorporated into other
 * software packages, subject to the following license:
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this source file (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#ifndef __XEN_PRIVCMD_H__
#define __XEN_PRIVCMD_H__

struct ioctl_privcmd_hypercall
{
	unsigned long op; /* hypercall number */
	unsigned long arg[5]; /* arguments */
	long retval; /* return value */
};
typedef struct ioctl_privcmd_hypercall privcmd_hypercall_t;

struct ioctl_privcmd_mmapbatch {
	int num;     /* number of pages to populate */
	domid_t dom; /* target domain */
	unsigned long addr;  /* virtual address */
	const xen_pfn_t *arr; /* array of mfns */
	int *err; /* array of error codes */
};
typedef struct ioctl_privcmd_mmapbatch privcmd_mmapbatch_t;

typedef struct privcmd_mmap_entry {
	unsigned long va;
	unsigned long mfn;
	unsigned long npages;
} privcmd_mmap_entry_t;

struct ioctl_privcmd_mmapresource {
	domid_t dom; /* target domain */
	unsigned int type; /* type of resource to map */
	unsigned int id; /* type-specific resource identifier */
	unsigned int idx; /* the index of the initial frame to be mapped */
	unsigned long num; /* number of frames of the resource to be mapped */
	unsigned long addr; /* physical address to map into */
};
typedef struct ioctl_privcmd_mmapresource privcmd_mmap_resource_t;

#define IOCTL_PRIVCMD_HYPERCALL					\
	_IOWR('E', 0, struct ioctl_privcmd_hypercall)
#define IOCTL_PRIVCMD_MMAPBATCH					\
	_IOWR('E', 1, struct ioctl_privcmd_mmapbatch)
#define IOCTL_PRIVCMD_MMAP_RESOURCE				\
	_IOW('E', 2, struct ioctl_privcmd_mmapresource)
#define IOCTL_PRIVCMD_RESTRICT					\
	_IOW('E', 4, domid_t)

#endif /* !__XEN_PRIVCMD_H__ */
