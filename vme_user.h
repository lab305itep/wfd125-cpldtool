#ifndef _VME_USER_H
#define _VME_USER_H

#include <sys/ioctl.h>

/* From include/linux/vme.h */

#define VME_A16         0x1
#define VME_A24         0x2
#define VME_A32         0x4
#define VME_A64         0x8
#define VME_CRCSR       0x10

#define VME_SUPER       0x1000
#define VME_USER        0x2000
#define VME_PROG        0x4000
#define VME_DATA        0x8000

#define VME_SCT         0x1
#define VME_BLT         0x2
#define VME_MBLT        0x4
#define VME_2eVME       0x8
#define VME_2eSST       0x10
#define VME_2eSSTB      0x20

#define VME_D8          0x1
#define VME_D16         0x2
#define VME_D32         0x4
#define VME_D64         0x8

/* From drivers/staging/vme/devices/vme_user.h */
/* If ioctl's don't work, try checking headers from your kernel. */

typedef unsigned int u32;

#define __packed                        __attribute__((packed))

struct vme_master {
        int enable;                     /* State of Window */
        unsigned long long vme_addr;    /* Starting Address on the VMEbus */
        unsigned long long size;        /* Window Size */
        u32 aspace;                     /* Address Space */
        u32 cycle;              /* Cycle properties */
        u32 dwidth;             /* Maximum Data Width */
} __packed;

#define VME_IOC_MAGIC 0xAE

#define VME_GET_MASTER _IOR(VME_IOC_MAGIC, 3, struct vme_master)
#define VME_SET_MASTER _IOW(VME_IOC_MAGIC, 4, struct vme_master)

#endif
