/*
    SvirLex 2014 - wfd125 manipulation tool through its CPLD via vme_user kernel driver
*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>

#include "vme_user.h"

// module address space is max 8 regs at A000 + (SERIAL << 4)
#define BASE 0xA000
// register addresses
#define CSR  0
#define SDAT 2
#define SNUM 4
#define BNUM 6
// FLASH commands
#define FGETID 0x9E	// Get ID
#define FRDSTA 0x05	// Read status register
#define FCLRFL 0x50	// Clear status flag register
#define FRDMEM 0x03	// Read memory
#define FWRENB 0x06	// Write enable
#define FWRDSB 0x04	// Write disable
#define FBULKE 0xC7	// Bulk erase
#define FSECTE 0xD8	// Sector erase
#define FSSECE 0x20	// Subsector erase
#define FPROGP 0x02	// Page program
// FLASH sizes (bytes)
#define FSIZE 0x1000000	// Total size
#define SSIZE 0x10000	// Sector size (256 sectors)
#define SSSIZ 0x1000	// Subsector size (4096 subsectors)
#define PSIZE 0x100	// Page size (64k pages)

#ifndef SWAP_MODE
#define SWAP_MODE VME4L_NO_SWAP
#endif

typedef struct {
    unsigned addr;
    unsigned len;
    unsigned *ptr;
} VMEMAP;

VMEMAP map = {0, 0, NULL};
// module address
unsigned maddr;

// reads VME mapped memory, A16/D16 assumed, odd byte 
unsigned char vrd(unsigned adr)
{
    union SWP {
	unsigned short u;
	char c[2];
    } a;
    a.u = ((unsigned short *)map.ptr)[(adr + maddr)/2];
    return a.c[1];
}

// writes VME mapped memory, A16/D16 assumed, odd byte 
unsigned char vwr(unsigned adr, unsigned char D)
{
    union SWP {
	unsigned short u;
	char c[2];
    } a;
    a.c[1] = D;
    a.c[0] = 0;
    ((unsigned short *)map.ptr)[(adr + maddr)/2] = a.u;
    return a.c[1];
}

// no comments
void w125c_Usage(void)
{
    printf("Usage: cpldtool <serial#> <command> [<args>]\n");
    printf("\tCommands:\n");
    printf("\t *** FLASH is 16 MBytes, addreses up to 0xFFFFFF \n");
    printf("\t *** All flash commands leave the module in FLASH access mode\n");
    printf("\tErase [<bytes> [<begAddr>]]  -- erases FLASH: no args - entire flash,\n");
    printf("\t\tone arg - first <bytes>, two args - <bytes> starting at <begAddr>.\n");
    printf("\t\tErasure is made in 4kB sectors touched by <begAddr>-<begAddr>+<bytes> range.\n");
    printf("\tBlankcheck [<bytes> [<begAddr>]] -- check FLASH for blank values, same args.\n");
    printf("\tWrite <filename> -- writes binary file to (preerased) FLASH from addr 0.\n");
    printf("\tRead <filename> [<bytes>] -- reads <bytes> from FLASH addr 0 to binary file.\n");
    printf("\tVerify <filename> -- verifies FLASH against binary file.\n");
    printf("\tAutowrite <filename> -- does Erase (by file length), blank check, write, verify and PROG pulsing.");
    printf("\tProgram [<filename>] -- loads binary file directly to FPGA chain.\n");
    printf("\t\t With no argument only pulses PROG with Xilinx in SPI-Master mode\n");
}

// Maps VME from addr, len addresses
int w125c_Map(unsigned addr, unsigned len, int fd)
{
    int rc = 1;
    if (map.ptr != NULL) munmap(map.ptr, map.len);
    map.ptr = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, addr);
    if (map.ptr == MAP_FAILED)
    {
	map.addr = 0;
	map.len = 0;
	map.ptr = NULL;
	printf("Mapping region [%8.8X-%8.8X] failed with error %m\n",
	    addr, addr + len - 1);
    } else {
	map.addr = addr;
	map.len = len;
	rc = 0;
    }
    return rc;
}

// All FLASH subroutines assert PROG to tristate Xilinx and leave
// CPLD with asserted PROG and enabled FLASH access, but inactive FLASH CS

// Executes one FLASH command cmd
// if *adr is not NULL transfers 3 bytes of adr
// positive len: reads len bytes to buf
// negative len: writes len bytes from buf
// zero len: no data transfer
void w125c_FlashIO(char cmd, unsigned * adr, char * buf, int len) {
    
    int i;
    // assert PROG to disable Xilinx, enable flash access, no CS
    vwr(CSR, 0x22);
    // assert CS
    vwr(CSR, 0x23);
    // send command
    vwr(SDAT, cmd);
    // send addr if rqd
    if (adr) {
	vwr(SDAT, ((char *)adr)[2]);
	vwr(SDAT, ((char *)adr)[1]);
	vwr(SDAT, ((char *)adr)[0]);
    }
    if (len > 0) {
	// read byte by byte
	for (i=0; i<len; i++) {
    	    // cycle clocks
	    vwr(SDAT, 0);
	    // read result
	    buf[i] = vrd(SDAT);
	}
    } else if (len < 0) {
	// write byte by byte
	for (i=0; i<(-len); i++) {
    	    // write
	    vwr(SDAT, buf[i]);
	}
    }
    // deassert CS
    vwr(CSR, 0x22);
    return;
}

// Erases FLASH in minimal portions of subsectors 
// including those touched by addr and addr+len
// addr - full 24 bit beginning address
// len - length in bytes
// optimized to unite subsectors to sectors of entire flash where possible
// returns 0 on success
int w125c_FlashErase(unsigned addr, unsigned len) {
    // first subsector
    unsigned baddr = addr >> 12;
    // last subsector
    unsigned eaddr = (addr + len - 1) >> 12;
    // current full address
    unsigned caddr;
    char buf[2];
    int i;
    // Max operation time, in 100 ms intervals
    int timeout = 0;
    struct timespec ts;
    
    printf("W125C: INFO - Erasing subsectors %2.2X--%2.2X\n", baddr, eaddr);
    // Clear status flag reg (error bits)
    w125c_FlashIO(FCLRFL, NULL, NULL, 0);
    for ( ; baddr <= eaddr; ) {
	caddr = baddr << 12;
	// Write enable
	w125c_FlashIO(FWRENB, NULL, NULL, 0);
	// Check write enable bit
	w125c_FlashIO(FRDSTA, NULL, buf, 1);
	if (!(buf[0] & 0x02)) {
	    printf("\nW125C: ERASE FATAL - Cannot set WRITE ENABLE bit. Status %X\n", buf[0]);
	    return -1;
	}
	// if from first subsector to the last one -- erase all
	if (baddr == 0 && eaddr == 0xFFF) {
	    w125c_FlashIO(FBULKE, NULL, NULL, 0);
	    baddr += 0x1000;
	    printf("B");
	    fflush(stdout);
	    timeout = 250000;
	}
	// if from first subsector in sector to the last one or further -- erase this sector
	else if ((baddr & 0x00F) == 0 && eaddr >= (baddr | 0x00F)) {
	    w125c_FlashIO(FSECTE, &caddr, NULL, 0);
	    baddr += 0x10;
	    printf("S");
	    fflush(stdout);
	    timeout = 3000;
	}
	// otherwise erase this subsector 
	else {
	    w125c_FlashIO(FSSECE, &caddr, NULL, 0);
	    baddr += 0x1;
	    printf("s");
	    fflush(stdout);
	    timeout = 800;
	}
	// Check erase start
	w125c_FlashIO(FRDSTA, NULL, buf, 1);
	if (!(buf[0] & 0x01)) {
	    printf("\nW125C: ERASE FATAL - Erase didn't start\n");
	    return -2;
	}
	if (!(buf[0] & 0x02)) {
	    printf("\nW125C: ERASE FATAL - WRITE ENABLE bit unexpectedly cleared during erase\n");
	    return -3;
	}
	// Wait for termination
	ts.tv_sec = 0;
	ts.tv_nsec = 100000000; // 100 ms
	for (i=0; i < timeout; i++ ) {
	    // read status register
	    w125c_FlashIO(FRDSTA, NULL, buf, 1);
	    // lower bit 1 -- busy
	    if (!(buf[0] & 0x01)) break;
	    if (!((i+1)%10)) printf(".");
	    fflush(stdout);
	    nanosleep(&ts, NULL);
	}
	if (i >= timeout) {
	    printf("\nW125C: ERASE FATAL - Timeout waiting for opration end\n");
	    return -3;
	}	
    }
    printf("\n");
    return 0;
}

// Reads FLASH from addr to addr+len and checks for 0xFF values
// returns 0 on success
int w125c_FlashBCheck(unsigned addr, unsigned len) {
    unsigned char buf[4096];
    unsigned i;
    int toread, j;
    
    printf("W125C: INFO - Blank checking addresses %6.6X--%6.6X\n", addr, addr+len-1);
    for (i=addr; i<addr+len; ) {
	toread = (i+4096 < addr+len) ? 4096 : addr+len-i;
	w125c_FlashIO(FRDMEM, &i, buf, toread);
	for (j=0; j<toread; j++) {
	    if (buf[j] != 0xFF) {
		printf("\nW125C: BLANK CHECK FATAL - Failed at address 0x%6.6X: 0x%2.2X \n", i+j, buf[j] & 0xFF);
		return -1;
	    }
	}
	if (!(i & 0x1F000)) {
	    printf("b");
	    fflush(stdout);
	}
	i+= toread;
    }
    printf("\n");
    return 0;
}

// Writes binary file fname to FLASH starting at address addr
// writes are made in 256 byte pages, total length defined by file size
// FLASH should be preerased
// returns 0 on success
int w125c_FlashWrite(unsigned addr, char * fname) {

    FILE * f;
    int todo, i;
    unsigned char buf[256];
    unsigned char rbuf[2];
    unsigned caddr;
    struct timespec ts;
    
    // Open file
    f = fopen(fname, "rb");
    if (!f) {
	printf("W125C: WRITE FATAL - cannot open file %s\n", fname);
	return -1;
    }
    printf("W125C: INFO - Writing file %s\n", fname);
    // Clear status flag reg (error bits)
    w125c_FlashIO(FCLRFL, NULL, NULL, 0);
    // Read and program
    for (caddr=addr; ; ) {
	// reading up to page boundary
	todo = (caddr & 0xFFFF00) + 0x100 - caddr;
	// or up to end of file
	todo = fread(buf, 1, todo, f);
	// Program page
	// Write enable
	w125c_FlashIO(FWRENB, NULL, NULL, 0);
	// Check write enable bit
	w125c_FlashIO(FRDSTA, NULL, rbuf, 1);
	if (!(rbuf[0] & 0x02)) {
	    printf("\nW125C: WRITE FATAL - Cannot set WRITE ENABLE bit\n");
	    return -2;
	}
	// Start Program
	w125c_FlashIO(FPROGP, &caddr, buf, -todo);
	// Check erase start
	w125c_FlashIO(FRDSTA, NULL, rbuf, 1);
	if (!(rbuf[0] & 0x01)) {
	    printf("\nW125C: WRITE FATAL - Write didn't start\n");
	    return -3;
	}
	if (!(rbuf[0] & 0x02)) {
	    printf("\nW125C: WRITE FATAL - WRITE ENABLE bit unexpectedly cleared during write\n");
	    return -4;
	}
	// Wait for termination
	ts.tv_sec = 0;
	ts.tv_nsec = 100000; // 100 us
	for (i=0; i < 50; i++ ) {	// max 5 ms
	    // read status register
	    w125c_FlashIO(FRDSTA, NULL, rbuf, 1);
	    // lower bit 1 -- busy
	    if (!(rbuf[0] & 0x01)) break;
	    nanosleep(&ts, NULL);
	}
	if (i >= 50) {
	    printf("\nW125C: WRITE FATAL - Timeout waiting for opration end\n");
	    return -5;
	}
	if (!(caddr & 0x1FF00)) {
	    printf("w");
	    fflush(stdout);
	}
	caddr += todo;
	if (feof(f)) break;
    }
    fclose(f);
    printf("\nW125C: INFO - %d bytes written to flash\n", caddr - addr);
    return 0;
}

// Reads FLASH from address addr to addr+len to binary file fname
// returns 0 on success
int w125c_FlashRead(unsigned addr, unsigned len, char * fname) {
    unsigned char buf[4096];
    unsigned i;
    int toread, j;
    FILE * f;
    
    f = fopen(fname, "wb");
    if (!f) {
	printf("W125C: READ FATAL - cannot open file %s\n", fname);
	return -1;
    }
    printf("W125C: INFO - Reading FLASH addresses %6.6X--%6.6X to file %s\n", addr, addr+len-1, fname);
    for (i=addr; i<addr+len; ) {
	toread = (i+4096 < addr+len) ? 4096 : addr+len-i;
	w125c_FlashIO(FRDMEM, &i, buf, toread);
	fwrite(buf, toread, 1, f);
	if (!(i & 0x1F000)) {
	    printf("r");
	    fflush(stdout);
	}
	i+= toread;
    }
    fclose(f);
    printf("\n");
    return 0;
}

// Verifies FLASH from address addr against binary file fname
// verification length defined by file size
// returns 0 on success
int w125c_FlashVerify(unsigned addr, char * fname) {
    unsigned char buf[4096];
    unsigned char fbuf[4096];
    unsigned i;
    int toread, j;
    FILE * f;
    
    f = fopen(fname, "rb");
    if (!f) {
	printf("W125C: VERIFY FATAL - cannot open file %s\n", fname);
	return -1;
    }
    printf("W125C: INFO - Verifying FLASH against file %s\n", fname);
    for (i=addr; ; ) {
	toread = 4096;
	toread = fread(fbuf, 1, toread, f);
	w125c_FlashIO(FRDMEM, &i, buf, toread);
	for (j=0; j<toread; j++) {
	    if (buf[j] != fbuf[j]) {
		printf("\nW125C: VERIFY FATAL - Failed at address 0x%6.6X: 0x%2.2X \n", i+j, buf[j] & 0xFF);
		return -1;
	    }
	}
	if (!(i & 0x1F000)) {
	    printf("v");
	    fflush(stdout);
	}
	i+= toread;
	if (feof(f)) break;
    }
    fclose(f);
    printf("\nW125C: INFO - %d bytes verified\n", i - addr);
    return 0;
}

// Loads file fname directly to Xilinx, amount defined by file size
// manipulates PROG as necessary
// returns 0 on success
int w125c_XilinxLoad(char * fname) {
    
    int i, j, todo;
    FILE * f;
    unsigned char buf[4096];

    f = fopen(fname, "rb");
    if (!f) {
	printf("W125C: XILIXLOAD FATAL - cannot open file %s\n", fname);
	return -1;
    }
    printf("W125C: INFO - Loading Xilinx with file %s\n", fname);
    // assert PROG with enabled Xilinx access
    vwr(CSR, 0x30);
    // remove PROG, xilinx acess enabled
    vwr(CSR, 0x10);
    // wait for INIT
    for (i=0; i<1000; i++) if (vrd(CSR) & 0x40) break;
    if (i >= 1000) {
	printf("W125C: XILIXLOAD FATAL - no INIT after PROG\n");
	return -2;
    }
    // Load data
    for (i=0;;) {
	// read buffer
	todo = sizeof(buf);
	// or up to end of file
	todo = fread(buf, 1, todo, f);
	for (j=0; j<sizeof(buf); j++) vwr(SDAT, buf[j]); 
	if (!(i & 0x1F000)) {
	    printf("x");
	    fflush(stdout);
	}
	i += todo;
	if (feof(f)) break;
    }
    // disable Xilinx access
    vwr(CSR, 0x00);
    
    fclose(f);
    printf("\nW125C: INFO - %d bytes programmed to Xilinx\n", i);
    return 0;
}

// Waits specified timeout in seconds for DONE to go high
// returns 0 on success
int w125c_WaitDone(int timeout) {
    int i;
    unsigned char buf;
    char sym[] = {'.',':',';','"'};
    
    // Wait for DONE
    for (i=0; i < timeout; i++) {
	buf = vrd(CSR);
	// Done obtained ?
	if (buf & 0x80) break;
	printf("%c", sym[buf >> 6]);
	    fflush(stdout);
	    sleep(1);
    }
    if (i<timeout) { 
	printf(" *** DONE ***\n");
	return 0;
    } else {
	printf(" !!! NOT Done !!!\n");
	return -1;
    }
}

// According to Usage()
int main(int argc, char **argv)
{
    int fd;
    int rc;
    int N, i;
    int noprog = 0;
    unsigned char buf[30];
    unsigned addr, len;
    struct vme_master master;
    char flashID[] = {0x20, 0xBA, 0x18, 0x10};
    struct stat st;
    
    printf("*** WFD125 Programming through CPLD tool (c) SvirLex 2014 ***\n");
//	Open VME in A16/D16 and map entire region
    fd = open("/dev/bus/vme/m0", O_RDWR);
    if (fd == -1) {
        perror("W125C: FATAL - can not open VME");
        printf("Try running:\n\tmodprobe vme\n\tmodprobe vme_tsi148\n\tmodprobe vme_user bus=0\n");
        return EXIT_FAILURE;
    }

    master.enable = 1;
    master.vme_addr = 0x0;
    master.size = 0x10000;
    master.aspace = VME_A16;
    master.cycle = VME_USER | VME_DATA;
    master.dwidth = VME_D16;

    rc = ioctl(fd, VME_SET_MASTER, &master);
    if (rc != 0) {
	perror("W125C: FATAL - can not setup VME window");
	goto Quit;
    }
    rc = w125c_Map(0, 0x10000, fd);
    if (rc) {
	printf("W125C: FATAL - can not map A16 memory - %m\n");
	goto Quit;
    }
//  	Decode serial and check
    if (argc < 3) { w125c_Usage(); goto Quit; }
    maddr = BASE + ((N=strtoul(argv[1], NULL, 0)) << 4);
//	Check module serial
    if (N != vrd(SNUM)) {
	printf("W125C: FATAL - No module with serial number %d found OR CPLD not configured\n", N);
	goto Quit;
    }
//	Check FLASH ID
    w125c_FlashIO(FGETID, NULL, buf, 20);
    if (memcmp(buf, flashID, sizeof(flashID))) {
        printf("W125C: FATAL -- wrong flash ID found or flash unreliable\nexpect:\t");
        for (i=0; i<sizeof(flashID); i++) printf("%2.2X ", flashID[i] & 0xFF);
        printf("\nobtain:\t");
        for (i=0; i<sizeof(flashID); i++) printf("%2.2X ", buf[i] & 0xFF);
        printf("\n");
        goto QuitAfterFlash;
    }
    printf("*** Found module with Serial:%d Batch:%d Flash MfcID:%2.2X MemType:%2.2X MemCap:%2.2X\n",
	vrd(SNUM), vrd(BNUM), buf[0] & 0xFF, buf[1] & 0xFF, buf[2] & 0xFF);

    rc = 0;
//	Decode command and args	
    switch (toupper(argv[2][0])) {
	case 'E':
	    switch (argc) { 
		case 3:		// Entire Flash
		    w125c_FlashErase(0, FSIZE);
		    break;
		case 4:		// <bytes> from zero
		    len = strtoul(argv[3], NULL, 0);
		    if (len > FSIZE) len = FSIZE;
		    w125c_FlashErase(0, len);
		    break;
		default:
		    addr = strtoul(argv[4], NULL, 0);
		    len = strtoul(argv[3], NULL, 0);
		    if (addr > FSIZE) break;
		    if (addr + len > FSIZE) len = FSIZE - addr;
		    w125c_FlashErase(addr, len);
		    break;
	    }
	    break;
	case 'B':
	    switch (argc) { 
		case 3:		// Entire Flash
		    w125c_FlashBCheck(0, FSIZE);
		    break;
		case 4:		// <bytes> from zero
		    len = strtoul(argv[3], NULL, 0);
		    if (len > FSIZE) len = FSIZE;
		    w125c_FlashBCheck(0, len);
		    break;
		default:
		    addr = strtoul(argv[4], NULL, 0);
		    len = strtoul(argv[3], NULL, 0);
		    if (addr > FSIZE) break;
		    if (addr + len > FSIZE) len = FSIZE - addr;
		    w125c_FlashBCheck(addr, len);
		    break;
	    }
	    break;
	case 'W':
	    if (argc < 4) {
		w125c_Usage();
		break;
	    }
	    w125c_FlashWrite(0, argv[3]);
	    break;
	case 'R':
	    if (argc < 4) {
		w125c_Usage();
		break;
	    } else if (argc > 4) {
		// amount given
		len = strtoul(argv[4], NULL, 0);
		w125c_FlashRead(0, len, argv[3]);
	    } else {
		w125c_FlashRead(0, FSIZE, argv[3]);
	    }
	    break;
	case 'V':
	    if (argc < 4) {
		w125c_Usage();
		break;
	    }
	    w125c_FlashVerify(0, argv[3]);
	    break;
	case 'A':
	    if (argc < 4) {
		w125c_Usage();
		break;
	    }
	    if (stat(argv[3], &st)) {
		printf("W125C: FATAL - Cannot acess file %s\n", argv[3]);
		break;
	    }
	    len = st.st_size;
	    if (w125c_FlashErase(0, len)) break;
	    if (w125c_FlashBCheck(0, len)) break;
	    if (w125c_FlashWrite(0, argv[3])) break;
	    if (w125c_FlashVerify(0, argv[3])) break;
	    // Pulse prog here
	    vwr(CSR, 0x20);
	    vwr(CSR, 0x00);
	    w125c_WaitDone(30);
	    noprog = 1;
	    break;
	case 'P':
	    if (argc < 4) {
		// assert PROG, FLASH and Xilinx disabled = Xilinx SPI master
		vwr(CSR, 0x20);
		// remove PROG
		vwr(CSR, 0x00);
		rc = w125c_WaitDone(30);
	    } else {
		w125c_XilinxLoad(argv[3]);
		rc = w125c_WaitDone(3);
	    }
	    noprog = 1;
	    break;
	default:
	    w125c_Usage();
	    break;
    }

QuitAfterFlash:
//	After FLASH commands: disable FLASH and Xilinx access, but leave PROG asserted
    if (!noprog) vwr(CSR, 0x20);
Quit:
//	Close VME	
    close(fd);
    return rc;
}
