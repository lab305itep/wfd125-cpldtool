/*
    SvirLex 2014 - wfd125 manipulation tool through its CPLD via VME with TSI148 chipset
    Using MEN drivers
*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <MEN/vme4l.h>
#include <MEN/vme4l_api.h>
#include <readline/readline.h>

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
// FLASH sizes (bytes)
#define FSIZE 0x1000000

#ifndef SWAP_MODE
#define SWAP_MODE VME4L_NO_SWAP
#endif

#if (SWAP_MODE == VME4L_NO_SWAP)
#define SWAP2(A) __swap2(A)
#else
#define SWAP2(A) (A)
#endif

typedef struct {
    unsigned addr;
    unsigned len;
    unsigned *ptr;
} VMEMAP;

VMEMAP map = {0, 0, NULL};
// module address
unsigned maddr;

unsigned __swap2(unsigned short i)
{
    union SWP {
	unsigned short u;
	char c[2];
    } a, b;
    a.u = i;
    b.c[0] = a.c[1];
    b.c[1] = a.c[0];
    return b.u;
}

unsigned char vrd(unsigned adr)
{
    union SWP {
	unsigned short u;
	char c[2];
    } a;
    a.u = ((unsigned short *)map.ptr)[(adr + maddr)/2];
    return a.c[1];
}

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


void RegTest(int N, unsigned repeat, VMEMAP *map)
{
    int i;
    volatile unsigned *A;
    volatile unsigned *T;
    unsigned WA, RA, WT, RT;
    unsigned err;
    
    A = &(map->ptr[N*0x40 + 1]);
    T = &(map->ptr[N*0x40 + 3]);
    err = 0;
    srand48(time(0));
    
    for (i=0; i<repeat; i++) {
	WA = mrand48();
	WT = mrand48();
	*A = WA;
	*T = WT;
	RA = *A;
	RT = *T;
	if (WA != RA || WT != RT) {
	    err++;
	    if (err < 100) printf("%6.6X: W(%8.8X %8.8X) != R(%8.8X %8.8X)\n", i, WA, WT, RA, RT);
	}
    }
    printf("Test finished with %d errors.\n", err);
}


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
    printf("\tWrite <filename> -- writes binary file to preerased FLASH from addr 0.\n");
    printf("\tRead <filename> [<bytes>] -- reads <bytes> from FLASH to binary file.\n");
    printf("\tVerify <filename> -- verifies FLASH against binary file.\n");
    printf("\tAutowrite <filename> -- does Erase (by file length), blank check, write and verify.");
    printf("\tProgram [<filename>] -- loads binary file directly to FPGA chain.\n");
    printf("\t\t With no argument only pulses PROG with Xilinx in SPI-Master mode\n");
}

int w125c_Map(unsigned addr, unsigned len, int fd)
{
    int rc;
    if (map.ptr != NULL) VME4L_UnMap(fd, map.ptr, map.len);
    rc = VME4L_Map(fd, addr, len, (void **)&map.ptr);
    if (rc != 0) {
	map.addr = 0;
	map.len = 0;
	map.ptr = NULL;
	printf("Mapping region [%8.8X-%8.8X] failed with error %m\n",
	    addr, addr + len - 1);
    } else {
	map.addr = addr;
	map.len = len;
    }
    return rc;
}

void w125c_FlashRead(char cmd, unsigned * adr, char * buf, int len) {
    
    int i;
    // assert PROG to disable Xilinx, enable flash access, no CS
    vwr(CSR, 0x22);
    // assert CS
    vwr(CSR, 0x23);
    // send command
    vwr(SDAT, cmd);
    vrd(CSR);
    // send addr if rqd
    if (adr) {
	vwr(SDAT, ((char *)adr)[2]);
	vrd(CSR);
	vwr(SDAT, ((char *)adr)[1]);
	vrd(CSR);
	vwr(SDAT, ((char *)adr)[0]);
	vrd(CSR);
    }
    // read byte by byte
    for (i=0; i<len; i++) {
    	// cycle clocks
	vwr(SDAT, 0);
	// small delay
	vrd(CSR);
	// read result
	buf[i] = vrd(SDAT);
    }
    // deassert CS
    vwr(CSR, 0x22);
    return;
}
 
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
    w125c_FlashRead(FCLRFL, NULL, NULL, 0);
    for ( ; baddr <= eaddr; ) {
	caddr = baddr << 12;
	// Write enable
	w125c_FlashRead(FWRENB, NULL, NULL, 0);
	// Check write enable bit
	w125c_FlashRead(FRDSTA, NULL, buf, 1);
	if (!(buf[0] & 0x02)) {
	    printf("\nW125C: ERASE FATAL - Cannot set WRITE ENABLE bit\n");
	    return -1;
	}
	// if from first subsector to the last one -- erase all
	if (baddr == 0 && eaddr == 0xFFF) {
	    w125c_FlashRead(FBULKE, NULL, NULL, 0);
	    baddr += 0x1000;
	    printf("B");
	    fflush(stdout);
	    timeout = 250000;
	}
	// if from first subsector in sector to the last one or further -- erase this sector
	else if ((baddr & 0x00F) == 0 && eaddr >= (baddr | 0x00F)) {
	    w125c_FlashRead(FSECTE, &caddr, NULL, 0);
	    baddr += 0x10;
	    printf("S");
	    fflush(stdout);
	    timeout = 3000;
	}
	// otherwise erase this subsector 
	else {
	    w125c_FlashRead(FSSECE, &caddr, NULL, 0);
	    baddr += 0x1;
	    printf("s");
	    fflush(stdout);
	    timeout = 800;
	}
	// Check erase start
	w125c_FlashRead(FRDSTA, NULL, buf, 1);
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
	    w125c_FlashRead(FRDSTA, NULL, buf, 1);
	    // lower bit 1 -- busy
	    if (!(buf[0] & 0x01)) break;
	    if (!((i+1)%10)) printf(".");
	    fflush(stdout);
	    nanosleep(&ts, NULL);
	}
	if (i >= timeout/100) {
	    printf("\nW125C: ERASE FATAL - Timeout waiting for opration end\n");
	    return -3;
	}	
    }
    printf("\n");
    return 0;
}

int w125c_FlashBCheck(unsigned addr, unsigned len) {
    unsigned char buf[4096];
    unsigned i;
    int toread, j;
    
    printf("W125C: INFO - Blank checking addresses %6.6X--%6.6X\n", addr, addr+len-1);
    for (i=addr; i<addr+len; ) {
	toread = (i+4096 < addr+len) ? 4096 : addr+len-i;
	w125c_FlashRead(FRDMEM, &i, buf, toread);
	for (j=0; j<toread; j++) {
	    if (buf[j] != 0xFF) {
		printf("\nW125C: BLANK CHECK FATAL - Failed at address 0x%6.6X: 0x%2.2X \n", buf[j] & 0xFF);
		return -1;
	    }
	}
	if (!(i & 0x1F000)) {
	    printf(".");
	    fflush(stdout);
	}
	i+= toread;
    }
    printf("\n");
    return 0;
}

int main(int argc, char **argv)
{
    int fd;
    int rc;
    int N, i;
    char buf[4096];
    unsigned addr, len;
    VME4L_SPACE spc, spcr;
    vmeaddr_t vmeaddr;
    char flashID[] = {0x20, 0xBA, 0x18, 0x10};
    
    printf("*** WFD125 Programming through CPLD tool (c) SvirLex 2014 ***\n");
//	Open VME in A16/D16 and map entire region
    fd = VME4L_Open(0);
    if (fd < 0) {
	printf("W125C: FATAL - can not open VME - %m\n");
	return fd;
    }
    rc = VME4L_SwapModeSet(fd, SWAP_MODE);
    if (rc) {
	printf("W125C: FATAL - can not set swap mode - %m\n");
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
    w125c_FlashRead(FGETID, NULL, buf, 20);
    if (memcmp(buf, flashID, sizeof(flashID))) {
        printf("W125C: FATAL -- wrong flash ID found or flash unreliable\nexpect:\t");
        for (i=0; i<sizeof(flashID); i++) printf("%2.2X ", flashID[i] & 0xFF);
        printf("\nobtain:\t");
        for (i=0; i<sizeof(flashID); i++) printf("%2.2X ", buf[i] & 0xFF);
        printf("\n");
        goto Quit;
    }
    printf("*** Found module with Serial:%d Batch:%d Flash MfcID:%2.2X MemType:%2.2X MemCap:%2.2X\n",
	vrd(SNUM), vrd(BNUM), buf[0] & 0xFF, buf[1] & 0xFF, buf[2] & 0xFF);

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
    }

/*
    for(;;) {
	if (cmd) free(cmd);
	cmd = readline("VmeBur (H-help)>");
	if (cmd == NULL || strlen(cmd) == 0) continue;
	add_history(cmd);
	tok = strtok(cmd, DELIM);
	if (tok == NULL || strlen(tok) == 0) continue;
	switch(toupper(tok[0])) {
	case '*':	// Comment
	    break;
	case '0':
	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
	case '7':
	case '8':
	case '9':
	case 'A':
	case 'B':
	case 'C':
	case 'D':
	case 'E':
	case 'F':
	    if (map.ptr == NULL) {
		printf("Map some region first.\n");
		break;
	    }
	    addr = strtoul(tok, NULL, 16);
	    if (addr+4 > map.len) {
		printf("Shift (%8.8X) above the mapped length (%8.8X)\n", addr, map.len);
	    }
	    tok = strtok(NULL, DELIM);
	    if (tok == NULL || strlen(tok) == 0) {	// read
		switch (mode) {
		case 'L':
		    printf("VME[%8.8X + %8.8X] = %8.8X\n", map.addr, addr, SWAP(map.ptr[addr/4]));
		    break;
		case 'S':
		    printf("VME[%8.8X + %8.8X] = %4.4hX\n", map.addr, addr, SWAP2(((unsigned short *)map.ptr)[addr/2]) & 0xFFFF);
		    break;
		case 'C':
		    printf("VME[%8.8X + %8.8X] = %2.2hhX\n", map.addr, addr, ((unsigned char *)map.ptr)[addr] & 0xFF);
		    break;
		}
	    } else {					// write
		len = strtoul(tok, NULL, 16);
		switch (mode) {
		case 'L':
		    map.ptr[addr/4] = SWAP(len);
		    printf("VME[%8.8X + %8.8X] <= %8.8X\n", map.addr, addr, len);
		    break;
		case 'S':
		    ((unsigned short *)map.ptr)[addr/2] = SWAP2(len) & 0xFFFF;
		    printf("VME[%8.8X + %8.8X] <= %4.4X\n", map.addr, addr, len);
		    break;
		case 'C':
		    ((unsigned char *)map.ptr)[addr] = len & 0xFF;
		    printf("VME[%8.8X + %8.8X] <= %2.2X\n", map.addr, addr, len);
		    break;
		}
	    }
	    rc = VME4L_BusErrorGet(fd, &spcr, &vmeaddr, 1 );
	    if (rc) printf("VME BUS ERROR: rc=%d @ spc=%d addr=0x%X\n", rc, spcr, vmeaddr);
	    break;
	case 'H':	// help
	    Help();
	    break;
	case 'M':	// Map address length
	    tok = strtok(NULL, DELIM);
	    if (tok == NULL || strlen(tok) == 0) {
		printf("VME region [%8.8X-%8.8X] is mapped at local address %8.8X\n",
		    map.addr, map.addr + map.len - 1, map.ptr);
		break;
	    }
	    addr = strtoul(tok, NULL, 16);
	    tok = strtok(NULL, DELIM);
	    if (tok == NULL || strlen(tok) == 0) {
		printf("Usage: Map address length\n");
		break;
	    }
	    len = strtoul(tok, NULL, 16);
	    Map(addr, len, &map, fd);
	    break;
	case 'P':	// Print [address [length]]
	    if (map.ptr == NULL) {
		printf("Map some region first.\n");
		break;
	    }
	    addr = 0;
	    len = map.len;
	    tok = strtok(NULL, DELIM);
	    if (tok != NULL && strlen(tok) != 0) {
		addr = strtoul(tok, NULL, 16);
		tok = strtok(NULL, DELIM);
		if (tok != NULL && strlen(tok) != 0) len = strtoul(tok, NULL, 16);
	    }
	    Dump(addr, len, &map);
	    break;
	case 'Q' :	// Quit / Exit
	case 'X' :
	    goto Quit;
	case 'R' :	// register read/write test
	    if (map.ptr == NULL) {
		printf("Map the region first. Most likely you need:\n\tM ADC16000 2000\n");
		break;
	    }
	    len = 10000;	// repeat counter
	    tok = strtok(NULL, DELIM);
	    if (tok == NULL || strlen(tok) == 0) {
		printf("Unit number is mandatory.\n");
		break;
	    }
	    N = 0x1F & strtol(tok, NULL, 16);
	    tok = strtok(NULL, DELIM);
	    if (tok != NULL && strlen(tok) != 0) len = strtoul(tok, NULL, 16);
	    RegTest(N, len, &map);
	    break;
	case 'T' :	// test memory
	    if (map.ptr == NULL) {
		printf("Map the region first. Most likely you need:\n\tM ADC16000 2000\n");
		break;
	    }
	    addr = 0;
	    len = 0x800000;	// 32 Mbytes = 8 Mdwords
	    tok = strtok(NULL, DELIM);
	    if (tok == NULL || strlen(tok) == 0) {
		printf("Unit number is mandatory.\n");
		break;
	    }
	    N = 0x1F & strtol(tok, NULL, 16);
	    tok = strtok(NULL, DELIM);
	    if (tok != NULL && strlen(tok) != 0) {
		addr = strtoul(tok, NULL, 16);
		tok = strtok(NULL, DELIM);
		if (tok != NULL && strlen(tok) != 0) len = strtoul(tok, NULL, 16);
	    }
	    MemTest(N, addr, len, &map);
	    break;
	default:
	    printf("Unknown command \"%c\"\n", toupper(tok[0]));
	}
    }
*/

Quit:
//	Close VME	
    if (map.ptr != NULL) VME4L_UnMap(fd, map.ptr, map.len);
    VME4L_Close(fd);
    return 0;
}