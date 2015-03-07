/*
 * This is the explorative (pre librarification) code
 * that was used to create a proof of concept
 * bluetooth scanner (CLI) application for the iPhone 3G.
 * (Cambridge Silicon Radio BlueCore 6)
 *
 * The copyright in this source file is hereby released into
 * the public domain. No warranty is given for its use.
 *
 */
#import <stdio.h>
#import <errno.h>
#import <fcntl.h>
#import <unistd.h>
#import <stdlib.h>
#import <string.h>
#import <signal.h>
#import <termios.h>
#import <sys/types.h>

#define DEV_UART "/dev/uart.bluetooth"
#define DEV_RSET "/dev/btreset"

#define HCI_COMMAND_PKT 0x01

int initialise() {
    
    int fd;
    open_btreset();
    
    // we're init'ing: send open_device a 1
    if((fd = open_device(1)) == -1){
        printf("Failed to open device\n");
        return -1;
    }
    
    if(boot_device(fd,0x00,0x23,0x12,0xd0,0xc6,0x1f) != 0 ){    // Release library should dynamically set BD address
        printf("BlueCore Initialisation Failed\n");
        return -1;
    }
    
    // give chip time to recover:
    sleep(1);
    close(fd);
    
    return 0;
}
int boot_device(int fd, int a, int b, int c, int d, int e, int f) {

    //Note: all PSVals in little endian format

    unsigned char psval[200];
    int psval_len; // in uint16's

    // set BD address
    psval[0] = d;
    psval[1] = 0x00;
    psval[2] = f;
    psval[3] = e;
    psval[4] = c;
    psval[5] = 0x00;
    psval[6] = b;
    psval[7] = a;

    psval_len = 4;

    if(pskey_set(fd, 0, 0x7003, 0x0001, 0x0000, psval, psval_len, 0) != 0){
        printf("1 failed\n");
        return -1;
    }


    /* Configure for a 26Mhz external clock input */
    psval[0] = 0x90;
    psval[1] = 0x65;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x01fe, 0x0000, psval, psval_len, 0) != 0){
        printf("1 failed\n");
        return -1;
    }

    /* Configure the UART speed */
    psval[0] = 0xb0;
    psval[1] = 0x03;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x01be, 0x0000, psval, psval_len, 0) != 0){
        printf("2 failed\n");
        return -1;
    }

    /* PCM config */
    psval[0] = 0x80;
    psval[1] = 0x08;
    psval[2] = 0x12;
    psval[3] = 0x00;
    psval_len = 2;

    if(pskey_set(fd, 0, 0x7003, 0x01b3, 0x0000, psval, psval_len, 0) != 0){
        printf("3 failed\n");
        return -1;
    }

    /* PCM config - continued */
    psval[0] = 0x60;
    psval[1] = 0x00;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x01b6, 0x0000, psval, psval_len, 0) != 0){
        printf("4 failed\n");
        return -1;
    }

    /* Configure TX GAIN ramp */
    psval[0] = 0x10;
    psval[1] = 0x24;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x001d, 0x0000, psval, psval_len, 0) != 0){
        printf("5 failed\n");
        return -1;

    }

    /* Configure the ACL Buffer size */
    psval[0] = 0xfe;
    psval[1] = 0x01;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x0011, 0x0000, psval, psval_len, 0) != 0){
        printf("6 failed\n");
        return -1;

    }

    /* Configure the number of ACL buffers */
    psval[0] = 0x06;
    psval[1] = 0x00;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x0013, 0x0000, psval, psval_len, 0) != 0){
        printf("7 failed\n");
        return -1;
    }

    /* Configure the number of SCO buffers */
    psval[0] = 0x02;
    psval[1] = 0x00;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x000e, 0x0000, psval, psval_len, 0) != 0){
        printf("8 failed\n");
        return -1;
    }

    /* Co-existence Setup - Set the BT_TX line as PIO_9 */
    psval[0] = 0x00;
    psval[1] = 0x20;
    psval[2] = 0x00;
    psval[3] = 0x00;
    psval[4] = 0x00;
    psval[5] = 0x00;
    psval_len = 3;

    if(pskey_set(fd, 0, 0x7003, 0x001d, 0x0000, psval, psval_len, 0) != 0){
        printf("9 failed\n");
        return -1;
    }

    /* Co-existence Setup - set PIO pins used for channel number */
    psval[0] = 0x11;
    psval[1] = 0x00;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x002a, 0x0000, psval, psval_len, 0) != 0){
        printf("10 failed\n");
        return -1;
    }

    /* Co-existence Setup - set PIO1 to be driven high to enable the External PA */
    psval[0] = 0x01;
    psval[1] = 0x00;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x0209, 0x0000, psval, psval_len, 0) != 0){
        printf("11 failed\n");
        return -1;
    }
    /* Lets go into H*4* mode */
    psval[0] = 0x03;
    psval[1] = 0x00;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x01f9, 0x0000, psval, psval_len, 0) != 0){
        printf("12 failed\n");
        return -1;
    }
    /* PSKEY_UART_HOST_WAKE_SIGNAL (For BC6 we are using PIO3 - Needs transition from Low to High to trigger) */
    psval[0] = 0x31;
    psval[1] = 0x00;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x01ca, 0x0000, psval, psval_len, 0) != 0){
        printf("13 failed\n");
        return -1;
    }
    /* PSKEY_UART_HOST_WAKE : enable, sleep timeout = 500ms, break len = 5ms, pause length = 32ms*/
    psval[0] = 0x01;
    psval[1] = 0x00;
    psval[2] = 0xf4;
    psval[3] = 0x01;
    psval[4] = 0x05;
    psval[5] = 0x00;
    psval[6] = 0x20;
    psval[7] = 0x00;
    psval_len = 4;

    if(pskey_set(fd, 0, 0x7003, 0x01c7, 0x0000, psval, psval_len, 0) != 0){
        printf("14 failed\n");
        return -1;
    }
    /* PSKEY_UART_ACK_TIMEOUT : Set the ack timeout for messages sent to the host - Currently: 200ms */
    psval[0] = 0xc8;
    psval[1] = 0x00;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x01c4, 0x0000, psval, psval_len, 0) != 0){
        printf("15 failed\n");
        return -1;
    }
    /* Turn on power switch */
    psval[0] = 0x01;
    psval[1] = 0x00;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x0209, 0x0000, psval, psval_len, 0) != 0){
        printf("16 failed\n");
        return -1;
    }

    /* Optimised Power table -- note always assumes wifi present */
        psval[0] = 0x00;
        psval[1] = 0x16;
        psval[2] = 0x50;
        psval[3] = 0x00;
        psval[4] = 0x00;
        psval[5] = 0x14;
        psval[6] = 0x50;
        psval[7] = 0x00;
        psval[8] = 0x00;
        psval[9] = 0xeb;
        psval[10] = 0x00;
        psval[11] = 0x21;
        psval[12] = 0x50;
        psval[13] = 0x00;
        psval[14] = 0x00;
        psval[15] = 0x1e;
        psval[16] = 0x50;
        psval[17] = 0x00;
        psval[18] = 0x00;
        psval[19] = 0xf0;
        psval[20] = 0x00;
        psval[21] = 0x20;
        psval[22] = 0x40;
        psval[23] = 0x00;
        psval[24] = 0x00;
        psval[25] = 0x1d;
        psval[26] = 0x40;
        psval[27] = 0x00;
        psval[28] = 0x00;
        psval[29] = 0xf5;
        psval[30] = 0x00;
        psval[31] = 0x24;
        psval[32] = 0x30;
        psval[33] = 0x00;
        psval[34] = 0x00;
        psval[35] = 0x20;
        psval[36] = 0x30;
        psval[37] = 0x00;
        psval[38] = 0x00;
        psval[39] = 0xfa;
        psval[40] = 0x00;
        psval[41] = 0x25;
        psval[42] = 0x10;
        psval[43] = 0x00;
        psval[44] = 0x00;
        psval[45] = 0x25;
        psval[46] = 0x20;
        psval[47] = 0x00;
        psval[48] = 0x00;
        psval[49] = 0xff;
        psval[50] = 0x00;
        psval[51] = 0x2b;
        psval[52] = 0x00;
        psval[53] = 0x00;
        psval[54] = 0x00;
        psval[55] = 0x2d;
        psval[56] = 0x10;
        psval[57] = 0x00;
        psval[58] = 0x04;
        psval[59] = 0x00;
        psval[60] = 0x00;
        psval[61] = 0x3a;
        psval[62] = 0x00;
        psval[63] = 0x00;
        psval[64] = 0x00;
        psval[65] = 0x3a;
        psval[66] = 0x00;
        psval[67] = 0x00;
        psval[68] = 0x09;
        psval[69] = 0x00;
    psval_len = 35;

    if(pskey_set(fd, 0, 0x7003, 0x0031, 0x0000, psval, psval_len, 0) != 0){
        printf("17 failed\n");
        return -1;
    }

    /* PSKEY_LM_TEST_SEND_ACCEPTED_TWICE */
        psval[0] = 0x01;
        psval[1] = 0x00;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x00f6, 0x0000, psval, psval_len, 0) != 0){
        printf("18 failed\n");
        return -1;
    }
    /* Boost power for basic rate by running mixer and baseband flat out */
        psval[0] = 0x07;
        psval[1] = 0x00;
        psval[2] = 0x03;
        psval[3] = 0x00;
    psval_len = 2;

    if(pskey_set(fd, 0, 0x7003, 0x0243, 0x0000, psval, psval_len, 0) != 0){
        printf("19 failed\n");
        return -1;
    }
        psval[0] = 0x07;
        psval[1] = 0x00;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x03af, 0x0000, psval, psval_len, 0) != 0){
        printf("20 failed\n");
        return -1;
    }
        psval[0] = 0x3f;
        psval[1] = 0x00;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x21e9, 0x0000, psval, psval_len, 0) != 0){
        printf("21 failed\n");
        return -1;
    }
        
    /* Make sure we run with half MHz offsets */
        psval[0] = 0xfe;
        psval[1] = 0xff;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x0217, 0x0000, psval, psval_len, 0) != 0){
        printf("22 failed\n");
        return -1;
    }
        psval[0] = 0xfe;
        psval[1] = 0xff;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x21d1, 0x0000, psval, psval_len, 0) != 0){
        printf("23 failed\n");
        return -1;
    }
        psval[0] = 0xff;
        psval[1] = 0xff;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x21d0, 0x0000, psval, psval_len, 0) != 0){
        printf("24 failed\n");
        return -1;
    }
        psval[0] = 0xff;
        psval[1] = 0xff;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x21cf, 0x0000, psval, psval_len, 0) != 0){
        printf("25 failed\n");
        return -1;
    }
        
    /* Put default power beyond end of power table */
        psval[0] = 0x14;
        psval[1] = 0x00;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x0021, 0x0000, psval, psval_len, 0) != 0){
        printf("26 failed\n");
        return -1;
    }
        psval[0] = 0x14;
        psval[1] = 0x00;
    psval_len = 1;

    if(pskey_set(fd, 0, 0x7003, 0x0017, 0x0000, psval, psval_len, 0) != 0){
        printf("27 failed\n");
        return -1;
    }


    /* Warm reset for the keys to take effect */
    psval_len = 0;
    if(pskey_set(fd, 0, 0x4002, 0x0000, 0x0000, NULL, psval_len, 1) != 0) {
        return -1;
    }

    return 0;

}
void uninitialise(int fd) {
    
    //Cold Halt - will never fail due to power cycling
    pskey_set(fd, 0, 0x4003, 0x0000, 0x0000, NULL, 0, 1);
 
    close(fd);
}
int hci_inquiry(int fd, char host_addr[][], unsigned char host_psrep_mode[][], char host_cod[][], char host_clkoff[][], char host_fname[][]) {

    unsigned char cmd[300];		/* Command - needs to take friendly name (248 bytes...)*/
    unsigned char resp[300];	/* Response */
    int  clen = 0;              /* Command len */
    unsigned short int inv_little_clkoff[100];

    int hosts = 0; //number of hosts found

    //write scan enable
    cmd[0] = HCI_COMMAND_PKT;
    cmd[1] = 0x1a; //inq OCF
    cmd[2] = 0x03 << 2; //Link Control OGF (<<2)
    cmd[3] = 0x01; //param len (NOT num)
    //end header
    cmd[4] = 0x03; // LITTLE ENDIAN lap (GIAC) general inq access code...

    clen = 5;

    do {
        if (write(fd,cmd,clen) != clen){
            printf("Write failed: %s",strerror(errno));
            return -1;
        }
        
        if (read_hci_event(fd, resp, 100, 0) < 0 ) { // if hci_read_event returns count < 0 thats a fail?
            printf("Read failed\n");
            return -1;
        }
    } while(resp[1] != 0x0e); //command status

    memset(cmd,0,5);
    memset(resp,0,30);

    //write class of device
    cmd[0] = HCI_COMMAND_PKT;
    cmd[1] = 0x24; //inq OCF
    cmd[2] = 0x03 << 2; //Link Control OGF (<<2)
    cmd[3] = 0x03; //param len (NOT num)
    //end header
    cmd[4] = 0x0c;
    cmd[5] = 0x02;
    cmd[6] = 0x70;

    clen = 7;

    do {
        if(write(fd,cmd,clen) != clen){
            printf("Write failed: %s",strerror(errno));
            return -1;
        }

        if(read_hci_event(fd, resp, 100, 0) < 0 ) { // if hci_read_event returns count < 0 thats a fail?
            printf("Read failed\n");
            return -1;
        }

    } while(resp[1] != 0x0e); //command complete
    
    //write friendly name
    char friendly[248] = "pwned bt";
    int flen;

    flen = strlen(friendly);

    cmd[0] = HCI_COMMAND_PKT;
    cmd[1] = 0x13; //Write BT Friendly Name OCF
    cmd[2] = 0x03 << 2; //Link Control OGF (<<2)
    cmd[3] = 0xf8; //Parameters: 248 bytes (in dec)

    memcpy(cmd+4,friendly,248);
    memset(cmd+4+flen,0,248-flen);

    clen = 4+248;

    do {
        if(write(fd,cmd,clen) != clen){
            printf("Write failed: %s",strerror(errno));
            return -1;
        }
        
        if(read_hci_event(fd, resp, 100, 0) < 0 ) { // if hci_read_event returns count < 0 thats a fail?
            printf("Read failed\n");
            return -1;
        }

    } while(resp[1] != 0x0e); //command status

    memset(cmd,0,5);
    memset(resp,0,30);

    //enter inquiry mode
    cmd[0] = HCI_COMMAND_PKT;
    cmd[1] = 0x01; //inq OCF
    cmd[2] = 0x04; //Link Control OGF (<<2)
    cmd[3] = 0x05; //param len (NOT num)
    //end header
    cmd[4] = 0x33; // LITTLE ENDIAN lap (GIAC) general inq access code...
    cmd[5] = 0x8b; // middle significant byte
    cmd[6] = 0x9e; // most significant byte
    cmd[7] = 0x08; // time in near seconds (10)
    cmd[8] = 0x00; // number of responses (0)

    clen = 9;

    do {
        if(write(fd,cmd,clen) != clen){
            printf("Write failed: %s",strerror(errno));
            return -1;
        }

        if(read_hci_event(fd, resp, 200, 0) < 0 ) { // if hci_read_event returns count < 0 thats a fail?
            printf("Read failed\n");
            return -1;
        }
        /* do until the first byte recieved is 0xff, which is the event code for bluecore specific events */

    } while(resp[1] != 0x0f); //command status

    do {
        if(read_hci_event(fd, resp, 100, 0) < 0 ) { // if hci_read_event returns count < 0 thats a fail?
            printf("Read failed\n");
            return -1;
        }

        if(resp[1] == 0x02){

            printf("\n %02x Host(s) Found - BD_ADDR: %02x:%02x:%02x:%02x:%02x:%02x\n", resp[3], resp[9],resp[8],resp[7],resp[6],resp[5],resp[4]);
            printf("Page scan rep mode %02x\n", resp[10]);
            printf("class of device %02x%02x%02x\n", resp[15],resp[14],resp[13]);
            printf("clock offset %02x%02x\n\n", resp[17],resp[16]);

            inv_little_clkoff[hosts] = resp[17] | (resp[16] << 8);

            int p;
            for(p=0; p<=5;++p){
                host_addr[hosts][p] = resp[9-p];
            }

            host_psrep_mode[hosts][0] = resp[10];

            //in little endian format NOTE!!!!!
            sprintf(host_clkoff[hosts], "%02x%02x",resp[16],resp[17]);

            hosts = hosts + 1;
        }

    } while(resp[1] != 0x01); //command status

    memset(cmd,0,5);
    memset(resp,0,30);

    //get friendly names now:
    int y;
    for(y=0; y <= (hosts-1); ++y) {

        //request friendly name, for device bd_addr
        cmd[0] = HCI_COMMAND_PKT;
        cmd[1] = 0x19; //inq OCF
        cmd[2] = 0x01 <<2; //Link Control OGF (<<2)
        cmd[3] = 10; //param len (NOT num)
        //end header
        cmd[4] = host_addr[y][5];
        cmd[5] = host_addr[y][4];
        cmd[6] = host_addr[y][3];
        cmd[7] = host_addr[y][2];
        cmd[8] = host_addr[y][1];
        cmd[9] = host_addr[y][0];
        //end bd_addr 
        //ps rep mode
        cmd[10] = host_psrep_mode[y][0];
        //reserved 
        cmd[11] = 0x00;
        //start clock offset
        cmd[12] = inv_little_clkoff[y] >> 1; 
        cmd[13] = inv_little_clkoff[y];

        clen = 14;

        do {
            if(write(fd,cmd,clen) != clen){
                printf("Write failed: %s",strerror(errno));
                return -1;
            }

            if(read_hci_event(fd, resp, 100, 0) < 0 ) {
                printf("Read failed\n");
                return -1;
            }

        } while(resp[1] != 0x0f); //command status

        do {
            if(read_hci_event(fd, resp, 100, 0) < 0 ) { // if hci_read_event returns count < 0 thats a fail?
                printf("Read failed\n");
                return -1;
            }
        } while(resp[1] != 0x07); //remote name request complete event

        //put the names into host_fname array
        memset(host_fname[y],0,250);
        int q;
        
        for(q=1; q<=248; ++q){
            if(resp[q+9] == 0x00) break;
            
            if(q == 248 && resp[q+9] != 0x00){
                host_fname[y][q-1] = resp[q+9];
                host_fname[y][q] = 0x00;
                break;
            }

            host_fname[y][q-1] = resp[q+9];
        }

        printf("\n Host Friendly Name: %s\n\n",host_fname[y]);

        }

    return hosts;
}

int read_hci_event(int fd, unsigned char* buf, int size, int debug) {
    
    int remain, r;
    int count = 0;
    
    if (size <= 0) return -1;
    
    /* The first byte identifies the packet type. For HCI event packets, it
     * should be 0x04, so we read until we get to the 0x04. */
    while (1) {
        r = read(fd, buf, 1);
        if (r <= 0){
            return -1;
        }
        if (buf[0] == 0x04)
            break;
    }
    count++;
    
    /* The next two bytes are the event code and parameter total length. */
    while (count < 3) {
        r = read(fd, buf + count, 3 - count);
        if (r <= 0)
            return -1;
        count += r;
    }
    
    /* Now we read the parameters. */
    if (buf[2] < (size - 3)) {
        remain = buf[2];
    } else {
        remain = size - 3;
    }
    
    while ((count - 3) < remain) {
        r = read(fd, buf + count, remain - (count - 3));
        if (r <= 0)
            return -1;
        count += r;
    }
    
    if(debug) {
        int x;
        for(x=0;x<25;++x){
            printf("thingo: 0x%02x\n",buf[x]);
        }
        printf("\n");
    }
    
    return count;
}
int open_btreset() {
    
    int fd;
    
    if((fd = open(DEV_RSET, O_RDWR | O_NOCTTY)) == -1){ // read/write access, do not become controlling terminal
        printf("Error opening device: %s\n", strerror(errno));
        return -1;
    }
    
    tcflush(fd, TCIOFLUSH);
    close(fd);
    return 0;
}
int open_device(int init) {
    
    struct termios ti;
    int fd;
    
    if((fd = open(DEV_UART, O_RDWR | O_NOCTTY)) == -1){ // read/write access, do not become controlling terminal
        printf("Error opening device: %s\n", strerror(errno));
        return -1;
    }
    
    tcflush(fd, TCIOFLUSH);
    
    if(tcgetattr(fd, &ti) == -1){
        printf("Error getting attributes: %s\n", strerror(errno));
        return -1;
    }
    
    cfmakeraw(&ti);
    ti.c_cflag |= CLOCAL;  //ignore modem control lines
    ti.c_cflag |= CRTSCTS; //enable hardware flow control
    
    if(tcsetattr(fd, TCSANOW, &ti) == -1){ //set attributes now
        printf("Error setting attributes: %s\n", strerror(errno));
        return -1;
    }
    
    if(init == 1) {
        //i.e if init'ing
        if(cfsetspeed(&ti,B115200) == -1){
            printf("Error setting speed: %s",strerror(errno));
            return -1;
        }
    } else if (init == 0) {
        //i.e. if already init'ed
        if(cfsetspeed(&ti,B230400) == -1){
            printf("Error setting speed: %s",strerror(errno));
            return -1;
        }
    }
    
    if(tcsetattr(fd, TCSANOW, &ti) == -1){ //set attributes now
        printf("Error setting (speed) attributes: %s\n", strerror(errno));       //the attributes are set (immediately) again. is this needed?
        return -1;
    }
    
    tcflush(fd, TCIOFLUSH);
    
    return fd;
}

//This returns the STATUS field from the bc response packet as thats the only thing neccesary
//(to determine cmd success or failure) - becuase we dont need to return the whole
//packet you dont need to pass pskey_set an array  pointer to fill!
int pskey_set(int fd, int seqno, uint16_t varid, uint16_t pskey, uint16_t stores, unsigned char* psval, int psval_len, int power_cycled){
    
    int msg_size;
    int pkt_len;
    int clen = 0;
    unsigned char resp[200];
    
    //normal bccmd packet SET-REQ
    unsigned char cmd[200];
    
    //calculate lengths here
    //usuals if lens are short
    pkt_len = 23;
    msg_size = 9; //half BBCMD message size
    
    if((psval_len * 2) > 2){
        //i.e as soon as msg_size infaltes beyond 18 (9) - start dynamically setting values
        msg_size = (16 + (psval_len * 2)) / 2;
        pkt_len = 21 + (psval_len * 2); 		// 5 is the const for the hci hdr len
    }
    
    /* HCI header */
    cmd[0] = HCI_COMMAND_PKT;
    cmd[1] = 0x00;		/* CSR command */
    cmd[2] = 0xfc;		/* MANUFACTURER_SPEC */
    cmd[3] = pkt_len;			/* len - usually 23 */
    /* CSR MSG header */
    cmd[4] = 0xC2;		/* first+last+channel=BCC */
    /* CSR BCC header */
    cmd[5] = 0x02;		/* type = SET-REQ */
    cmd[6] = 0x00;		/* - msB */
    cmd[7] = msg_size;
    cmd[8] = msg_size >> 8;		/* - msB */
    cmd[9] = seqno & 0xFF;/* seq num */
    cmd[10] = (seqno >> 8) & 0xFF;	/* - msB */
    
    cmd[11] = varid;		/* var_id = CSR_CMD_WARM_RESET */
    cmd[12] = varid >> 8;		/* - msB */
    cmd[13] = 0x00;		/* status = STATUS_OK */
    cmd[14] = 0x00;		/* - msB */
    /* normal payload */
    
    cmd[15] = pskey ;      // PSKey to be modified
    cmd[16] = pskey >> 8; //
    
    cmd[17] = psval_len; //len PSValue field (uint16's)
    cmd[18] = psval_len >> 8;
    
    cmd[19] = stores; //stores - 0x0000 default
    cmd[20] = stores >> 8;
    
    memcpy(cmd+21, psval, (psval_len * 2));
    
    clen = pkt_len + 4; //21 + (psval_len * 2);
    
    if(clen < 27) clen = 27;
    
    memset(cmd + 21 + (psval_len * 2), 0, clen - 21 - (psval_len * 2));
    
    //Dont read if there is a possibility GETRESP will not reach the host
    if(power_cycled != 1){
        
        do {
            if(write(fd,cmd,clen) != clen){
                printf("Write failed: %s",strerror(errno));
                return -1;
            }
            
            if(read_hci_event(fd, resp, 100, 0) < 0 ) { // if hci_read_event returns count < 0, thats a fail
                printf("Read failed\n");
                return -1;
            }
        } while(resp[1] != 0xff);
        
        if(resp[12] != 0 || resp[13] != 0){
            printf("STATUS NOT OK - BCCMD ERROR NO: 0x%02x%02x\n", resp[13], resp[12]);
            return -1;
        }
        
    } else {
        //just write - dont read
        if(write(fd,cmd,clen) != clen){
            printf("Write failed: %s",strerror(errno));
            return -1;
        }
        
    }
    return 0;
}