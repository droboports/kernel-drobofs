#include <stdio.h>
#include <stdlib.h>
#include "mv_eth_proc.h"

extern char **environ; /* all environments */

static unsigned int port = 0, q = 0, weight = 0, status = 0, mac[6] = {0,};
static unsigned int policy =0, command = 0, packet = 0;
static unsigned int value = 0;
static unsigned int inport, outport, dip, sip, da[6] = {0, }, sa[6] = {0, };
static unsigned int db_type = 0;

void show_usage(int badarg)
{
        fprintf(stderr,
	"Usage: 										\n"
	" egigatool -h										\n"
	"   Display this help.									\n"
	"											\n"
	" egigatool <option> <Port No>								\n"	
	"   OPTIONS:										\n"
        "   -rxcoal <port> <usec>               Set RX interrupt coalescing value               \n"
        "   -txcoal <port> <usec>               Set TX interrupt coalescing value               \n"
	"   -ejp    <port> <0|1>                Set EJP mode: 0 - Disable, 1 - Enable           \n"
	" egigatool <option> 									\n"
	"   OPTIONS:										\n"
        "   -txdone <quota>                     Set threshold to start tx_done operations       \n"
	"   -txen <deep>                        Set deep of lookup for TX enable race.          \n"
        "                                       0 - means disable.                              \n"
	"   -skb <0|1>                          SKB reuse support: 0 - disabled, 1 - enabled    \n"
	"   -srq <0..8> <bpdu|arp|tcp|udp>      Set Rx q number for different packet types.     \n"
	"                                       q number 8 means no special treatment.		\n"
	"		                        q number 8 for Arp means drop.			\n"
	"   -sq  <0..8> <%%2x:%%2x:%%2x:%%2x:%%2x:%%2x>  				        \n"
	"                                       Set Rx q number for a group of Multicast MACs,	\n"
	"                                       Or set Tx q number for a unicast address. 	\n"
	"                                       q number 8 indicate delete entry.		\n"
        "   -srp                 <WRR|FIXED>    Set the Rx Policy to WRR or Fixed 		\n"
	"   -srqw <0..7> <quota>                Set quota(hex) for RXQ (packet resolution)  	\n"
	"   -stp  <0..7> <quota> <WRR|FIXED>    Set the Tx Policy to WRR or Fixed               \n"
	"                                       And set quota(hex) for TXQ (256 Bytes resolution)\n"
	"											\n"
        " egigatool -fprs <inp> <outp> <DIP> <SIP> <DA> <SA>					\n"
        "                                       Set NFP rule for IPv4 routing			\n"
        "                                       where DA and SA are MAC addresses xx:xx:xx:xx:xx:xx\n"
	" egigatool -fprd <DIP> <SIP>           Delete NFP Rule for IPv4 routing		\n"
	" egigatool -fp_dis                     Disable Network Fast Processing			\n"
	" egigatool -fp_en                      Enable Network Fast Processing			\n"
	" egigatool -fp_st                      Display NFP Status (enabled/disabled)		\n"
	" egigatool -fp_print <DB>              Print Rule Database, where DB is routing or nat \n"
	"											\n"
	" egigatool -St <option> <port>							        \n"
	"   Display different status information of the port through the kernel printk.		\n"
	"   OPTIONS:										\n"
	"   p                                   Display General port information.		\n"
        "   mac                                 Display MAC addresses information               \n"
	"   q   <0..7>                          Display specific q information.			\n"
	"   rxp                                 Display Rx Policy information.			\n"
	"   txp                                 Display Tx Policy information.			\n"
	"   cntrs                               Display the HW MIB counters			\n"
	"   regs                                Display a dump of the giga registers		\n"
	"   statis                              Display port statistics information. 		\n"
	"   netdev                              Display net_device status information.          \n"
	"   nfp                                 Display port NFP statistics                     \n\n"	
        );
        exit(badarg);
}

static void parse_pt(char *src)
{
        if (!strcmp(src, "bpdu"))
        	packet = PT_BPDU;
	else if(!strcmp(src, "arp"))
        	packet = PT_ARP;
	else if(!strcmp(src, "tcp"))
        	packet = PT_TCP;
	else if(!strcmp(src, "udp"))
        	packet = PT_UDP;
	else {
		fprintf(stderr, "Illegall packet type, packet type should be bpdu/arp/tcp/udp. \n");	
                exit(-1);
        }
        return;
}

static void parse_db_name(char *src)
{
	if (!strcmp(src, "routing"))
		db_type = DB_ROUTING;
	else if (!strcmp(src, "nat"))
		db_type = DB_NAT;
	else {
		fprintf(stderr, "Illegall DB name, DB name should be routing/nat. \n");
		exit(-1);
	}
	return;
}

static void parse_port(char *src)
{
	int count;

        count = sscanf(src, "%x",&port);

        if ((port > MAX_PORT) ||(count != 1))  {
		fprintf(stderr, "Illegal port number, max port supported is %d.\n",MAX_PORT);	
                exit(-1);
        }
        return;
}


static void parse_q(char *src)
{
	int count;

        count = sscanf(src, "%x",&q);

        if ((q >  MAX_Q + 1) || (count != 1)) {
		fprintf(stderr, "Illegal q number, max q supported is %d.\n",MAX_Q);	
                exit(-1);
        }
        return;
}

static void parse_policy(char *src)
{

        if (!strcmp(src, "WRR"))
        	policy = WRR;
	else if (!strcmp(src, "FIXED"))
        	policy = FIXED;
	else {
		fprintf(stderr, "Illegall policy, policy can be WRR or Fixed.\n");	
                exit(-1);
        }
        return;
}

static void parse_status(char *src)
{
    	if (!strcmp(src, "p")) {
      		status = STS_PORT;
      	}
        else if (!strcmp(src, "mac")) {
                status = STS_PORT_MAC;
        }
     	else if(!strcmp(src, "q")) {
       		status = STS_PORT_Q;
     	}
   	else if(!strcmp(src, "rxp")) {
          	status = STS_PORT_RXP;
      	}
     	else if(!strcmp(src, "txp")) {
           	status = STS_PORT_TXP;
      	}
        else if(!strcmp(src, "cntrs")) {
                status = STS_PORT_MIB;
        }
        else if(!strcmp(src, "regs")) {
                status = STS_PORT_REGS;
        }
      	else if(!strcmp(src, "statis")) {
             	status = STS_PORT_STATIS;
        }
        else if(!strcmp(src, "nfp")) {
                status = STS_PORT_NFP_STATS;
        }
	else if(!strcmp(src, "netdev")) {
                status = STS_NETDEV;
        }
        else {
                fprintf(stderr, "Illegall ststus %d.\n");
                exit(-1);
        }
        return;
}
static void parse_dec_val(char *src, unsigned int* val_ptr)
{
    int i, count;

    count = sscanf(src, "%d", val_ptr);
    if(count != 1) {
        fprintf(stderr, "Illegall value - should be decimal.\n");
        exit(-1);
    }
    return;
}

static void parse_hex_val(char *src, unsigned int* hex_val_ptr)
{
    int i, count;

    count = sscanf(src, "%x", hex_val_ptr);
    if(count != 1) {
        fprintf(stderr, "Illegall value - should be hex.\n");
        exit(-1);
    }
    return;
}

static int parse_mac(char *src, unsigned int macaddr[])
{
        int count;
        int i;
        int buf[6];

        count = sscanf(src, "%2x:%2x:%2x:%2x:%2x:%2x",
                &buf[0], &buf[1], &buf[2], &buf[3], &buf[4], &buf[5]);

        if (count != MAC_ADDR_LEN) {
		fprintf(stderr, "Illegall MAC address, MAC adrdess should be %%2x:%%2x:%%2x:%%2x:%%2x:%%2x.\n");
                exit(-1);
        }

        for (i = 0; i < count; i++) {
                macaddr[i] = buf[i];
        }
        return 0;
}

static int parse_ip(char *src, unsigned int* ip)
{
    int count, i;
    int buf[4];

    count = sscanf(src, "%d.%d.%d.%d",
                &buf[0], &buf[1], &buf[2], &buf[3]);

    if (count != 4) {
        fprintf(stderr, "Illegall IP address (should be %%d.%%d.%%d.%%d)\n");
        exit(-1);
    }
    *ip = (((buf[0] & 0xFF) << 24) | ((buf[1] & 0xFF) << 16) |
           ((buf[2] & 0xFF) << 8) | ((buf[3] & 0xFF) << 0));
    return 0;
}

static void parse_cmdline(int argc, char **argp)
{
	unsigned int i = 1;

	if(argc < 2) {
		show_usage(1);
	}

	if (!strcmp(argp[i], "-h")) {
		show_usage(0);
	}
	else if (!strcmp(argp[i], "-srq")) {
		command = COM_SRQ;
		i++;	
		if(argc != 5)
			show_usage(1); 
		parse_q(argp[i++]);
		parse_pt(argp[i++]);
	}
	else if (!strcmp(argp[i], "-sq")) {
		command = COM_SQ;
		i++;
		if(argc != 6)
			show_usage(1); 
		parse_q(argp[i++]);
		parse_mac(argp[i++], mac);
	}
	else if (!strcmp(argp[i], "-srp")) {
		command = COM_SRP;
		i++;
		if(argc != 4)
			show_usage(1); 
		parse_policy(argp[i++]);
	}
	else if (!strcmp(argp[i], "-srqw")) {
		command = COM_SRQW;
		i++;
		if(argc != 5)
			show_usage(1); 
		parse_q(argp[i++]);
		parse_hex_val(argp[i++], &weight);
	}
    	else if (!strcmp(argp[i], "-stp")) {
        	command = COM_STP;
        	i++;
        	if(argc != 6)
            		show_usage(1);
        	parse_q(argp[i++]);
		parse_hex_val(argp[i++], &weight);
		parse_policy(argp[i++]);
    	}
    	else if (!strcmp(argp[i], "-fprs")) {
        	command = COM_IP_RULE_SET;
        	i++;
        	if(argc != 8)
            		show_usage(1);
		parse_dec_val(argp[i++], &inport);
        	parse_dec_val(argp[i++], &outport);
		parse_ip(argp[i++], &dip);
		parse_ip(argp[i++], &sip);
        	parse_mac(argp[i++], da);
		parse_mac(argp[i++], sa);
		return;
    	}
	else if (!strcmp(argp[i], "-fprd")) {
		command = COM_IP_RULE_DEL;
		i++;
		if(argc != 4)
			show_usage(1);
		parse_ip(argp[i++], &dip);
		parse_ip(argp[i++], &sip);
		return;
	}
	else if (!strcmp(argp[i], "-fp_dis")) {
		command = COM_FP_DISABLE;
		if(argc != 2)
			show_usage(1);
		return;
	}
	else if (!strcmp(argp[i], "-fp_en")) {
		command = COM_FP_ENABLE;
		if(argc != 2)
			show_usage(1);
		return;
	}
	else if (!strcmp(argp[i], "-fp_st")) {
		command = COM_FP_STATUS;
		if(argc != 2)
			show_usage(1);
		return;
	}
	else if (!strcmp(argp[i], "-fp_print")) {
		command = COM_FP_PRINT;
		i++;
		if (argc != 3)
			show_usage(1);
		parse_db_name(argp[i++]);
		return;
	}
    	else if (!strcmp(argp[i], "-txdone")) {
        	command = COM_TXDONE_Q;
        	i++;
		if(argc != 3)
			show_usage(1);
        	parse_dec_val(argp[i++], &value);
		return;
    	}
    	else if (!strcmp(argp[i], "-txen")) {
                command = COM_TX_EN;
                i++;
		if(argc != 3)
			show_usage(1);
                parse_dec_val(argp[i++], &value);
                return;
        }
        else if (!strcmp(argp[i], "-skb")) {
                command = COM_SKB_REUSE;
                i++;
		if(argc != 3)
			show_usage(1);
                parse_dec_val(argp[i++], &value);
                return;
        }
	else if (!strcmp(argp[i], "-rxcoal")) {
        	command = COM_RX_COAL;
        	i++;
		if(argc != 4)
			show_usage(1);
        	parse_port(argp[i++]);
        	parse_dec_val(argp[i++], &value);
		return;
    	}
    	else if (!strcmp(argp[i], "-txcoal")) {
        	command = COM_TX_COAL;
        	i++;
		if(argc != 4)
			show_usage(1);
        	parse_port(argp[i++]);
        	parse_dec_val(argp[i++], &value);
		return;
    	}
        else if (!strcmp(argp[i], "-ejp")) {
                command = COM_EJP_MODE;
                i++;
                if(argc != 4)
                        show_usage(1);
                parse_port(argp[i++]);
                parse_dec_val(argp[i++], &value);
                return;
	}
    	else if (!strcmp(argp[i], "-St")) {
        	command = COM_STS;
        	i++;
		if(argc < 4)
			show_usage(1);
		parse_status(argp[i++]);
		if( status == STS_PORT_Q ) {
            		if(argc != 5)
                		show_usage(1);
               		parse_q(argp[i++]);
	    	}
            	else if(argc != 4)
                	show_usage(1);
    	}
#ifdef CONFIG_MV_ETH_HEADER
        else if (!strcmp(argp[i], "-head")) {
		handle_head_cmd(argc,argp);
		exit(0);
        }	
#endif
	else {
		show_usage(i++);
	}
	parse_port(argp[i++]);
}

static int procit(void)
{
  	FILE *mvethproc;
  	mvethproc = fopen(FILE_PATH FILE_NAME, "w");
  	if (!mvethproc) {
    		printf ("Eror opening file %s/%s\n",FILE_PATH,FILE_NAME);
    		exit(-1);
  	}

	switch (command) {
		case COM_TXDONE_Q:
		case COM_TX_EN:
		case COM_SKB_REUSE:
			fprintf (mvethproc, ETH_CMD_STRING, ETH_PRINTF_LIST);
			break;
		case COM_RX_COAL:
		case COM_TX_COAL:
		case COM_EJP_MODE:
			fprintf (mvethproc, PORT_CMD_STRING, PORT_PRINTF_LIST);
			break;
		case COM_IP_RULE_SET:
			fprintf(mvethproc, IP_RULE_STRING, IP_RULE_PRINT_LIST);
			break;
		case COM_IP_RULE_DEL:
			fprintf(mvethproc, IP_RULE_DEL_STRING, IP_RULE_DEL_PRINT_LIST);
			break;
		case COM_FP_DISABLE:
		case COM_FP_ENABLE:
		case COM_FP_STATUS:
			fprintf(mvethproc, FP_EN_DIS_STRING, FP_EN_DIS_PRINT_LIST);
			break;
		case COM_FP_PRINT:
			fprintf(mvethproc, FP_DB_PRINT_STRING, FP_DB_PRINT_PRINT_LIST);
			break;
		default:
			fprintf (mvethproc, PROC_STRING, PROC_PRINT_LIST);
			break;
	}

	fclose (mvethproc);
	return 0;
}

int main(int argc, char **argp, char **envp)
{
        parse_cmdline(argc, argp);
        return procit();
}

