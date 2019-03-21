#define IP  0x0021
#define LCP 0xc021
#define NCP 0x8021

typedef struct {
	uint32_t address : 8;
	uint32_t control : 8;
	uint32_t protocol : 16;
} pppHeader;

typedef struct {
	uint32_t code : 8;
	uint32_t indentifier : 8;
	uint32_t length : 16;
	uint8_t * data;
} cpFrame; 

typedef struct {
	uint32_t version : 4;
	uint32_t ihl : 4;
	uint32_t dscp : 6; 
	uint32_t ecn : 1; 
	uint32_t length : 16;

	uint32_t identyfication : 16;
	uint32_t flags : 3;
	uint32_t fragmentOffset : 8;

	uint32_t ttl : 8;
	uint32_t protocol : 8;
	uint32_t checksum : 16;

	uint32_t sourceAddr;

	uint32_t destinationAddr;
} ipv4Header;

typedef struct {
	uint32_t type : 8;
	uint32_t code : 8;
	uint32_t checksum : 16;

	uint32_t identifier : 16;
	uint32_t sequenceNumber : 16;

	uint8_t * payload;
} icmpHeader;