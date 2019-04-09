#define IP  0x2100
#define LCP 0x21C0
#define NCP 0x2180

typedef struct {
	uint32_t address : 8;
	uint32_t control : 8;
	uint32_t protocol : 16;
	uint8_t data[0];
} pppHeader;

typedef struct {
	uint32_t code : 8;
	uint32_t identifier : 8;
	uint32_t length : 16;
	uint8_t data[0];
} cpFrame; 

typedef struct {
	uint32_t type : 8;
	uint32_t length : 8;
	uint8_t data[0];
} cpOption;

typedef struct {
	uint32_t ihl : 4;
	uint32_t version : 4;
	uint32_t ecn : 2; 
	uint32_t dscp : 6; 
	uint32_t length : 16;

	uint32_t identyfication : 16;
	uint32_t fragmentOffset1 : 5;
	uint32_t flags : 3;
	uint32_t fragmentOffset2 : 8;

	uint32_t ttl : 8;
	uint32_t protocol : 8;
	uint32_t checksum : 16;

	uint32_t sourceAddr;

	uint32_t destinationAddr;

	uint8_t payload[0];
} ipHeader;

typedef struct {
	uint32_t type : 8;
	uint32_t code : 8;
	uint32_t checksum : 16;

	uint32_t identifier : 16;
	uint32_t sequenceNumber : 16;

	uint8_t payload[0];
} icmpHeader;
