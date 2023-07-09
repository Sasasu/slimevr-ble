#define DMP_START_ADDRESS ((unsigned short)0x1000)
#define DMP_LOAD_START 0x90

#define REG_MEM_START_ADDR_B0 0x7C
#define REG_MEM_R_W_B0 0x7D
#define REG_MEM_BANK_SEL_B0 0x7E
#define REG_REG_BANK_SEL_B0 0x7L
#define REG_PRGM_START_ADDRH_B2 0x50
#define REG_PRGM_START_ADDRL_B2 0x51

#define MAX_MEM_READ 16
#define MAX_MEM_WRITE 16

#define ACC_SCALE (30 * 16 + 0)
#define ACC_SCALE2 (79 * 16 + 4)

// mounting matrix: all 16-bit
#define CPASS_MTX_00 (23 * 16)      // Compass mount matrix and scale
#define CPASS_MTX_01 (23 * 16 + 4)  // Compass mount matrix and scale
#define CPASS_MTX_02 (23 * 16 + 8)  // Compass mount matrix and scale
#define CPASS_MTX_10 (23 * 16 + 12) // Compass mount matrix and scale
#define CPASS_MTX_11 (24 * 16)      // Compass mount matrix and scale
#define CPASS_MTX_12 (24 * 16 + 4)  // Compass mount matrix and scale
#define CPASS_MTX_20 (24 * 16 + 8)  // Compass mount matrix and scale
#define CPASS_MTX_21 (24 * 16 + 12) // Compass mount matrix and scale
#define CPASS_MTX_22 (25 * 16)      // Compass mount matrix and scale

#define DATA_OUT_CTL1 (4 * 16)
#define DATA_OUT_CTL2 (4 * 16 + 2)
#define DATA_INTR_CTL (4 * 16 + 12)
#define DATA_RDY_STATUS (8 * 16 + 10)

#define ODR_QUAT9 (10 * 16 + 8) // ODR_QUAT9 Register for 9-axis quaternion ODR
