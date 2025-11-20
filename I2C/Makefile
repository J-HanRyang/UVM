RTL_DIR 	= ./rtl
TB_DIR 		= ./tb
BUILD_DIR 	= ./build

DUT 		= $(RTL_DIR)/I2C_Master.sv\
		  $(RTL_DIR)/I2C_Slave.sv
TEST_BENCH 	= $(TB_DIR)/UVM_I2C.sv
COMP_OPT 	= -full64 -sverilog -timescale=1ns/100ps \
				-ntb_opts uvm-1.2 \
				-debug_access+all -kdb\
				-o $(BUILD_DIR)/simv \
				-l $(BUILD_DIR)/comp.log

TEST_NAME 	= i2c_test
FSDB_FILE 	= $(BUILD_DIR)/wave.fsdb
SIMV 		= $(BUILD_DIR)/simv
SIMV_OPT 	= +UVM_TESTNAME=$(TEST_NAME) \
				-l $(BUILD_DIR)/simv.log \
				+fsdbfile+$(FSDB_FILE)

.PHONY : all vsc simv clean

all: vcs simv

vcs:
	-mkdir build
	vcs $(COMP_OPT) $(DUT) $(TEST_BENCH)

simv:
	$(SIMV) $(SIMV_OPT)

verdi:
	verdi -dbdir $(BUILD_DIR)/simv.daidir -ssf $(FSDB_FILE)

clean:
	rm -rf build csrc *.key *.h
