CLK_PERIOD ?= 2
TEST ?= /home/users/shiyuan6/afs-home/ee271_proj/rasterizer-fall-2022-main/test-vectors/vec_271_01_sv.dat
export CLOCK_PERIOD = $(CLK_PERIOD)
export HLS_TEST_VECTOR = $(TEST)

build/Rasterizer.v1/rtl.v: scripts/build.tcl src/bounding_box_hls.h src/jitter_hls.h src/rasterizer_testbench.cpp src/rasterizer_top.cpp src/rast_types_hls.h src/sample_test_hls.h src/test_iterator_hls.h
	rm -f build.ccs
	rm -rf build
	catapult -shell -f scripts/build.tcl

test/Rasterizer.v1/scverify/Verify_orig_cxx_osci.mk: 
	catapult -shell -f scripts/test.tcl

.PHONY: c_test rtl_test clean run_dc

c_test: test/Rasterizer.v1/scverify/Verify_orig_cxx_osci.mk
	catapult -shell -f scripts/c_test.tcl

rtl_test: build/Rasterizer.v1/rtl.v
	catapult -shell -f scripts/rtl_test.tcl

run_dc: build/Rasterizer.v1/rtl.v
	cd synth && dc_shell -f rast_dc.tcl

## Unit Tests
bbox_test/BoundingBoxGenerator.v1/scverify/Verify_orig_cxx_osci.mk:
	catapult -shell -f scripts/bbox_test.tcl

bbox_c_test: bbox_test/BoundingBoxGenerator.v1/scverify/Verify_orig_cxx_osci.mk
	catapult -shell -f scripts/bbox_c_test.tcl

clean:
	rm -f build.ccs
	rm -rf build
	rm -f test.ccs
	rm -rf test