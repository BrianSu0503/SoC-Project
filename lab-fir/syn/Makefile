sim:
	vcs -R +v2k -full64 testfixture_a.v ../rtl/FIR.v -debug_acc +nospecify
	#vcs -R +v2k -full64 testfixture_b.v ../rtl/FIR.v -debug_acc +nospecify
	#vcs -R +v2k -full64 testfixture_c.v ../rtl/FIR.v -debug_acc +nospecify
gatesim:
	vcs -R +v2k -full64 testfixture_a.v ../syn/netlist/FIR_syn.v ../bram/bram32.v -v /home/raid7_2/course/cvsd/CBDK_IC_Contest_v2.5/Verilog/tsmc13_neg.v +define+SDF -debug_acc +neg_tchk
	#vcs -R +v2k -full64 testfixture_b.v ../syn/netlist/FIR_syn.v -v /home/raid7_2/course/cvsd/CBDK_IC_Contest_v2.5/Verilog/tsmc13_neg.v +define+SDF -debug_acc +neg_tchk
	#vcs -R +v2k -full64 testfixture_c.v ../syn/netlist/FIR_syn.v -v /home/raid7_2/course/cvsd/CBDK_IC_Contest_v2.5/Verilog/tsmc13_neg.v +define+SDF -debug_acc +neg_tchk
