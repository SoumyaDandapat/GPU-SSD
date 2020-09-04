// Copyright (c) 2009-2011, Tor M. Aamodt, Ivan Sham, Ali Bakhoda, 
// George L. Yuan, Wilson W.L. Fung
// The University of British Columbia
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
// Redistributions in binary form must reproduce the above copyright notice, this
// list of conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.
// Neither the name of The University of British Columbia nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef PCIE_H
#define PCIE_H

#include "../gpgpu-sim/l2cache.h"
#include "../gpgpu-sim/dram.h"
#include <vector>

enum RequestStatus	{
	IDLE,
	WAIT,
	BUSY
};

class CPURequest {
public:
    mem_fetch *mf;
    mem_fetch **mf_list;
    bool *m_valid;
    RequestStatus m_status;
    unsigned latency;
    unsigned m_num_rd_reqs;
    unsigned m_num_wr_reqs;
};

using namespace std;
class PCIe	{
private:
	unsigned m_num_sub_part;
	unsigned m_last_sub_part;
	memory_sub_partition **m_msp;
	CPURequest *m_cpu_req;
    class gpgpu_sim *m_gpu;
    unsigned wait_count;
    unsigned coaleased;
public:
	PCIe(gpgpu_sim *gpu,memory_sub_partition **msp,unsigned num_sub_part);
	~PCIe();
	bool arbitrate_partition_for_miss();
	void cycle();
	void icnt_cycle();
	void print();
    void generate_prefetch_packets();
	void reserve_blocks();
};

#endif /*PCIE_H*/
