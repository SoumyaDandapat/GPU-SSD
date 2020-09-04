// Copyright (c) 2009-2011, Tor M. Aamodt, Wilson W.L. Fung, Ali Bakhoda,
// Ivan Sham, George L. Yuan,
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

#include "pcie.h"
#include "gpu-sim.h"

#define DEC2ZERO(x) x = (x)? (x-1) : 0;

PCIe::~PCIe()	{
	delete m_cpu_req;
}

void PCIe::print()	{
	printf("PCIe:\t");
	if(m_cpu_req->m_valid)
		m_cpu_req->mf->print(stdout,false);
	std::cout<<"PCIe Request---"<<m_cpu_req->m_num_rd_reqs<<"\t"<<m_cpu_req->m_num_wr_reqs<<"\t"<<coaleased<<std::endl;
}

PCIe::PCIe(gpgpu_sim *gpu,memory_sub_partition **msp,unsigned num_sub_part)	{
	m_num_sub_part = num_sub_part;
	m_last_sub_part = m_num_sub_part - 1;
	m_msp = msp;
    m_gpu = gpu;
    m_cpu_req = new CPURequest();
    m_cpu_req->m_num_rd_reqs = 0;
    m_cpu_req->m_num_wr_reqs = 0;
    m_cpu_req->latency = 0;
    m_cpu_req->mf = NULL;
    m_cpu_req->mf_list = new mem_fetch*[m_num_sub_part];
    m_cpu_req->m_valid = new bool[m_num_sub_part];
	for(unsigned i = 0; i < m_num_sub_part; i++)	{
	    m_cpu_req->mf_list[i] = NULL;
	    m_cpu_req->m_valid[i] = false;
	}
    m_cpu_req->m_status = IDLE;
    wait_count = 0;
    coaleased = 0;
}

void PCIe::cycle()	{
    DEC2ZERO(m_cpu_req->latency);
}

void PCIe::icnt_cycle()	{
	bool status = true,has_pending = true;
	switch(m_cpu_req->m_status)	{
    case BUSY:
		if(m_cpu_req->latency == 0)	{
			for(unsigned i = 0; i < m_num_sub_part; i++)	{
				if(m_cpu_req->m_valid[i])	{
					if(m_cpu_req->mf_list[i] != NULL && ((m_cpu_req->mf_list[i]->data_pkt != NULL && 
						m_cpu_req->mf_list[i]->data_pkt->m_write_ready == 0) || m_cpu_req->mf_list[i]->data_pkt == NULL))	{
						unsigned sub_part_id = m_cpu_req->mf_list[i]->get_sub_partition_id();
						if(!m_msp[sub_part_id]->is_pcie_resp_full())	{
							m_cpu_req->mf_list[i]->set_status(IN_CPU_RET,gpu_sim_cycle + gpu_tot_sim_cycle); 
							//m_cpu_req->mf_list[i]->print(stdout,false);
							assert(!m_cpu_req->mf_list[i]->m_is_tag);
							m_msp[sub_part_id]->push_response(m_cpu_req->mf_list[i]);
							m_cpu_req->mf_list[i] =  NULL;
							m_cpu_req->m_valid[i] = false;
						}
					}
					else
						m_cpu_req->m_valid[i] = false;
				}
			}
			bool status = false;
			for(unsigned i = 0; i < m_num_sub_part; i++)	{
				if(m_cpu_req->m_valid[i])	{
					status = true;
				}
			}
			if(status)
				has_pending = true;
			else	
				has_pending = false;
	 	}
	 	if(!has_pending)	{
	 		m_cpu_req->m_status = IDLE;
	 		m_cpu_req->mf = NULL;
	 		for(unsigned i = 0; i < m_num_sub_part; i++)	{
				m_cpu_req->mf_list[i] =  NULL;
				m_cpu_req->m_valid[i] = false;
			}
		}
		break;


	case IDLE:
		if(arbitrate_partition_for_miss())
			m_cpu_req->m_status = WAIT;
		break;


	case WAIT:
		reserve_blocks();
		wait_count++;
		//for(unsigned i = 0; i < m_num_sub_part; i++)
		//	m_cpu_req->mf_list[i]->print(stdout,false);
		for(unsigned i = 0; i < m_num_sub_part; i++)
			if(m_cpu_req->m_valid[i] == false)
					status = false;
		if(status || wait_count == 2)	{
			for(unsigned i = 0; i < m_num_sub_part; i++)
				if(m_cpu_req->mf_list[i] != NULL)	{
					//std::cout<<"Prefetching...\n";
					m_cpu_req->mf_list[i]->set_status(IN_CPU,gpu_sim_cycle + gpu_tot_sim_cycle);
				}
			if(m_cpu_req->mf->is_write())	{
				m_cpu_req->latency = 100;
				m_cpu_req->m_num_wr_reqs++;
			}
			else	{
				m_cpu_req->m_num_rd_reqs++;
				unsigned line_size = m_msp[0]->m_config->m_dram_cache_config.get_line_sz();
				unsigned num_l2_lines = line_size / 128;
				m_cpu_req->latency = 100 + num_l2_lines * 5;
			}
			m_cpu_req->m_status = BUSY;
			wait_count = 0;
		}
		break;
	}
}

bool PCIe::arbitrate_partition_for_miss()	{
	unsigned next_part_rr = (m_last_sub_part + 1) % m_num_sub_part;
	unsigned size = m_msp[next_part_rr]->miss_queue_size();
	unsigned selected_part = next_part_rr;
	if(size == 0)	{
		m_last_sub_part = next_part_rr;
		return false;
	}
	if(selected_part != m_num_sub_part)   {
        mem_fetch *mf = m_msp[selected_part]->pop_miss();
        assert(mf != NULL);
		assert(!mf->is_prefetch());
		/*if(mf->get_mem_config()->m_dram_cache_config.m_prefetch && !mf->is_write())	{
			m_cpu_req->mf = mf;
 			assert(!mf->is_prefetch());
			generate_prefetch_packets();
			assert(!mf->is_prefetch());
		}
		else	{*/
			for(unsigned i = 0; i < m_num_sub_part; i++)	{
				m_cpu_req->mf_list[i] = NULL;
				m_cpu_req->m_valid[i] = true;
			}
			m_cpu_req->mf = mf;
 			m_cpu_req->mf_list[0] = mf;
			m_cpu_req->mf_list[0]->set_status(IN_CPU,gpu_sim_cycle + gpu_tot_sim_cycle); 
			m_cpu_req->m_valid[0] = true;
		//}
		assert(!mf->is_prefetch());
		return true;
    }
    return false;
}





void PCIe::reserve_blocks()  {
	//std::cout<<"Reserving...\n";
					
	for(unsigned i = 0; i < m_num_sub_part; i++)	{
    	if(m_cpu_req->m_valid[i])	
    		continue;
        unsigned spid = m_cpu_req->mf_list[i]->get_sub_partition_id();
        enum cache_request_status status = m_msp[spid]->reserve_prefetch(m_cpu_req->mf_list[i]);
        if(status == HIT)	{
			delete m_cpu_req->mf_list[i];
			m_cpu_req->mf_list[i] = NULL;
			m_cpu_req->m_valid[i] = true;
		}
		else if(status != RESERVATION_FAIL)	{
			if(status == MISS && m_cpu_req->mf_list[i]->is_mshr_hit())	{
				// try coalesing existing requests
			//	printf("After");m_cpu_req->mf_list[i]->print(stdout,false);
				coaleased++;
				mem_fetch *mf_del = m_cpu_req->mf_list[i];
				m_cpu_req->mf_list[i] = m_msp[spid]->get_coalesed_packet(m_cpu_req->mf_list[i]);
				if(m_cpu_req->mf_list[i] != NULL)
					assert(!m_cpu_req->mf_list[i]->is_prefetch());
				delete mf_del;
			}
			m_cpu_req->m_valid[i] = true;
		}
    }
}


//#define GPUDMM 1


void PCIe::generate_prefetch_packets()	{
	mem_fetch *mf = m_cpu_req->mf;
	new_addr_type addr = mf->get_addr();
	addrdec_t base_addr = mf->get_tlx_addr();
#ifdef GPUDMM	
	unsigned col = base_addr.col;
#else
	unsigned demand_spid = mf->get_sub_partition_id();
#endif
	for(unsigned i = 0; i < m_num_sub_part; i++)	{
		unsigned jump_addr = ((addr & 0xFFFF8FFF) | (i << 12));
		assert(!mf->is_prefetch());
		mem_fetch *pref_pkt = new mem_fetch(mf,jump_addr,true);
		assert(!mf->is_prefetch());
		addrdec_t dram_addr = pref_pkt->get_tlx_addr();
#ifdef GPUDMM	
		if(dram_addr.col == base_addr.col)	{
#else
		if(demand_spid == i)	{
#endif
			m_cpu_req->mf_list[i] = mf;
			m_cpu_req->m_valid[i] = true;
			delete pref_pkt;
		}
		else	{
			m_cpu_req->mf_list[i] = pref_pkt;
		}
		assert(base_addr.row == dram_addr.row);
#ifdef GPUDMM	
		assert(base_addr.bk == dram_addr.bk);
		assert(dram_addr.chip == base_addr.chip);
#else
		//assert(dram_addr.col == base_addr.col);
#endif
	}
}
