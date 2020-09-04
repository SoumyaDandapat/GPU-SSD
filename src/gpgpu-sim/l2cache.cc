// Copyright (c) 2009-2011, Tor M. Aamodt
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <list>
#include <set>

#include "../option_parser.h"
#include "mem_fetch.h"
#include "dram.h"
#include "gpu-cache.h"
#include "histogram.h"
#include "l2cache.h"
#include "../statwrapper.h"
#include "../abstract_hardware_model.h"
#include "gpu-sim.h"
#include "shader.h"
#include "mem_latency_stat.h"
#include "l2cache_trace.h"


mem_fetch * partition_mf_allocator::alloc(new_addr_type addr, mem_access_type type, unsigned size, bool wr ) const 
{
    assert( wr );
    mem_access_t access( type, addr, size, wr );
    mem_fetch *mf = new mem_fetch( access, 
                                   NULL,
                                   WRITE_PACKET_SIZE, 
                                   -1, 
                                   -1, 
                                   -1,
                                   m_memory_config );
    return mf;
}

memory_partition_unit::memory_partition_unit( unsigned partition_id, 
                                              const struct memory_config *config,
                                              class memory_stats_t *stats )
: m_id(partition_id), m_config(config), m_stats(stats), m_arbitration_metadata(config) 
{
    char dram_cache_name[32];
    snprintf(dram_cache_name, 32, "DRAM_CACHE_%03d", m_id);
    m_pcieInterface = new PCIeInterface(this);
    m_pcie_allocator = new partition_mf_allocator(config);

    m_dram = new dram_t(m_id,m_config,m_stats,this);
    m_dram_cache = new data_cache(dram_cache_name,m_config->m_dram_cache_config,-1,-1,m_pcieInterface,m_pcie_allocator,IN_PARTITION_DRAM_CACHE_MISS_QUEUE,DRAM_WR_ALLOC_R, DRAM_WRBK_ACC);

    m_sub_partition = new memory_sub_partition*[m_config->m_n_sub_partition_per_memory_channel]; 
    for (unsigned p = 0; p < m_config->m_n_sub_partition_per_memory_channel; p++) {
        unsigned sub_partition_id = m_id * m_config->m_n_sub_partition_per_memory_channel + p; 
        m_sub_partition[p] = new memory_sub_partition(sub_partition_id, this,m_config, stats); 
    }
    num_read_saved = 0;
    num_write_saved = 0;
}

memory_partition_unit::~memory_partition_unit() 
{
    delete m_dram; 
    for (unsigned p = 0; p < m_config->m_n_sub_partition_per_memory_channel; p++) {
        delete m_sub_partition[p]; 
    } 
    delete[] m_sub_partition; 
}

memory_partition_unit::arbitration_metadata::arbitration_metadata(const struct memory_config *config) 
: m_last_borrower(config->m_n_sub_partition_per_memory_channel - 1), 
  m_private_credit(config->m_n_sub_partition_per_memory_channel, 0), 
  m_shared_credit(0) 
{
    // each sub partition get at least 1 credit for forward progress 
    // the rest is shared among with other partitions 
    m_private_credit_limit = 1; 
    m_shared_credit_limit = config->gpgpu_frfcfs_dram_sched_queue_size 
                            + config->gpgpu_dram_return_queue_size 
                            - (config->m_n_sub_partition_per_memory_channel - 1); 
    if (config->gpgpu_frfcfs_dram_sched_queue_size == 0 
        or config->gpgpu_dram_return_queue_size == 0) 
    {
        m_shared_credit_limit = 0; // no limit if either of the queue has no limit in size 
    }
    assert(m_shared_credit_limit >= 0); 
}

bool memory_partition_unit::arbitration_metadata::has_credits(int inner_sub_partition_id,unsigned req_credits) const 
{
        int spid = inner_sub_partition_id; 
    if (m_private_credit[spid] < m_private_credit_limit)
		req_credits--; 
	if (m_shared_credit_limit == 0 || ((m_shared_credit + req_credits) < m_shared_credit_limit)) {
		return true; 
	} else {
		return false; 
	}
}

void memory_partition_unit::arbitration_metadata::borrow_credit(int inner_sub_partition_id) 
{
    int spid = inner_sub_partition_id; 
    if (m_private_credit[spid] < m_private_credit_limit) {
        m_private_credit[spid] += 1; 
    } else if (m_shared_credit_limit == 0 || m_shared_credit < m_shared_credit_limit) {
        m_shared_credit += 1; 
    } else {
        assert(0 && "DRAM arbitration error: Borrowing from depleted credit!"); 
    }
    m_last_borrower = spid; 
}

void memory_partition_unit::arbitration_metadata::return_credit(int inner_sub_partition_id) 
{
    int spid = inner_sub_partition_id; 
    if (m_private_credit[spid] > 0) {
        m_private_credit[spid] -= 1; 
    } else {
        m_shared_credit -= 1; 
    } 
    assert((m_shared_credit >= 0) && "DRAM arbitration error: Returning more than available credits!"); 
}

void memory_partition_unit::arbitration_metadata::print( FILE *fp ) const 
{
    fprintf(fp, "private_credit = "); 
    for (unsigned p = 0; p < m_private_credit.size(); p++) {
        fprintf(fp, "%d ", m_private_credit[p]); 
    }
    fprintf(fp, "(limit = %d)\n", m_private_credit_limit); 
    fprintf(fp, "shared_credit = %d (limit = %d)\n", m_shared_credit, m_shared_credit_limit); 
}

bool memory_partition_unit::busy() const 
{
    bool busy = false; 
    for (unsigned p = 0; p < m_config->m_n_sub_partition_per_memory_channel; p++) {
        if (m_sub_partition[p]->busy()) {
            busy = true; 
        }
    }
    return busy; 
}

void memory_partition_unit::cache_cycle(unsigned cycle) 
{
    for (unsigned p = 0; p < m_config->m_n_sub_partition_per_memory_channel; p++) {
        m_sub_partition[p]->cache_cycle(cycle); 
    }
}

void memory_partition_unit::visualizer_print( gzFile visualizer_file ) const 
{
    m_dram->visualizer_print(visualizer_file);
    for (unsigned p = 0; p < m_config->m_n_sub_partition_per_memory_channel; p++) {
        m_sub_partition[p]->visualizer_print(visualizer_file); 
    }
}

// determine whether a given subpartition can issue to DRAM 
bool memory_partition_unit::can_issue_to_dram(int inner_sub_partition_id,unsigned credit_req) 
{
    int spid = inner_sub_partition_id; 
    bool sub_partition_contention = m_sub_partition[spid]->dram_L2_queue_full(); 
    bool has_dram_resource = m_arbitration_metadata.has_credits(spid,credit_req); 
//    if(gpu_sim_cycle+gpu_tot_sim_cycle > 20172)
//    printf("sub partition %d sub_partition_contention=%c has_dram_resource=%c\n", 
//                    spid, (sub_partition_contention)? 'T':'F', (has_dram_resource)? 'T':'F'); 
	
    return (has_dram_resource && !sub_partition_contention); 
}

int memory_partition_unit::global_sub_partition_id_to_local_id(int global_sub_partition_id) const
{
    return (global_sub_partition_id - m_id * m_config->m_n_sub_partition_per_memory_channel); 
}

void memory_partition_unit::dram_cycle() 
{
    bool turn = false;
    if ( m_dram_cache->access_ready()) { 
	   mem_fetch *mf = m_dram_cache->next_access();
	   if(!mf->is_prefetch())	{
		   unsigned dest_global_spid = mf->get_sub_partition_id(); 
		   int dest_spid = global_sub_partition_id_to_local_id(dest_global_spid); 
		   assert(m_sub_partition[dest_spid]->get_id() == dest_global_spid); 
		   push_dram_latency_queue(mf);
	   }
	   else
		   delete mf;
       turn = true;
    }
    // pop completed memory request from dram and push it to dram-to-L2 queue 
    // of the original sub partition 
    if(!turn)	{
	    mem_fetch* mf_return = m_dram->return_queue_top();
	    if (mf_return) {
            unsigned dest_global_spid = mf_return->get_sub_partition_id(); 
            int dest_spid = global_sub_partition_id_to_local_id(dest_global_spid); 
            assert(m_sub_partition[dest_spid]->get_id() == dest_global_spid); 
            if(mf_return->m_is_tag) {
                if(!mf_return->m_is_tag_update) {
                    if(mf_return->m_dram_cache_process)
                        push_dram_latency_queue(mf_return->data_pkt);
                    else
                        m_arbitration_metadata.return_credit(dest_spid);
                }
                else
                    m_arbitration_metadata.return_credit(dest_spid);
                m_dram->return_queue_pop();
                delete mf_return;
            }
            else    {
            	if(mf_return->get_access_type() != DRAM_RD_WRBK)	{
	                if (!m_sub_partition[dest_spid]->dram_L2_queue_full()) {
	                    if( mf_return->get_access_type() == L1_WRBK_ACC ) {
	                        m_sub_partition[dest_spid]->set_done(mf_return); 
	                        delete mf_return;
	                    } else {
	                        m_sub_partition[dest_spid]->dram_L2_queue_push(mf_return);
	                        mf_return->set_status(IN_PARTITION_DRAM_TO_L2_QUEUE,gpu_sim_cycle+gpu_tot_sim_cycle);
	                        m_arbitration_metadata.return_credit(dest_spid); 
	                        MEMPART_DPRINTF("mem_fetch request %p return from dram to sub partition %d\n", mf_return, dest_spid); 
	                    }
	                    m_dram->return_queue_pop(); 
	                } /*else {
	                    m_dram->return_queue_pop(); 
	                }*/
	            }
	            else	{
	            	mf_return->data_pkt->m_write_ready--;
	            	m_dram->return_queue_pop();
	            	delete mf_return;
	            }
            }
        }
    }
    m_dram_cache->cycle();
    m_dram->cycle(); 
    m_dram->dram_log(SAMPLELOG);   

    if( !m_dram->full() ) {
        // L2->DRAM queue to DRAM latency queue
        // Arbitrate among multiple L2 subpartitions 
        int last_issued_partition = m_arbitration_metadata.last_borrower(); 
        for (unsigned p = 0; p < m_config->m_n_sub_partition_per_memory_channel; p++) {
            int spid = (p + last_issued_partition + 1) % m_config->m_n_sub_partition_per_memory_channel; 
            if (!m_sub_partition[spid]->L2_dram_queue_empty()) {
		mem_fetch *mf = m_sub_partition[spid]->L2_dram_queue_top();
                if(process_dram_cache_request(mf,spid))    {
		    MEMPART_DPRINTF("Issue mem_fetch request %p from sub partition %d to dram\n", mf, spid);
                    m_sub_partition[spid]->L2_dram_queue_pop();
                    break;  // the DRAM should only accept one request per cycle
                }
            }
        }
    }

    // DRAM latency queue
    if( !m_dram_latency_queue.empty() && ( (gpu_sim_cycle+gpu_tot_sim_cycle) >= m_dram_latency_queue.front().ready_cycle ) && !m_dram->full() ) {
        mem_fetch* mf = m_dram_latency_queue.front().req;
        m_dram_latency_queue.pop_front();
        m_dram->push(mf);
    }
}

void memory_partition_unit::set_done( mem_fetch *mf )
{
    unsigned global_spid = mf->get_sub_partition_id(); 
    int spid = global_sub_partition_id_to_local_id(global_spid); 
    assert(m_sub_partition[spid]->get_id() == global_spid); 
    if (mf->get_access_type() == L1_WRBK_ACC || mf->get_access_type() == L2_WRBK_ACC
         || mf->get_access_type() == DRAM_WRITE || mf->get_access_type() == DRAM_FILL_AUX || mf->get_access_type() == DRAM_PKT) {
        m_arbitration_metadata.return_credit(spid); 
        MEMPART_DPRINTF("mem_fetch request %p return from dram to sub partition %d\n", mf, spid); 
    }
    
    if(mf->get_access_type() == DRAM_FILL)	{
		mem_fetch *orig_mf = m_replay_queue[mf];
		m_dram_cache->fill(orig_mf,gpu_sim_cycle+gpu_tot_sim_cycle);
		m_arbitration_metadata.return_credit(spid); 
		m_replay_queue.erase(mf);
    }
    else
		m_sub_partition[spid]->set_done(mf);
    if(!mf->m_dram_cache_process) 
        delete mf;
}

void memory_partition_unit::set_dram_power_stats(unsigned &n_cmd,
                                                 unsigned &n_activity,
                                                 unsigned &n_nop,
                                                 unsigned &n_act,
                                                 unsigned &n_pre,
                                                 unsigned &n_rd,
                                                 unsigned &n_wr,
                                                 unsigned &n_req) const
{
    m_dram->set_dram_power_stats(n_cmd, n_activity, n_nop, n_act, n_pre, n_rd, n_wr, n_req);
}

void memory_partition_unit::print( FILE *fp ) const
{
    fprintf(fp, "Memory Partition %u: \n", m_id); 
    for (unsigned p = 0; p < m_config->m_n_sub_partition_per_memory_channel; p++) {
        m_sub_partition[p]->print(fp); 
    }
    fprintf(fp, "In Dram Latency Queue (total = %zd): \n", m_dram_latency_queue.size()); 
    for (std::list<dram_delay_t>::const_iterator mf_dlq = m_dram_latency_queue.begin(); 
         mf_dlq != m_dram_latency_queue.end(); ++mf_dlq) {
        mem_fetch *mf = mf_dlq->req; 
        fprintf(fp, "Ready @ %llu - ", mf_dlq->ready_cycle); 
        if (mf) 
            mf->print(fp); 
        else 
            fprintf(fp, " <NULL mem_fetch?>\n"); 
    }
    m_dram->print(fp);
    m_dram_cache->display_state(fp);
    
    for (unsigned spid = 0; spid < m_config->m_n_sub_partition_per_memory_channel; spid++) {
    	bool sub_partition_contention = m_sub_partition[spid]->dram_L2_queue_full(); 
    	bool has_dram_resource = m_arbitration_metadata.has_credits(spid,1);  
    	printf("sub partition %d sub_partition_contention=%c has_dram_resource=%c\n", 
                    spid, (sub_partition_contention)? 'T':'F', (has_dram_resource)? 'T':'F'); 
	
    }
}

void memory_partition_unit::push_dram_latency_queue(mem_fetch *mf)	{
	dram_delay_t d;
        d.req = mf;
        d.ready_cycle = gpu_sim_cycle+gpu_tot_sim_cycle + m_config->dram_latency;
        m_dram_latency_queue.push_back(d);
	mf->set_status(IN_PARTITION_DRAM_LATENCY_QUEUE,gpu_sim_cycle+gpu_tot_sim_cycle);
}

bool memory_partition_unit::dram_full()	{
	return m_dram->full();
}


void memory_partition_unit::return_credit(unsigned spid)	{
	m_arbitration_metadata.return_credit(spid);
}

memory_sub_partition::memory_sub_partition( unsigned sub_partition_id, memory_partition_unit *mpu,
                                            const struct memory_config *config,
                                            class memory_stats_t *stats )
{
    m_id = sub_partition_id;
    m_config=config;
    m_stats=stats;
    m_mpu = mpu;
    assert(m_id < m_config->m_n_mem_sub_partition); 

    char L2c_name[32];
    snprintf(L2c_name, 32, "L2_bank_%03d", m_id);
    m_L2interface = new L2interface(this);
    m_mf_allocator = new partition_mf_allocator(config);

    if(!m_config->m_L2_config.disabled())
       m_L2cache = new l2_cache(L2c_name,m_config->m_L2_config,-1,-1,m_L2interface,m_mf_allocator,IN_PARTITION_L2_MISS_QUEUE);

    unsigned int icnt_L2;
    unsigned int L2_dram;
    unsigned int dram_L2;
    unsigned int L2_icnt;
    unsigned int dram_pcie;
    sscanf(m_config->gpgpu_dram_pcie_queue_config,"%u", &dram_pcie);
    sscanf(m_config->gpgpu_L2_queue_config,"%u:%u:%u:%u", &icnt_L2,&L2_dram,&dram_L2,&L2_icnt );
    m_icnt_L2_queue = new fifo_pipeline<mem_fetch>("icnt-to-L2",0,icnt_L2); 
    m_L2_dram_queue = new fifo_pipeline<mem_fetch>("L2-to-dram",0,L2_dram);
    m_dram_L2_queue = new fifo_pipeline<mem_fetch>("dram-to-L2",0,dram_L2);
    m_L2_icnt_queue = new fifo_pipeline<mem_fetch>("L2-to-icnt",0,L2_icnt);
    m_dram_pcie_queue = new fifo_pipeline<mem_fetch>("dram-to-pcie",0,dram_pcie);
    wb_addr=-1;
}

memory_sub_partition::~memory_sub_partition()
{
    delete m_icnt_L2_queue;
    delete m_L2_dram_queue;
    delete m_dram_L2_queue;
    delete m_L2_icnt_queue;
    delete m_L2cache;
    delete m_L2interface;
}

void memory_sub_partition::cache_cycle( unsigned cycle )
{
//	if(gpu_sim_cycle+gpu_tot_sim_cycle > 3500)
 //   printf("%u\t%d\t%d\t%d\t%d\t%d\n",m_id,m_icnt_L2_queue->get_length(),m_L2_dram_queue->get_length(),m_dram_pcie_queue->get_length(),
//			m_L2_icnt_queue->get_length(),m_dram_L2_queue->get_length());
    // L2 fill responses
    if( !m_config->m_L2_config.disabled()) {
       if ( m_L2cache->access_ready() && !m_L2_icnt_queue->full() ) {
           mem_fetch *mf = m_L2cache->next_access();
           if(mf->get_access_type() != L2_WR_ALLOC_R){ // Don't pass write allocate read request back to upper level cache
				mf->set_reply();
				mf->set_status(IN_PARTITION_L2_TO_ICNT_QUEUE,gpu_sim_cycle+gpu_tot_sim_cycle);
                assert(!mf->m_is_tag);
				m_L2_icnt_queue->push(mf);
           }else{
				m_request_tracker.erase(mf);
				delete mf;
           }
       }
    }

    // DRAM to L2 (texture) and icnt (not texture)
    if ( !m_dram_L2_queue->empty() ) {
        mem_fetch *mf = m_dram_L2_queue->top();
        if ( !m_config->m_L2_config.disabled() && m_L2cache->waiting_for_fill(mf) ) {
            if (m_L2cache->fill_port_free()) {
                mf->set_status(IN_PARTITION_L2_FILL_QUEUE,gpu_sim_cycle+gpu_tot_sim_cycle);
                m_L2cache->fill(mf,gpu_sim_cycle+gpu_tot_sim_cycle);
                m_dram_L2_queue->pop();
            }
        } else if ( !m_L2_icnt_queue->full() ) {
            assert(!mf->m_is_tag);
            mf->set_status(IN_PARTITION_L2_TO_ICNT_QUEUE,gpu_sim_cycle+gpu_tot_sim_cycle);
            m_L2_icnt_queue->push(mf);
            m_dram_L2_queue->pop();
        }
    }

    // prior L2 misses inserted into m_L2_dram_queue here
    if( !m_config->m_L2_config.disabled() )
       m_L2cache->cycle();

    // new L2 texture accesses and/or non-texture accesses
    if ( !m_L2_dram_queue->full() && !m_icnt_L2_queue->empty() ) {
        mem_fetch *mf = m_icnt_L2_queue->top();
        if ( !m_config->m_L2_config.disabled() &&
              ( (m_config->m_L2_texure_only && mf->istexture()) || (!m_config->m_L2_texure_only) )
           ) {
            // L2 is enabled and access is for L2
            bool output_full = m_L2_icnt_queue->full(); 
            bool port_free = m_L2cache->data_port_free(); 
            if ( !output_full && port_free ) {
                std::list<cache_event> events;
		enum cache_request_status status = m_L2cache->access(mf->get_addr(),mf,gpu_sim_cycle+gpu_tot_sim_cycle,events);
                bool write_sent = was_write_sent(events);
                bool read_sent = was_read_sent(events);

                if ( status == HIT ) {
		    if( !write_sent ) {
                        // L2 cache replies
                        assert(!read_sent);
                        if( mf->get_access_type() == L1_WRBK_ACC ) {
                            m_request_tracker.erase(mf);
                            delete mf;
                        } else {
                            mf->set_reply();
                            mf->set_status(IN_PARTITION_L2_TO_ICNT_QUEUE,gpu_sim_cycle+gpu_tot_sim_cycle);
                            assert(!mf->m_is_tag);
			    m_L2_icnt_queue->push(mf);
                        }
                        m_icnt_L2_queue->pop();
                    } else {
                        assert(write_sent);
                        m_icnt_L2_queue->pop();
                    }
                } else if ( status != RESERVATION_FAIL ) {
                    // L2 cache accepted request
                    m_icnt_L2_queue->pop();
                } else {
                    assert(!write_sent);
                    assert(!read_sent);
                    // L2 cache lock-up: will try again next cycle
                }
            }
        } else {
	    assert(0);
            // L2 is disabled or non-texture access to texture-only L2
            mf->set_status(IN_PARTITION_L2_TO_DRAM_QUEUE,gpu_sim_cycle+gpu_tot_sim_cycle);
            m_L2_dram_queue->push(mf);
            m_icnt_L2_queue->pop();
        }
    }

    // ROP delay queue
    if( !m_rop.empty() && (cycle >= m_rop.front().ready_cycle) && !m_icnt_L2_queue->full() ) {
        mem_fetch* mf = m_rop.front().req;
        m_rop.pop();
        m_icnt_L2_queue->push(mf);
        mf->set_status(IN_PARTITION_ICNT_TO_L2_QUEUE,gpu_sim_cycle+gpu_tot_sim_cycle);
    }
}

bool memory_sub_partition::full() const
{
    return m_icnt_L2_queue->full();
}

bool memory_sub_partition::L2_dram_queue_empty() const
{
   return m_L2_dram_queue->empty(); 
}

class mem_fetch* memory_sub_partition::L2_dram_queue_top() const
{
   return m_L2_dram_queue->top(); 
}

void memory_sub_partition::L2_dram_queue_pop() 
{
   m_L2_dram_queue->pop(); 
}

bool memory_sub_partition::dram_L2_queue_full() const
{
   return m_dram_L2_queue->full(); 
}

void memory_sub_partition::dram_L2_queue_push( class mem_fetch* mf )
{
   m_dram_L2_queue->push(mf); 
}

void memory_sub_partition::print_cache_stat(unsigned &accesses, unsigned &misses) const
{
    FILE *fp = stdout;
    if( !m_config->m_L2_config.disabled() )
       m_L2cache->print(fp,accesses,misses);
}

void memory_sub_partition::print( FILE *fp ) const
{
    if ( !m_request_tracker.empty() ) {
        fprintf(fp,"Memory Sub Parition %u: pending memory requests:\n", m_id);
        for ( std::set<mem_fetch*>::const_iterator r=m_request_tracker.begin(); r != m_request_tracker.end(); ++r ) {
            mem_fetch *mf = *r;
            if ( mf)
            	mf->print(fp);
            else
                fprintf(fp," <NULL mem_fetch?>\n");
        }
    }
    if( !m_config->m_L2_config.disabled() )
       m_L2cache->display_state(fp);
}

void memory_stats_t::visualizer_print( gzFile visualizer_file )
{
   // gzprintf(visualizer_file, "Ltwowritemiss: %d\n", L2_write_miss);
   // gzprintf(visualizer_file, "Ltwowritehit: %d\n",  L2_write_access-L2_write_miss);
   // gzprintf(visualizer_file, "Ltworeadmiss: %d\n", L2_read_miss);
   // gzprintf(visualizer_file, "Ltworeadhit: %d\n", L2_read_access-L2_read_miss);
   if (num_mfs)
      gzprintf(visualizer_file, "averagemflatency: %lld\n", mf_total_lat/num_mfs);
}

void gpgpu_sim::print_dram_stats(FILE *fout) const
{
	unsigned cmd=0;
	unsigned activity=0;
	unsigned nop=0;
	unsigned act=0;
	unsigned pre=0;
	unsigned rd=0;
	unsigned wr=0;
	unsigned req=0;
	unsigned tot_cmd=0;
	unsigned tot_nop=0;
	unsigned tot_act=0;
	unsigned tot_pre=0;
	unsigned tot_rd=0;
	unsigned tot_wr=0;
	unsigned tot_req=0;

	for (unsigned i=0;i<m_memory_config->m_n_mem;i++){
		m_memory_partition_unit[i]->set_dram_power_stats(cmd,activity,nop,act,pre,rd,wr,req);
		tot_cmd+=cmd;
		tot_nop+=nop;
		tot_act+=act;
		tot_pre+=pre;
		tot_rd+=rd;
		tot_wr+=wr;
		tot_req+=req;
	}
    fprintf(fout,"gpgpu_n_dram_reads = %d\n",tot_rd );
    fprintf(fout,"gpgpu_n_dram_writes = %d\n",tot_wr );
    fprintf(fout,"gpgpu_n_dram_activate = %d\n",tot_act );
    fprintf(fout,"gpgpu_n_dram_commands = %d\n",tot_cmd);
    fprintf(fout,"gpgpu_n_dram_noops = %d\n",tot_nop );
    fprintf(fout,"gpgpu_n_dram_precharges = %d\n",tot_pre );
    fprintf(fout,"gpgpu_n_dram_requests = %d\n",tot_req );
}

unsigned memory_sub_partition::flushL2() 
{ 
    if (!m_config->m_L2_config.disabled()) {
        m_L2cache->flush(); 
    }
    return 0; // L2 is read only in this version
}

bool memory_sub_partition::busy() const 
{
    return !m_request_tracker.empty();
}

void memory_sub_partition::push( mem_fetch* req, unsigned long long cycle ) 
{
    if (req) {
        m_request_tracker.insert(req);
        m_stats->memlatstat_icnt2mem_pop(req);
        if( req->istexture() || 1) {
            m_icnt_L2_queue->push(req);
            req->set_status(IN_PARTITION_ICNT_TO_L2_QUEUE,gpu_sim_cycle+gpu_tot_sim_cycle);
        } else {
            rop_delay_t r;
            r.req = req;
            r.ready_cycle = cycle + m_config->rop_latency;
            m_rop.push(r);
            req->set_status(IN_PARTITION_ROP_DELAY,gpu_sim_cycle+gpu_tot_sim_cycle);
        }
    }
}

mem_fetch* memory_sub_partition::pop() 
{
    mem_fetch* mf = m_L2_icnt_queue->pop();
    m_request_tracker.erase(mf);
    if ( mf && mf->isatomic() )
        mf->do_atomic();
    if( mf && (mf->get_access_type() == L2_WRBK_ACC || mf->get_access_type() == L1_WRBK_ACC) ) {
        delete mf;
        mf = NULL;
    } 
    if(mf != NULL)  {
        assert(!mf->m_is_tag);
    	assert(mf->get_access_type() != DRAM_FILL_AUX);
    }
    return mf;
}

mem_fetch* memory_sub_partition::top() 
{
    mem_fetch *mf = m_L2_icnt_queue->top();
    if( mf && (mf->get_access_type() == L2_WRBK_ACC || mf->get_access_type() == L1_WRBK_ACC) ) {
        assert(!mf->m_is_tag);
        m_L2_icnt_queue->pop();
        m_request_tracker.erase(mf);
        delete mf;
        mf = NULL;
    }
    if(mf)  {
        assert(!mf->m_is_tag);
    } 
    return mf;
}

void memory_sub_partition::set_done( mem_fetch *mf )
{
    m_request_tracker.erase(mf);
}

void memory_sub_partition::accumulate_L2cache_stats(class cache_stats &l2_stats) const {
    if (!m_config->m_L2_config.disabled()) {
        l2_stats += m_L2cache->get_stats();
    }
}

void memory_sub_partition::get_L2cache_sub_stats(struct cache_sub_stats &css) const{
    if (!m_config->m_L2_config.disabled()) {
        m_L2cache->get_sub_stats(css);
    }
}

void memory_sub_partition::visualizer_print( gzFile visualizer_file )
{
    // TODO: Add visualizer stats for L2 cache 
}

bool memory_partition_unit::process_dram_cache_request(mem_fetch *mf,unsigned spid) {
    bool port_free = m_dram_cache->data_port_free();
    unsigned line_size = m_config->m_dram_cache_config.get_line_sz();
    unsigned num_l2_lines = line_size / m_config->m_L2_config.get_line_sz();
    //printf("%u\t%u\n",line_size,num_l2_lines);
    unsigned credit_req;
    if(m_config->m_dram_cache_config.m_redram
        && (mf->get_access_type() == LOCAL_ACC_R || mf->get_access_type() == LOCAL_ACC_W || mf->get_access_type() == CONST_ACC_R))
        credit_req = 1;
    else
        credit_req = num_l2_lines+3;
    if (port_free && can_issue_to_dram(spid,credit_req)) {
		std::list<cache_event> events;
        assert(!mf->m_is_tag);
        enum cache_request_status status;
        if(m_config->m_dram_cache_config.m_redram
           && (mf->get_access_type() == LOCAL_ACC_R || 
			   mf->get_access_type() == LOCAL_ACC_W || 
			   (mf->get_addr() >= 0x807df000 && mf->get_addr() <= 0x80bff000)))	{
            status = HIT;
			unsigned idx;
	    	enum cache_request_status auxstatus = m_dram_cache->m_tag_array->probe(mf->get_addr(),idx);
	    	if(auxstatus != HIT && auxstatus != HIT_RESERVED)
				num_read_saved++;
		}
    	else 
        	status = m_dram_cache->access(mf->get_addr(),mf,gpu_sim_cycle+gpu_tot_sim_cycle,events);
        bool write_sent = was_write_sent(events);
        bool read_sent = was_read_sent(events);
        
        if ( status == HIT ) {
		    if( !write_sent )
                assert(!read_sent);
            if(m_config->m_dram_cache_config.m_redram
        && (mf->get_access_type() == LOCAL_ACC_R || mf->get_access_type() == LOCAL_ACC_W ||
			   (mf->get_addr() >= 0x807df000 && mf->get_addr() <= 0x80bff000) ))	{
                push_dram_latency_queue(mf);
            }
            else    {
                if(mf->is_write() && m_config->m_dram_cache_config.m_write_policy != WRITE_BACK)
                    mf = new mem_fetch(mf,3);
                mem_fetch *tag_read = new mem_fetch(mf,true,true,true);
                push_dram_latency_queue(tag_read);
            }
            //push_dram_latency_queue(mf);
			//m_arbitration_metadata.borrow_credit(spid);
            m_arbitration_metadata.borrow_credit(spid);
            //m_arbitration_metadata.borrow_credit(spid);
	    	return true;
        } else if ( status != RESERVATION_FAIL ) {
			if(!mf->is_write())     {
				m_arbitration_metadata.borrow_credit(spid);// for replay
			    if(mf->get_status() == IN_PARTITION_DRAM_CACHE_MISS_QUEUE)  {
					for(unsigned i = 0; i < num_l2_lines; i++)
						m_arbitration_metadata.borrow_credit(spid);
                    m_arbitration_metadata.borrow_credit(spid);// for tag update
                	if(write_sent)	{
			    		for(unsigned i = 0; i < num_l2_lines; i++)	{
			    			mem_fetch *wb_acc = new mem_fetch(mf,5);
			    			push_dram_latency_queue(wb_acc);
			    		}
			    	}
			    }
			}
            mem_fetch *tag_read = new mem_fetch(mf,true,true,false);
            push_dram_latency_queue(tag_read);
            m_arbitration_metadata.borrow_credit(spid);
            return true;
        } else {
            assert(!write_sent);
            assert(!read_sent);
		    // DRAM cache lock-up: will try again next cycle
            return false;
        }
    }
    return false;
}

void memory_partition_unit::accumulate_dram_cache_stats(class cache_stats &dram_stats,struct cache_sub_stats &css) const {
	dram_stats += m_dram_cache->get_stats();
	m_dram_cache->get_sub_stats(css);
	std::cout<<"Reads Saved:"<<num_read_saved<<std::endl;
    std::cout<<"Writes Saved:"<<num_write_saved<<std::endl;
	std::cout<<"Num. Invalid Blocks:"<<m_dram_cache->m_tag_array->get_num_invalid()<<std::endl;
}

bool memory_sub_partition::miss_queue_empty()  {
    if(!m_config->m_pinned)
        return m_dram_pcie_queue->empty();
    else
        return m_icnt_L2_queue->empty();
}

unsigned memory_sub_partition::miss_queue_size()  {
    assert(!m_config->m_pinned);
        return m_dram_pcie_queue->get_length();
}

mem_fetch *memory_sub_partition::top_miss()    {
    if(!m_config->m_pinned)	{
    	mem_fetch *mf = m_dram_pcie_queue->top();
    	if(mf->get_access_type() == DRAM_WRBK_ACC)	{
    		if(mf->m_write_ready == 0)
        		return mf;
        	else
        		return NULL;
        }
        else
        	return mf;
    }
    else
        return m_icnt_L2_queue->top();
}

mem_fetch *memory_sub_partition::pop_miss()    {
    if(!m_config->m_pinned)	{
		mem_fetch *mf = m_dram_pcie_queue->top();
        //assert(!mf->m_is_tag);
        m_dram_pcie_queue->pop();
		return mf;
	}
    else
        return m_icnt_L2_queue->pop();
}

void memory_sub_partition::push_response(mem_fetch *mf)    {
    if(!m_config->m_pinned)    {
		if(m_mpu->m_dram_cache->waiting_for_fill(mf))	{
			assert(!mf->is_write());
			mem_fetch *fill_req = new mem_fetch(mf,0);
			m_mpu->m_replay_queue[fill_req] = mf;
			m_mpu->push_dram_latency_queue(fill_req);
			unsigned line_size = m_config->m_dram_cache_config.get_line_sz();
    		unsigned num_l2_lines = line_size / m_config->m_L2_config.get_line_sz();
    		for(unsigned i = 0; i < num_l2_lines-1; i++)	{
    			mem_fetch *aux_fill_req = new mem_fetch(mf,1);
    			m_mpu->push_dram_latency_queue(aux_fill_req);
    		}
            mem_fetch *tag_fill = new mem_fetch(mf,true,false,false);
			m_mpu->push_dram_latency_queue(tag_fill);
		}
		else	{
			if(mf->get_access_type() != L1_WRBK_ACC && mf->get_access_type() != L2_WRBK_ACC && mf->get_access_type() != DRAM_WRBK_ACC)	{
				mf->set_status(IN_PARTITION_L2_TO_ICNT_QUEUE,gpu_sim_cycle+gpu_tot_sim_cycle);
				mf->set_reply();
                //assert(!mf->m_is_tag);
				m_L2_icnt_queue->push(mf);
			}
			else	{
				if(mf->get_access_type() == L1_WRBK_ACC)
					set_done(mf);		
				delete mf;
			}
		}
    }
    else    {
        mf->set_status(IN_PARTITION_L2_TO_ICNT_QUEUE,gpu_sim_cycle+gpu_tot_sim_cycle);
		mf->set_reply();
        assert(!mf->m_is_tag);
        m_L2_icnt_queue->push(mf); 
    }
}

bool memory_sub_partition::is_pcie_resp_full() {
    if(!m_config->m_pinned)
        return m_mpu->dram_full() || !m_mpu->m_dram_cache->fill_port_free() || m_L2_icnt_queue->full();
    else
        return m_L2_icnt_queue->full();
}

enum cache_request_status memory_sub_partition::reserve_prefetch(mem_fetch *mf)	{
	std::list<cache_event> events;
	unsigned global_spid = mf->get_sub_partition_id(); 
    int spid = m_mpu->global_sub_partition_id_to_local_id(global_spid); 
    unsigned line_size = m_config->m_dram_cache_config.get_line_sz();
    unsigned num_l2_lines = line_size / m_config->m_L2_config.get_line_sz();
    if(m_mpu->can_issue_to_dram(spid,num_l2_lines))	{
		enum cache_request_status status = m_mpu->m_dram_cache->access(mf->get_addr(),mf,gpu_sim_cycle+gpu_tot_sim_cycle,events);
		if(status == MISS && !mf->is_mshr_hit())	{
			for(unsigned i = 0; i < num_l2_lines; i++)
				m_mpu->m_arbitration_metadata.borrow_credit(spid);
            m_mpu->m_arbitration_metadata.borrow_credit(spid);// for tag update
			mf->set_status(RESERVATION_DONE,gpu_sim_cycle+gpu_tot_sim_cycle);
		}
		else if(status == HIT)
			mf->set_status(RESERVATION_DONE,gpu_sim_cycle+gpu_tot_sim_cycle);
		return status;	        
	}
	else
		return RESERVATION_FAIL;
}

mem_fetch *memory_sub_partition::get_coalesed_packet(mem_fetch *mf)	{
    mem_fetch *pkt = m_dram_pcie_queue->search_and_remove(mf,m_config->m_dram_cache_config);
	if(pkt == NULL)	{
		pkt = m_mpu->m_dram_cache->search_and_remove(mf,m_config->m_dram_cache_config);
	}
	return pkt;
}
