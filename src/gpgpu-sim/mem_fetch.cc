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

#include "mem_fetch.h"
#include "mem_latency_stat.h"
#include "shader.h"
#include "visualizer.h"
#include "gpu-sim.h"

unsigned mem_fetch::sm_next_mf_request_uid=1;
unsigned mem_fetch::pf_next_mf_request_uid=1;

mem_fetch::mem_fetch( const mem_access_t &access, 
                      const warp_inst_t *inst,
                      unsigned ctrl_size, 
                      unsigned wid,
                      unsigned sid, 
                      unsigned tpc, 
                      const class memory_config *config )
{
   m_request_uid = sm_next_mf_request_uid++;
   m_access = access;
   if( inst ) { 
       m_inst = *inst;
       assert( wid == m_inst.warp_id() );
   }
   m_data_size = access.get_size();
   m_ctrl_size = ctrl_size;
   m_sid = sid;
   m_tpc = tpc;
   m_wid = wid;
   config->m_address_mapping.addrdec_tlx(access.get_addr(),&m_raw_addr);
   m_partition_addr = config->m_address_mapping.partition_address(access.get_addr());
   m_type = m_access.is_write()?WRITE_REQUEST:READ_REQUEST;
   m_timestamp = gpu_sim_cycle + gpu_tot_sim_cycle;
   m_timestamp2 = 0;
   m_status = MEM_FETCH_INITIALIZED;
   m_status_change = gpu_sim_cycle + gpu_tot_sim_cycle;
   m_mem_config = config;
   icnt_flit_size = config->icnt_flit_size;
   m_prefetch = false;
   m_mshr_hit = false;
   ma = NULL;
   m_is_tag = false;
   m_dram_cache_process = true;
   if(get_access_type() == DRAM_WRBK_ACC)
      m_write_ready = m_mem_config->m_dram_cache_config.get_line_sz()/m_mem_config->m_L2_config.get_line_sz();
   else
      m_write_ready = 0;
   data_pkt = NULL;
}

mem_fetch::mem_fetch( mem_fetch *mf,int filltype )	{
   m_mem_config = mf->m_mem_config;
   m_request_uid = mf->m_request_uid;
	   mem_access_type type;
	   if(filltype == 0)
	   	type = DRAM_FILL;
	   else if(filltype == 1)
	   	type = DRAM_FILL_AUX;
     else if(filltype == 5)
      type = DRAM_RD_WRBK;
	   else
	    type = DRAM_WRITE;
   ma = new  mem_access_t( type,
                        mf->get_addr(),
                        m_mem_config->m_L2_config.get_line_sz(),
                        (filltype == 5) ? false : true, 
                        mf->get_access_warp_mask(),
                        mf->get_access_byte_mask() );
   m_access = *ma;
   m_inst = mf->m_inst;
   m_data_size = m_mem_config->m_L2_config.get_line_sz();
   m_ctrl_size = mf->m_ctrl_size;
   m_sid = mf->m_sid;
   m_tpc = mf->m_tpc;
   m_wid = mf->m_wid;
   m_mem_config->m_address_mapping.addrdec_tlx(m_access.get_addr(),&m_raw_addr);
   m_partition_addr = m_mem_config->m_address_mapping.partition_address(m_access.get_addr());
   if(filltype == 5)
      m_type = READ_REQUEST;
   else
      m_type = WRITE_REQUEST;
   m_timestamp = gpu_sim_cycle + gpu_tot_sim_cycle;
   m_timestamp2 = 0;
   m_status = MEM_FETCH_INITIALIZED;
   m_status_change = gpu_sim_cycle + gpu_tot_sim_cycle;
   icnt_flit_size = mf->icnt_flit_size;
   m_prefetch = false;
   m_mshr_hit = false;
   m_is_tag = false;
   m_dram_cache_process = true;
   m_write_ready = 0;
   if(filltype == 5)
      data_pkt = mf;
   else
      data_pkt = NULL;
}

mem_fetch::mem_fetch(mem_fetch *mf, new_addr_type addr,bool prefetch)	{
	assert(prefetch);
   m_request_uid = pf_next_mf_request_uid++;
   m_mem_config = mf->m_mem_config;
   ma = new  mem_access_t( GLOBAL_ACC_R,
                        addr,
                        m_mem_config->m_L2_config.get_line_sz(),
                        false, 
                        mf->get_access_warp_mask(),
                         mf->get_access_byte_mask());
   m_access = *ma;
   m_inst = warp_inst_t();
   m_data_size = m_mem_config->m_L2_config.get_line_sz();
   m_ctrl_size = mf->m_ctrl_size;
   m_sid = -1;
   m_tpc = -1;
   m_wid = -1;
   m_mem_config->m_address_mapping.addrdec_tlx(m_access.get_addr(),&m_raw_addr);
   m_partition_addr = m_mem_config->m_address_mapping.partition_address(m_access.get_addr());
   m_type = READ_REQUEST;
   m_timestamp = gpu_sim_cycle + gpu_tot_sim_cycle;
   m_timestamp2 = 0;
   m_status = MEM_FETCH_INITIALIZED;
   m_status_change = gpu_sim_cycle + gpu_tot_sim_cycle;
   icnt_flit_size = mf->icnt_flit_size;
   m_prefetch = prefetch;
   m_mshr_hit = false;
   m_is_tag = false;
   m_dram_cache_process = true;
   m_write_ready = 0;
   data_pkt = NULL;
 }

mem_fetch::mem_fetch( mem_fetch *mf,bool is_tag,bool is_read, bool is_hit) {
    assert(is_tag);
   m_mem_config = mf->m_mem_config;
   m_request_uid = sm_next_mf_request_uid++;
   ma = new  mem_access_t(DRAM_TAG,
                        mf->get_addr(),
                        m_mem_config->m_L2_config.get_line_sz(),
                        !is_read, 
                        mf->get_access_warp_mask(),
                         mf->get_access_byte_mask());
    m_access = *ma;
   m_inst = mf->m_inst;
   m_data_size = m_mem_config->m_L2_config.get_line_sz();
   m_ctrl_size = mf->m_ctrl_size;
   m_sid = mf->m_sid;
   m_tpc = mf->m_tpc;
   m_wid = mf->m_wid;
   m_mem_config->m_address_mapping.addrdec_tlx(m_access.get_addr(),&m_raw_addr);
   m_partition_addr = m_mem_config->m_address_mapping.partition_address(m_access.get_addr());
   m_type = is_read ? READ_REQUEST : WRITE_REQUEST;
   m_timestamp = gpu_sim_cycle + gpu_tot_sim_cycle;
   m_timestamp2 = 0;
   m_status = MEM_FETCH_INITIALIZED;
   m_status_change = gpu_sim_cycle + gpu_tot_sim_cycle;
   icnt_flit_size = mf->icnt_flit_size;
   m_prefetch = false;
   m_mshr_hit = false;
   m_is_tag = is_tag;
   m_is_tag_update = !is_read;
   m_dram_cache_process = is_hit;
   m_write_ready = 0;
   data_pkt = mf;
}

mem_fetch::~mem_fetch()
{
    if(ma != NULL)
        delete ma;
	m_request_uid = this->m_request_uid;
    m_status = MEM_FETCH_DELETED;
}

#define MF_TUP_BEGIN(X) static const char* Status_str[] = {
#define MF_TUP(X) #X
#define MF_TUP_END(X) };
#include "mem_fetch_status.tup"
#undef MF_TUP_BEGIN
#undef MF_TUP
#undef MF_TUP_END

void mem_fetch::print( FILE *fp, bool print_inst ) const
{
    if( this == NULL ) {
        fprintf(fp," <NULL mem_fetch pointer>\n");
        return;
    }
    fprintf(fp,"%llx  mf: uid=%6u, sid%02u:w%02u, part=%u, sub_part=%u ",this, m_request_uid, m_sid, m_wid, m_raw_addr.chip,get_sub_partition_id() );
    m_access.print(fp);
    if( (unsigned)m_status < NUM_MEM_REQ_STAT ) 
       fprintf(fp," status = %s (%llu), Prefetch=%u ", Status_str[m_status], m_status_change, m_prefetch );
    else
       fprintf(fp," status = %u??? (%llu), Prefetch=%u ", m_status, m_status_change, m_prefetch );
    if(m_is_tag)
        fprintf(fp,"\tTAG\t%d",get_type());
    else
        fprintf(fp,"\tDATA\t%d",get_type());
    if( !m_inst.empty() && print_inst ) m_inst.print(fp);
    else fprintf(fp,"\n");
}

void mem_fetch::set_status( enum mem_fetch_status status, unsigned long long cycle ) 
{
	assert(m_status != MEM_FETCH_DELETED);
    m_status = status;
    m_status_change = cycle;
}

bool mem_fetch::isatomic() const
{
   if( m_inst.empty() ) return false;
   return m_inst.isatomic();
}

void mem_fetch::do_atomic()
{
    m_inst.do_atomic( m_access.get_warp_mask() );
}

bool mem_fetch::istexture() const
{
    if( m_inst.empty() ) return false;
    return m_inst.space.get_type() == tex_space;
}

bool mem_fetch::isconst() const
{ 
    if( m_inst.empty() ) return false;
    return (m_inst.space.get_type() == const_space) || (m_inst.space.get_type() == param_space_kernel);
}

/// Returns number of flits traversing interconnect. simt_to_mem specifies the direction
unsigned mem_fetch::get_num_flits(bool simt_to_mem){
	unsigned sz=0;
	// If atomic, write going to memory, or read coming back from memory, size = ctrl + data. Else, only ctrl
	if( isatomic() || (simt_to_mem && get_is_write()) || !(simt_to_mem || get_is_write()) )
		sz = size();
	else
		sz = get_ctrl_size();

	return (sz/icnt_flit_size) + ( (sz % icnt_flit_size)? 1:0);
}

bool mem_fetch::match(mem_fetch *mf,cache_config &config)	{ 
   		new_addr_type block_addr1 = config.block_addr(get_addr());
   	   	new_addr_type block_addr2 = config.block_addr(mf->get_addr());
   	   	if(block_addr1 == block_addr2)
   	   		return true;
   	   	else
   	   		return false;
}


