#ifndef _RMW_H
#define _RMW_H

#include "Transaction.h"
#include "MemorySystem.h"
#include <memory>
#include <stdint.h>
#include <ostream>
#include <cstring>
#include <map>
#include "Callback.h"
#include "SimulatorObject.h"
#include "SystemConfiguration.h"

using namespace std;

namespace LPDDRSim {
//typedef Transaction& t_ptr;
//class MemoryController;
class LPMemorySystemTop;
class Rmw:public SimulatorObject {
    ofstream DDRSim_log;
    ofstream state_log;

    enum RMW_State {
        QUE_WAITING,
        MERGE_READ,
        SEND_READY
    };
    ofstream buf_log;
    struct conf_state {
        uint64_t task;
        unsigned ad_conf_cnt;
        conf_state() {
            ad_conf_cnt = 0;
            task = 0;
        }
    };
    struct cmd_state {
        uint64_t task;
        uint64_t rmwTimeAdded;
        RMW_State rmwState;
        cmd_state() {
            task = 0;
            rmwState = QUE_WAITING;
            rmwTimeAdded = 0;
        }
    };

    public:
    Rmw(LPMemorySystemTop *_top, unsigned id, unsigned log_id, ostream &DDRSim_log_, string LogPath);
    virtual ~Rmw();
    bool addTransaction(Transaction * trans);
    void update_cresp();
    bool addData(uint32_t *data, uint32_t channel, uint64_t task);
    void RmwInitOutputFiles();
    void cmd_set_conflict(Transaction *trans);
    void cmd_release_conflict(Transaction *trans);
    bool address_conf(Transaction *t, Transaction *cmd);
    void update();
    void sch_que();
    void arb_node();
    void send_wdata();
    bool hasPendingWork() const;
    bool flushWriteMergeBuffer();
    unsigned rmw_cmd_cnt;
    inline bool full() {return (RMW_QUE_DEPTH != 0 && rmw_cmd_cnt >= RMW_QUE_DEPTH);}
    void func_check();
    void update_state();
//    void trans_state_clr(Transaction *trans);
    uint64_t pre_req_time;          // time point for last cmd
    uint64_t pre_req_data_time;    // time point for last wdata 
    std::vector<uint64_t> WdataToSend;
    std::vector<uint32_t> WdataChannel;
    std::vector<Transaction *> RmwQue;

    struct WdataOrderEntry {
        uint64_t task;
        unsigned remaining_beats;
        WdataOrderEntry(uint64_t task_, unsigned remaining_beats_)
                : task(task_), remaining_beats(remaining_beats_) {}
    };
    std::vector<WdataOrderEntry> wdata_order_queue;
    void track_write_command(uint64_t task, unsigned beats);
    void check_write_data(uint64_t task);

    string rmw_log;
//    string log_path;
//    unsigned channel;
//    uint64_t channel_ohot;
    

    uint64_t pre_cresp_time;
    vector <uint64_t> RmwCmdResp;
    vector <uint8_t> RmwCmdRespCh;
    void gen_cresp(uint64_t task);
    bool cmd_response(uint64_t task,uint64_t address, uint8_t ch);
    
    unsigned GetRmwQsize();
    void check_cnt();
    void register_write(uint64_t address, uint32_t data);
    void statistics();

    //statistics
    unsigned totalReads; 
    unsigned totalBypassReads; 
    unsigned totalWrites; 
    unsigned totalBypassWrites; 
    unsigned totalFullWrites; 
    unsigned totalMaskWrites; 
    unsigned totalTransactions;
    unsigned totalWriteMergeInput;
    unsigned totalWriteMergePair;
    unsigned totalWriteMergeUnpairedToRmw;
    unsigned totalWriteMergeUnpairedDirect;
    unsigned totalWriteMergeBufferFull;
    unsigned pre_reads; 
    unsigned pre_bypass_reads; 
    unsigned pre_bypass_writes; 
    unsigned pre_writes; 
    unsigned pre_full_writes; 
    unsigned pre_mask_writes; 
    unsigned pre_totals; 
    unsigned preWriteMergeInput;
    unsigned preWriteMergePair;
    unsigned preWriteMergeUnpairedToRmw;
    unsigned preWriteMergeUnpairedDirect;
    unsigned preWriteMergeBufferFull;

    vector<uint32_t> rmw_que_cnt;
    
    uint64_t start_cycle;
    uint64_t end_cycle;

    // #region debug-point D:rmw-occupancy
    uint64_t dbg_cycles = 0;
    uint64_t dbg_full_cycles = 0;
    uint64_t dbg_no_ready_cycles = 0;
    uint64_t dbg_dmc_rejects = 0;
    uint64_t dbg_dispatches = 0;
    unsigned dbg_max_queue = 0;
    uint64_t dbg_reject_mc_occupancy_sum = 0;
    uint64_t dbg_reject_mc_wb_sum = 0;
    uint64_t dbg_reject_mc_mask_read_sum = 0;
    uint64_t dbg_reject_rmw_ready_sum = 0;
    uint64_t dbg_reject_rmw_waiting_sum = 0;
    unsigned dbg_reject_mc_occupancy_max = 0;
    unsigned dbg_reject_mc_wb_max = 0;
    // #endregion

    private:
//    std::vector<Transaction *> RmwQue;
    std::vector<conf_state *> RmwConfCnt;
    std::vector<cmd_state *> RmwCmdState;
    LPMemorySystemTop *top;
    unsigned channel;
    unsigned log_channel;
    uint64_t channel_ohot;
//    ostream &DDRSim_log;
    RMW_State rmw_state;
//    unsigned same_bank_cnt;
//    unsigned no_cmd_sch_cnt;
//    unsigned no_cmd_sch_th;
//    unsigned bytes_per_col;
    string log_path;
    unsigned rcmd_cnt;
    unsigned wcmd_cnt;

    struct WriteMergeEntry {
        Transaction *first_trans;
        Transaction *second_trans;
        unsigned first_data_ready_cnt;
        unsigned second_data_ready_cnt;
        bool has_second;
        uint64_t enqueue_time;
        uint8_t upstream_channel;
        WriteMergeEntry();
    };

    struct PendingWriteMergeResp {
        uint64_t task;
        uint8_t channel;
        uint64_t wait_data_task;
        PendingWriteMergeResp(uint64_t task_, uint8_t channel_, uint64_t wait_data_task_ = 0xffffffffffffffffull);
    };

    struct WriteMergeDataRemap {
        uint64_t src_task;
        uint64_t dst_task;
        unsigned remaining_beats;
        WriteMergeDataRemap(uint64_t src_task_, uint64_t dst_task_, unsigned remaining_beats_);
    };

    struct BypassedMergedWrite {
        uint64_t first_task;
        uint64_t merged_task;
        unsigned first_remaining;
        unsigned second_remaining;
        bool dispatched;
        BypassedMergedWrite(uint64_t first_task_, uint64_t merged_task_, unsigned first_remaining_, unsigned second_remaining_)
                : first_task(first_task_), merged_task(merged_task_), first_remaining(first_remaining_),
                  second_remaining(second_remaining_), dispatched(false) {}
    };

    std::vector<WriteMergeEntry> write_merge_buffer;
    std::vector<PendingWriteMergeResp> pending_write_merge_resps;
    std::vector<WriteMergeDataRemap> write_merge_data_remaps;
    std::map<uint64_t, unsigned> pending_write_data_cnt;
    std::map<uint64_t, unsigned> fast_bypass_write_data_cnt;
    std::map<uint64_t, BypassedMergedWrite> bypassed_merged_writes;
    uint64_t pre_write_merge_resp_time;

    bool is_write_merge_candidate(const Transaction *trans) const;
    bool can_merge_write_pair(const Transaction *first, const Transaction *second) const;
    Transaction *build_merged_write_transaction(Transaction *first, Transaction *second, uint64_t merged_task, bool mask_wcmd);
    bool handle_write_merge_transaction(Transaction *trans);
    bool dispatch_write_merge_entry(size_t index, bool force_mask_wcmd);
    bool pump_write_merge_buffer();
    bool flush_one_write_merge_entry();
    bool remap_write_merge_data(uint32_t *data, uint64_t task);
    bool is_write_merge_data_task(uint64_t task) const;
    bool add_write_merge_data(uint32_t *data, uint64_t task);
    void update_write_merge_resp();
    bool write_merge_response(uint64_t task, uint8_t channel);
    void rebuild_conflict_state();
    bool is_unpaired_write_merge_timeout(Transaction *trans, cmd_state *state);

    std::map<uint64_t, uint64_t> write_merge_first_resp_task;
    std::map<uint64_t, uint8_t> write_merge_first_resp_channel;
    std::map<uint64_t, bool> write_merge_unpaired_direct_accounted;

//    public:
//    unsigned push_cnt;


//    arb_cmd LastArbCmd;
//    std::vector <arb_cmd *> ArbCmd;
};
}
#endif
