#include "RMW.h"
#include "LPMemorySystemTop.h"
//#include "MemorySystem.h"
#include <assert.h>
#include <iomanip>
#include <algorithm>
#include <sstream>
using namespace std;

namespace LPDDRSim {
//#define PROTECT_SUB(a) a = (a > 0) ? (a - 1) : 0;

Rmw::WriteMergeEntry::WriteMergeEntry()
        : first_trans(NULL), second_trans(NULL), first_data_ready_cnt(0),
          second_data_ready_cnt(0), has_second(false), enqueue_time(0), upstream_channel(0) {}

Rmw::PendingWriteMergeResp::PendingWriteMergeResp(uint64_t task_, uint8_t channel_, uint64_t wait_data_task_)
        : task(task_), channel(channel_), wait_data_task(wait_data_task_) {}

Rmw::WriteMergeDataRemap::WriteMergeDataRemap(uint64_t src_task_, uint64_t dst_task_, unsigned remaining_beats_)
        : src_task(src_task_), dst_task(dst_task_), remaining_beats(remaining_beats_) {}

Rmw::Rmw(LPMemorySystemTop *_top, unsigned id, unsigned log_id, ostream &DDRSim_log_, string LogPath) :
        top(_top), 
        channel(id), 
        log_channel(log_id),
        log_path(LogPath) {
    channel_ohot = (log_channel < 64) ? (1ull << log_channel) : 0xffffffffffffffffull;
    rmw_cmd_cnt = 0;
//    bytes_per_col = JEDEC_DATA_BUS_BITS / 8;
//    push_cnt = 0;
//    log_path = top->log_path;
    WdataToSend.clear();
    WdataChannel.clear();
    wdata_order_queue.clear();
    pre_req_time = 0xFFFFFFFFFFFFFFFF;
    pre_req_data_time = 0xFFFFFFFFFFFFFFFF;
    pre_cresp_time = 0xFFFFFFFFFFFFFFFF;
    start_cycle = 0;
    end_cycle = 0;
    rcmd_cnt = 0;
    wcmd_cnt = 0;
    totalReads = 0; 
    totalBypassReads = 0; 
    totalWrites = 0; 
    totalBypassWrites = 0; 
    totalFullWrites = 0; 
    totalMaskWrites = 0; 
    totalTransactions = 0; 
    totalWriteMergeInput = 0;
    totalWriteMergePair = 0;
    totalWriteMergeUnpairedToRmw = 0;
    totalWriteMergeUnpairedDirect = 0;
    totalWriteMergeBufferFull = 0;
    pre_reads = 0; 
    pre_bypass_reads = 0; 
    pre_bypass_writes = 0; 
    pre_bypass_writes = 0; 
    pre_full_writes = 0; 
    pre_mask_writes = 0; 
    pre_totals = 0; 
    preWriteMergeInput = 0;
    preWriteMergePair = 0;
    preWriteMergeUnpairedToRmw = 0;
    preWriteMergeUnpairedDirect = 0;
    preWriteMergeBufferFull = 0;
    pre_write_merge_resp_time = 0;
    for (uint32_t index = 0; index < RMW_QUE_DEPTH+1; index++) {
        rmw_que_cnt.push_back(0);
    }
    RmwInitOutputFiles();

    }

Rmw::~Rmw() {
    // #region debug-point D:rmw-occupancy
    std::ostringstream dbg_payload;
    dbg_payload << "{\"sessionId\":\"v200-trace-low-bandwidth\",\"runId\":\"pre\",\"hypothesisId\":\"H1-H3\",\"location\":\"Rmw::~Rmw\",\"msg\":\"[DEBUG] rmw and mc summary\",\"data\":{\"channel\":"
        << log_channel << ",\"cycles\":" << dbg_cycles << ",\"fullCycles\":" << dbg_full_cycles
        << ",\"noReadyCycles\":" << dbg_no_ready_cycles << ",\"dmcRejects\":" << dbg_dmc_rejects
        << ",\"dispatches\":" << dbg_dispatches << ",\"maxQueue\":" << dbg_max_queue
        << ",\"rejectMcOccupancySum\":" << dbg_reject_mc_occupancy_sum
        << ",\"rejectMcWbSum\":" << dbg_reject_mc_wb_sum
        << ",\"rejectMcMaskReadSum\":" << dbg_reject_mc_mask_read_sum
        << ",\"rejectRmwReadySum\":" << dbg_reject_rmw_ready_sum
        << ",\"rejectRmwWaitingSum\":" << dbg_reject_rmw_waiting_sum
        << ",\"rejectMcOccupancyMax\":" << dbg_reject_mc_occupancy_max
        << ",\"rejectMcWbMax\":" << dbg_reject_mc_wb_max << "}}";
    std::string dbg_json = dbg_payload.str();
    size_t dbg_quote = 0;
    while ((dbg_quote = dbg_json.find('"', dbg_quote)) != std::string::npos) {
        dbg_json.insert(dbg_quote, 1, '\\');
        dbg_quote += 2;
    }
    std::string dbg_command = "curl.exe -s -X POST http://127.0.0.1:7777/event -H \"Content-Type: application/json\" -d \"" + dbg_json + "\" > NUL";
    std::system(dbg_command.c_str());
    // #endregion
}

//void WriteBuff::rcmd_push_wcmd(Transaction * t) {
//    for (auto &w : Wbuff) {
//        if (t->bankIndex == w->bankIndex && t->row == w->row && t->col == w->col) {
//            push_cnt ++;
//            if (GrpMode.grp_mode6) { // priority is 1
//                w->pri = 1;
//            } else { // high priority
//                w->timeout = true;
//                wr_timeout_cnt ++;
//            }
//            if (DEBUG_BUS) {
//                PRINTN(setw(10)<<now()<<" -- ADD_PUSH :: task="<<t->task<<hex<<" address="<<t->address<<dec
//                        <<" rank="<<t->rank<<" bank="<<t->bankIndex<<" row="<<t->row<<" w_task="<<w->task<<endl);
//            }
//        }
//    }
//}

void Rmw::RmwInitOutputFiles() {
    if ((DEBUG_BUS || DEBUG_STATE || DEBUG_RMW_STATE) && (RMW_ENABLE || WCMD_MERGE_EN)) {
//        dmc_log = log_path + "/lpddr_sim" + std::to_string(channel) + ".log";
        rmw_log = log_path + "/lpddr_sim" + "_rmw" + std::to_string(log_channel) + ".log";
        DDRSim_log.open(rmw_log.c_str(),ios_base::out | ios_base::trunc);
        if (!DDRSim_log) {
            ERROR("Cannot open "<<rmw_log);
            assert(0);
        }
    }

    if (STATE_LOG && (RMW_ENABLE || WCMD_MERGE_EN)) {
        string st_log = log_path + "/lpddr_state" + "_rmw" + std::to_string(log_channel) + ".log";
        state_log.open(st_log.c_str(),ios_base::out | ios_base::trunc);
        if (!state_log) {
             ERROR("Cannot open "<<st_log);
             assert(0);
        }
    }

}



void Rmw::cmd_set_conflict(Transaction * t) {
    conf_state *c = new conf_state;
    c->task = t->task;

    for (auto &cmd : RmwQue) {
        if (cmd->task == t->task) continue;
        if (t->transactionType == DATA_READ && cmd->transactionType == DATA_READ) continue;
        bool col_conflict = address_conf(t, cmd);
        if (t->channel==cmd->channel && t->bankIndex == cmd->bankIndex && t->row == cmd->row && col_conflict) {
            c->ad_conf_cnt ++;
        }
    }
    RmwConfCnt.push_back(c);
}


void Rmw::cmd_release_conflict(Transaction *trans) {
    unsigned size = RmwQue.size();
    unsigned trans_rmwque_index = 0;
    
    for (unsigned i = 0; i < size; i ++) {
        if (trans->task == RmwQue[i]->task) {
            trans_rmwque_index = i;
            break;
        }
    }

    //index chk
    if (trans_rmwque_index >= size) {
        ERROR(setw(10)<<now()<<" -- Impossible index in Rmw Queue, index="<<trans_rmwque_index<<", task="<<trans->task);
        assert(0);
    }

//    for (unsigned i = 0; i < size; i ++) {
    for (unsigned i = trans_rmwque_index; i < size; i ++) {
        if (trans->task == RmwQue[i]->task) continue;
        if (trans->transactionType==DATA_READ && RmwQue[i]->transactionType==DATA_READ) continue;
        bool col_conflict = address_conf(trans, RmwQue[i]);
        if (trans->channel == RmwQue[i]->channel && trans->bankIndex == RmwQue[i]->bankIndex && trans->row == RmwQue[i]->row && col_conflict) {
            if (RmwConfCnt[i]->ad_conf_cnt > 0) {
                RmwConfCnt[i]->ad_conf_cnt--;
            }
        }
    }

}

bool Rmw::address_conf(Transaction *t, Transaction *cmd) {
        
        // data size cal
        unsigned t_data_size = (t->burst_length + 1) * DMC_DATA_BUS_BITS / 8;
        unsigned cmd_data_size = (cmd->burst_length + 1) * DMC_DATA_BUS_BITS / 8;
        // wrap or inc
//        bool t_wrap = ((t->address % t_data_size) == 0) ? false : true;
//        bool cmd_wrap = ((cmd->address % cmd_data_size) == 0) ? false : true;
        bool t_wrap = t->wrap_cmd;
        bool cmd_wrap = cmd->wrap_cmd;
//        if (t_wrap) {
//            DEBUG(now()<<"wrap cmd, task="<<t->task);
//        }
//        if (cmd_wrap) {
//            DEBUG(now()<<"wrap cmd in RMWQUE, task="<<cmd->task);
//        }
        // addr of trans && cmd
        unsigned t_start_addr_col = t_wrap ? ((t->addr_col / t_data_size) * t_data_size) : t->addr_col;
        unsigned t_end_addr_col = t_wrap ? ((t->addr_col / t_data_size + 1) * t_data_size) : t->addr_col + t_data_size;
        unsigned cmd_start_addr_col = cmd_wrap ? ((cmd->addr_col / cmd_data_size) * cmd_data_size) : cmd->addr_col;
        unsigned cmd_end_addr_col = cmd_wrap ? ((cmd->addr_col / cmd_data_size + 1) * cmd_data_size) : cmd->addr_col + cmd_data_size;
        // addr used for address conflict
        unsigned t_addr_left = 0;
        unsigned t_addr_right = 0;
        unsigned cmd_addr_left = 0;
        unsigned cmd_addr_right = 0;
        bool     ret = false;
         
//        if ((t_start_addr_col % RMW_CONF_SIZE == 0) && (t_end_addr_col % RMW_CONF_SIZE == 0)){       // address aligned with RMW_CONF_SIZE
        if (t_end_addr_col % RMW_CONF_SIZE == 0){       // address aligned with RMW_CONF_SIZE
            t_addr_left = t_start_addr_col / RMW_CONF_SIZE;
            t_addr_right = t_end_addr_col / RMW_CONF_SIZE;
        } else {       // address not aligned with RMW_CONF_SIZE 
            t_addr_left = t_start_addr_col / RMW_CONF_SIZE;
            t_addr_right = (t_end_addr_col / RMW_CONF_SIZE) + 1;
        }
        
//        if ((cmd_start_addr_col % RMW_CONF_SIZE == 0) && (cmd_end_addr_col % RMW_CONF_SIZE == 0)){       // address aligned with RMW_CONF_SIZE
        if (cmd_end_addr_col % RMW_CONF_SIZE == 0){       // address aligned with RMW_CONF_SIZE
            cmd_addr_left = cmd_start_addr_col / RMW_CONF_SIZE;
            cmd_addr_right = cmd_end_addr_col / RMW_CONF_SIZE;
        } else {       // address not aligned with RMW_CONF_SIZE 
            cmd_addr_left = cmd_start_addr_col / RMW_CONF_SIZE;
            cmd_addr_right = (cmd_end_addr_col / RMW_CONF_SIZE) + 1;
        }
        
        if ((t_addr_left >= cmd_addr_left && t_addr_left < cmd_addr_right) ||
                (t_addr_right > cmd_addr_left && t_addr_right <= cmd_addr_right)) {
//            DEBUG(now()<<" address conflict, transaction task="<<t->task<<" t_addr_left="<<t_addr_left<<" t_addr_right"<<t_addr_right
//                    <<" cmd in RMWQUE, task="<<cmd->task<<" cmd_addr_left"<<cmd_addr_left<<" cmd_addr_right"<<cmd_addr_right);
            ret = true;
        }
        return ret;

    
}

//void WriteBuff::trans_state_clr(Transaction * trans) {
//    trans->timeout = false;
//    trans->has_active = false;
//}

bool Rmw::is_write_merge_candidate(const Transaction *trans) const {
    if (trans == NULL) return false;
    if (!WCMD_MERGE_EN) return false;
    if (trans->transactionType != DATA_WRITE) return false;
    if (!trans->mergeflag) return false;
    if (trans->mask_wcmd) return false;
    if (trans->ecc_flag) return false;
    return ((trans->burst_length + 1) * DMC_DATA_BUS_BITS / 8) == 128;
}

bool Rmw::can_merge_write_pair(const Transaction *first, const Transaction *second) const {
    if (first == NULL || second == NULL) return false;
    if (!is_write_merge_candidate(first) || !is_write_merge_candidate(second)) return false;
    if (first->channel != second->channel) return false;
    uint64_t first_addr = first->address;
    uint64_t second_addr = second->address;
    uint64_t low_addr = first_addr < second_addr ? first_addr : second_addr;
    uint64_t high_addr = first_addr < second_addr ? second_addr : first_addr;
    return ((low_addr ^ high_addr) == 128) && ((low_addr & 127) == 0);
}

Transaction *Rmw::build_merged_write_transaction(Transaction *first, Transaction *second, uint64_t merged_task, bool mask_wcmd) {
    Transaction *lower = first;
    Transaction *upper = second;
    if (second != NULL && second->address < first->address) {
        lower = second;
        upper = first;
    }
    Transaction *merged = new Transaction(*lower);
    merged->task = merged_task;
    merged->address = lower->address & ~uint64_t(127);
    merged->mask_wcmd = mask_wcmd;
    merged->ecc_flag = false;
    merged->burst_length = mask_wcmd ? ((first->burst_length + 1) * 2 - 1)
            : ((first->burst_length + 1) + (upper->burst_length + 1) - 1);
    merged->data_size = (merged->burst_length + 1) * DMC_DATA_BUS_BITS / 8;
    merged->data_ready_cnt = 0;
    return merged;
}

bool Rmw::write_merge_response(uint64_t task, uint8_t ch) {
    return top->emit_write_done(ch, task, 0, 0, 0);
}

void Rmw::update_write_merge_resp() {
    if (pending_write_merge_resps.empty()) return;
    if (pre_write_merge_resp_time == now()) return;
    uint64_t wait_task = pending_write_merge_resps[0].wait_data_task;
    if (wait_task != 0xffffffffffffffffull) {
        for (auto &entry : bypassed_merged_writes) {
            if (entry.second.first_task == wait_task && entry.second.first_remaining != 0) return;
        }
        for (size_t i = 0; i < write_merge_data_remaps.size(); i++) {
            if (write_merge_data_remaps[i].src_task == wait_task) return;
        }
    }
    if (write_merge_response(pending_write_merge_resps[0].task, pending_write_merge_resps[0].channel)) {
        pre_write_merge_resp_time = now();
        pending_write_merge_resps.erase(pending_write_merge_resps.begin());
    }
}

bool Rmw::dispatch_write_merge_entry(size_t index, bool force_mask_wcmd) {
    if (index >= write_merge_buffer.size()) return false;
    WriteMergeEntry entry = write_merge_buffer[index];
    if (entry.first_trans == NULL) return false;
    unsigned first_beats = entry.first_trans->burst_length + 1;
    unsigned second_beats = entry.has_second ? entry.second_trans->burst_length + 1 : 0;
    if (entry.first_data_ready_cnt < first_beats) return false;
    if (entry.has_second && entry.second_data_ready_cnt < second_beats) return false;
    bool mask_wcmd = force_mask_wcmd && !entry.has_second;
    uint64_t dispatch_task = entry.has_second ? entry.second_trans->task : entry.first_trans->task;
    Transaction *dispatch_trans = (!entry.has_second && !mask_wcmd) ? entry.first_trans : build_merged_write_transaction(entry.first_trans, entry.second_trans, dispatch_task, mask_wcmd);
    uint8_t ch = (EM_ENABLE && EM_MODE==0) ? 0 : dispatch_trans->channel;
    bool ret = top->channels[ch]->addTransaction(dispatch_trans);
    if (!ret) {
        if (dispatch_trans != entry.first_trans) delete dispatch_trans;
        return false;
    }
    track_write_command(dispatch_task, first_beats + second_beats);
    if (entry.has_second) top->mark_merged_write(dispatch_task, entry.first_trans->task);
    write_merge_buffer.erase(write_merge_buffer.begin() + index);
    rmw_cmd_cnt--;
    wcmd_cnt--;
    if (entry.has_second) {
        for (size_t j = 0; j < first_beats + second_beats; j++) {
            WdataToSend.push_back(dispatch_task);
            WdataChannel.push_back(ch);
        }
        top->channels[ch]->write_map[entry.first_trans->task].num_256bit = first_beats;
        pending_write_data_cnt.erase(entry.first_trans->task);
        pending_write_data_cnt.erase(entry.second_trans->task);
        totalWriteMergePair++;
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- WCMERGE_PAIR_DISPATCH :: first_task="<<entry.first_trans->task
                    <<" second_task="<<entry.second_trans->task<<" merged_task="<<dispatch_task
                    <<" first_addr=0x"<<hex<<entry.first_trans->address<<" second_addr=0x"<<entry.second_trans->address<<dec<<endl);
        }
        delete entry.first_trans;
        delete entry.second_trans;
    } else if (mask_wcmd) {
        for (size_t j = 0; j < first_beats; j++) {
            WdataToSend.push_back(dispatch_task);
            WdataChannel.push_back(ch);
        }
        top->channels[ch]->memoryController->rmw_rd_finish[dispatch_task] = false;
        pending_write_data_cnt.erase(entry.first_trans->task);
        totalWriteMergeUnpairedToRmw++;
        delete entry.first_trans;
    } else {
        for (size_t j = 0; j < first_beats; j++) {
            WdataToSend.push_back(dispatch_task);
            WdataChannel.push_back(ch);
        }
        pending_write_data_cnt.erase(entry.first_trans->task);
        totalWriteMergeUnpairedDirect++;
    }
    return true;
}

bool Rmw::pump_write_merge_buffer() {
    for (size_t i = 0; i < write_merge_buffer.size(); i++) {
        bool timeout = WCMD_MERGE_TIMEOUT != 0 && now() - write_merge_buffer[i].enqueue_time >= WCMD_MERGE_TIMEOUT;
        if (write_merge_buffer[i].has_second || timeout) {
            if (dispatch_write_merge_entry(i, timeout && UNPAIRED_TO_RMW_EN)) return true;
        }
    }
    return false;
}

bool Rmw::flush_one_write_merge_entry() {
    if (write_merge_buffer.empty()) return false;
    return dispatch_write_merge_entry(0, UNPAIRED_TO_RMW_EN);
}

bool Rmw::flushWriteMergeBuffer() {
    bool progressed = false;
    while (!write_merge_buffer.empty() && pump_write_merge_buffer()) {
        progressed = true;
    }
    while (!write_merge_buffer.empty()) {
        if (!flush_one_write_merge_entry()) break;
        progressed = true;
    }
    return progressed;
}

bool Rmw::handle_write_merge_transaction(Transaction *trans) {
    totalWriteMergeInput++;
    for (size_t i = 0; i < RmwQue.size(); i++) {
        Transaction *first = RmwQue[i];
        if (can_merge_write_pair(first, trans)) {
            unsigned first_beats = first->burst_length + 1;
            unsigned second_data_ready_cnt = 0;
            auto pending_second = pending_write_data_cnt.find(trans->task);
            if (pending_second != pending_write_data_cnt.end()) {
                second_data_ready_cnt = pending_second->second;
                pending_write_data_cnt.erase(pending_second);
            }

            uint64_t first_task = first->task;
            uint64_t second_task = trans->task;
            uint8_t upstream_channel = log_channel;
            Transaction *merged = build_merged_write_transaction(first, trans, second_task, false);
            merged->data_ready_cnt = first->data_ready_cnt + second_data_ready_cnt;
            merged->arb_time = now() + ((RMW_CONF_SIZE == 32) ? 3 : 2);

            if (RMW_CMD_MODE == 2) {
                bypassed_merged_writes.emplace(second_task, BypassedMergedWrite(first_task, second_task,
                        first_beats - first->data_ready_cnt, trans->burst_length + 1 - second_data_ready_cnt));
            } else if (first->data_ready_cnt < first_beats) {
                write_merge_data_remaps.push_back(WriteMergeDataRemap(first_task, second_task,
                        first_beats - first->data_ready_cnt));
            }
            write_merge_first_resp_task[second_task] = first_task;
            write_merge_first_resp_channel[second_task] = upstream_channel;

            delete first;
            delete trans;
            RmwQue[i] = merged;
            RmwCmdState[i]->task = second_task;
            RmwCmdState[i]->rmwTimeAdded = now();
            RmwCmdState[i]->rmwState = QUE_WAITING;
            RmwConfCnt[i]->task = second_task;
            RmwConfCnt[i]->ad_conf_cnt = 0;
            totalWriteMergePair++;

            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- WCMERGE_RMWQUE_PAIR :: first_task="<<first_task
                        <<" second_task="<<second_task<<" merged_task="<<second_task
                        <<" data_ready_cnt="<<merged->data_ready_cnt<<" first_addr=0x"<<hex<<merged->address
                        <<dec<<" rmw_cmd_cnt="<<rmw_cmd_cnt<<endl);
            }
            return true;
        }
    }
    bool rmw_que_full = rmw_cmd_cnt >= RMW_QUE_DEPTH && RMW_QUE_DEPTH != 0;
    if (rmw_que_full) {
        totalWriteMergeBufferFull++;
        return false;
    }
    auto pending_it = pending_write_data_cnt.find(trans->task);
    if (pending_it != pending_write_data_cnt.end()) {
        trans->data_ready_cnt = pending_it->second;
        pending_write_data_cnt.erase(pending_it);
    }

    pre_req_time = now();
    cmd_state *c = new cmd_state;
    c->task = trans->task;
    c->rmwTimeAdded = now();
    trans->arb_time = now() + ((RMW_CONF_SIZE == 32) ? 3 : 2);
    conf_state *conf = new conf_state;
    conf->task = trans->task;
    RmwConfCnt.push_back(conf);
    RmwQue.push_back(trans);
    RmwCmdState.push_back(c);
    rmw_cmd_cnt++;
    wcmd_cnt++;
    totalWrites++;
    totalFullWrites++;
    totalTransactions++;
    if (DEBUG_BUS) {
        PRINTN(setw(10)<<now()<<" -- WCMERGE_RMWQUE_ADD :: task="<<trans->task<<" addr=0x"<<hex<<trans->address<<dec
                <<" data_ready_cnt="<<trans->data_ready_cnt<<" rmw_cmd_cnt="<<rmw_cmd_cnt<<endl);
    }
    return true;
}

void Rmw::rebuild_conflict_state() {
    for (auto conf : RmwConfCnt) {
        delete conf;
    }
    RmwConfCnt.clear();
    for (size_t i = 0; i < RmwQue.size(); i++) {
        conf_state *c = new conf_state;
        c->task = RmwQue[i]->task;
        for (size_t j = 0; j < i; j++) {
            if (address_conf(RmwQue[i], RmwQue[j])) {
                c->ad_conf_cnt++;
            }
        }
        RmwConfCnt.push_back(c);
    }
}

bool Rmw::is_unpaired_write_merge_timeout(Transaction *trans, cmd_state *state) {
    if (!is_write_merge_candidate(trans)) return false;
    if (WCMD_MERGE_TIMEOUT == 0) return false;
    return now() - state->rmwTimeAdded >= WCMD_MERGE_TIMEOUT;
}

bool Rmw::remap_write_merge_data(uint32_t *data, uint64_t task) {
    for (size_t i = 0; i < write_merge_data_remaps.size(); i++) {
        if (write_merge_data_remaps[i].src_task == task) {
            uint64_t dst_task = write_merge_data_remaps[i].dst_task;
            if (!addData(data, channel, dst_task)) return false;
            write_merge_data_remaps[i].remaining_beats--;
            if (write_merge_data_remaps[i].remaining_beats == 0) {
                write_merge_data_remaps.erase(write_merge_data_remaps.begin() + i);
            }
            return true;
        }
    }
    return false;
}

bool Rmw::is_write_merge_data_task(uint64_t task) const {
    if (pending_write_data_cnt.find(task) != pending_write_data_cnt.end()) return true;
    for (size_t i = 0; i < write_merge_buffer.size(); i++) {
        if (write_merge_buffer[i].first_trans != NULL && write_merge_buffer[i].first_trans->task == task) return true;
        if (write_merge_buffer[i].has_second && write_merge_buffer[i].second_trans != NULL && write_merge_buffer[i].second_trans->task == task) return true;
    }
    return false;
}

bool Rmw::add_write_merge_data(uint32_t *data, uint64_t task) {
    (void)data;
    for (size_t i = 0; i < write_merge_buffer.size(); i++) {
        WriteMergeEntry &entry = write_merge_buffer[i];
        if (entry.first_trans != NULL && entry.first_trans->task == task) {
            if (entry.first_data_ready_cnt <= entry.first_trans->burst_length) entry.first_data_ready_cnt++;
            pump_write_merge_buffer();
            return true;
        }
        if (entry.has_second && entry.second_trans != NULL && entry.second_trans->task == task) {
            if (entry.second_data_ready_cnt <= entry.second_trans->burst_length) entry.second_data_ready_cnt++;
            pump_write_merge_buffer();
            return true;
        }
    }
    pending_write_data_cnt[task]++;
    return true;
}

bool Rmw::hasPendingWork() const {
    return !write_merge_buffer.empty() || !pending_write_merge_resps.empty() || !write_merge_data_remaps.empty()
            || !pending_write_data_cnt.empty() || !fast_bypass_write_data_cnt.empty()
            || !WdataToSend.empty() || !RmwCmdResp.empty() || !RmwQue.empty();
}


bool Rmw::addData(uint32_t *data, uint32_t channel, uint64_t task) {

    pre_req_data_time = now();
    for (auto it = bypassed_merged_writes.begin(); it != bypassed_merged_writes.end(); ++it) {
        BypassedMergedWrite &entry = it->second;
        bool first_source = task == entry.first_task && entry.first_remaining != 0;
        bool second_source = task == entry.merged_task && entry.second_remaining != 0;
        if (!first_source && !second_source) continue;
        if (!entry.dispatched) {
            for (auto &rmwq : RmwQue) {
                if (rmwq->task == entry.merged_task) {
                    rmwq->data_ready_cnt++;
                    if (first_source) entry.first_remaining--;
                    else entry.second_remaining--;
                    return true;
                }
            }
            return false;
        }
        uint8_t ch = (EM_ENABLE && EM_MODE==0) ? 0 : channel;
        bool ret = top->channels[ch]->addData(data, entry.merged_task, false);
        if (!ret) return false;
        check_write_data(entry.merged_task);
        if (first_source) entry.first_remaining--;
        else entry.second_remaining--;
        if (entry.first_remaining == 0 && entry.second_remaining == 0) {
            bypassed_merged_writes.erase(it);
        }
        return true;
    }
    auto fast_bypass = fast_bypass_write_data_cnt.find(task);
    if (fast_bypass != fast_bypass_write_data_cnt.end()) {
        uint8_t ch = (EM_ENABLE && EM_MODE==0) ? 0 : channel;
        bool ret = top->channels[ch]->addData(data, task, false);
        if (ret) {
            check_write_data(task);
            if (fast_bypass->second > 1) {
                fast_bypass->second--;
            } else {
                fast_bypass_write_data_cnt.erase(fast_bypass);
            }
        }
        return ret;
    }
    if (remap_write_merge_data(data, task)) {
        return true;
    }
    if (is_write_merge_data_task(task)) {
        if (DEBUG_BUS) {
             PRINTN(setw(10)<<now()<<" -- WCMERGE_DATA_READY :: task="<<task<<endl);
        }
        return add_write_merge_data(data, task);
    }
    
    bool task_match = false;
    for (auto &rmwq : RmwQue) {
        if (task == rmwq->task) {
            task_match=true;
        };
        if ((rmwq->transactionType == DATA_WRITE)
                && rmwq->data_ready_cnt <= rmwq->burst_length && task==rmwq->task) {
            rmwq->data_ready_cnt ++;
            if (DEBUG_BUS) {
                 PRINTN(setw(10)<<now()<<" -- RMW_MATCH :: data_ready_cnt:"<<rmwq->data_ready_cnt
                         <<", "<<rmwq->data_size<<", task="<<rmwq->task<<endl);
            }
            return true;
        }
    }

    if (task_match==false) {
        if (WCMD_MERGE_EN) {
            pending_write_data_cnt[task]++;
            if (DEBUG_BUS) {
                 PRINTN(setw(10)<<now()<<" -- WCMERGE_EARLY_DATA :: task="<<task
                         <<" ready="<<pending_write_data_cnt[task]<<endl);
            }
            return true;
        }
        if (DEBUG_BUS) {
             PRINTN(setw(10)<<now()<<" -- RMW_WDATA_BYPASS :: task="<<task<<endl);
        }
        bool ret = false;
        if (EM_ENABLE && EM_MODE==0) {
            ret = top->channels[0]->addData(data ,task, false);
        } else {
            ret = top->channels[channel]->addData(data ,task, false);
        }
        if (ret) check_write_data(task);
        return ret;
    }

    ERROR(setw(10)<<now()<<" -- Impossible wdata, task="<<task);
    assert(0);

}

bool Rmw::addTransaction(Transaction * trans) {

//    if (trans->data_size == 0) {
//         DEBUG(" task="<<trans->task<<" data size="<<trans->data_size);
//    }

    if ((trans->transactionType==DATA_READ)&&(trans->mask_wcmd==true)){
        ERROR(setw(10)<<now()<<" -- No mask Flag In Read, task="<<trans->task<<" type="<<trans->transactionType
               <<" address="<<hex<<trans->address<<" mask_write="<<trans->mask_wcmd);
        assert(0);
    }


    uint8_t ch = (EM_ENABLE && EM_MODE==0) ? 0 : trans->channel;
    bool rmw_que_full  = rmw_cmd_cnt >= RMW_QUE_DEPTH && RMW_QUE_DEPTH != 0;
    bool rmw_que_empty = rmw_cmd_cnt == 0;

    if (is_write_merge_candidate(trans)) {
        return handle_write_merge_transaction(trans);
    }

    if (rmw_que_full) {
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- RMW BP CMD :: task="<<trans->task<<" type="<<trans->transactionType<<" mask_write="<<trans->mask_wcmd
                    <<" qos="<<trans->qos<<" burst_length:"<<trans->burst_length<<" address="<<hex<<trans->address
                    <<dec<<" rank="<<trans->rank<<" bank="<<trans->bankIndex<<" row="<<trans->row<<" channel="<<trans->channel<<" (rmw_cmd_cnt:"<<rmw_cmd_cnt
                    <<")"<<endl);
        }
        return false;
    }

    if (rmw_que_empty) {
        if ((trans->transactionType==DATA_READ)||((trans->transactionType==DATA_WRITE)&&(RMW_CMD_MODE==0)&&(trans->mask_wcmd==false))) {
            if (DEBUG_BUS) {
                PRINTN(setw(10)<<now()<<" -- RMW BYPASS :: task="<<trans->task<<" type="<<trans->transactionType<<" mask_write="<<trans->mask_wcmd
                        <<" qos="<<trans->qos<<" burst_length:"<<trans->burst_length<<" address="<<hex<<trans->address
                        <<dec<<" rank="<<trans->rank<<" bank="<<trans->bankIndex<<" row="<<trans->row<<" channel="<<trans->channel<<" (rmw_cmd_cnt:"<<rmw_cmd_cnt
                        <<")"<<endl);
            }

            bool ret = top->channels[ch]->addTransaction(trans); 
            
            if (ret) {
                if (trans->transactionType==DATA_READ){
                    totalBypassReads++;
                } else {
                    track_write_command(trans->task, trans->burst_length + 1);
                    totalBypassWrites++;
                    totalFullWrites ++;
                    unsigned beats = trans->burst_length + 1;
                    auto pending = pending_write_data_cnt.find(trans->task);
                    if (pending != pending_write_data_cnt.end()) {
                        unsigned pending_beats = std::min(pending->second, beats);
                        for (unsigned i = 0; i < pending_beats; i++) {
                            WdataToSend.push_back(trans->task);
                            WdataChannel.push_back(ch);
                        }
                        beats -= pending_beats;
                        pending_write_data_cnt.erase(pending);
                    }
                    if (beats != 0) {
                        fast_bypass_write_data_cnt[trans->task] = beats;
                    }
                }
            }
            
            return ret;
        }
    }


    pre_req_time = now();
    cmd_state *c = new cmd_state;
    c->task = trans->task;
    c->rmwTimeAdded = now();
    if (trans->transactionType == DATA_READ) {
        cmd_set_conflict(trans);
        if (RMW_CONF_SIZE == 32) {
            trans->arb_time = now() + 2;
        } else {
            trans->arb_time = now() + 1;
        }

        rcmd_cnt ++;
        rmw_cmd_cnt ++;
        totalReads ++;
        totalTransactions ++;
        RmwQue.push_back(trans);
        RmwCmdState.push_back(c);


        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- RMW_ADD ::[R]B["<<trans->burst_length<<"]"<<"QOS["<<trans->qos<<"] addr="<<hex
                    <<trans->address<<dec<<" addr_col="<<trans->addr_col<<" task="<<trans->task<<" type="<<trans->transactionType<<" mask_write="<<trans->mask_wcmd
                    <<" rank="<<trans->rank<<" group="<<trans->group<<" bank="
                    <<trans->bankIndex<<" row="<<trans->row<<" channel="<<trans->channel<<" (rmw_cmd_cnt:"<<rmw_cmd_cnt<<")"<<endl);
        }
        return true;
    } else {
        cmd_set_conflict(trans);
        auto pending = pending_write_data_cnt.find(trans->task);
        if (pending != pending_write_data_cnt.end()) {
            unsigned beats = trans->burst_length + 1;
            trans->data_ready_cnt += std::min(pending->second, beats - trans->data_ready_cnt);
            pending_write_data_cnt.erase(pending);
        }
        if (RMW_CONF_SIZE == 32) {
            trans->arb_time = now() + 3;
        } else {
            trans->arb_time = now() + 2;
        }
        if (trans->mask_wcmd == true) {
            top->channels[ch]->memoryController->rmw_rd_finish[trans->task] = false;
            totalMaskWrites ++;
        } else {
            totalFullWrites ++;
        }

        rmw_cmd_cnt ++;
        wcmd_cnt ++;
        totalWrites ++;
        totalTransactions ++;
        RmwQue.push_back(trans);
        RmwCmdState.push_back(c);
        
        if (RMW_CMD_MODE==1) {
            gen_cresp(trans->task);
            RmwCmdRespCh.push_back(trans->channel);
        }

//        if (!RmwCmdResp.empty()) {
//            if (pre_cresp_time != now()) {
//                if (cmd_response(RmwCmdResp[0], 0, ch)) {
//                    if (DEBUG_BUS) {
//                        PRINTN(setw(10)<<now()<<" -- Rmw Cresp Received :: task="<<RmwCmdResp[0]<<" RMW MODE="<<RMW_CMD_MODE<<endl);
//                    }
//                    pre_cresp_time = now();
//                    RmwCmdResp.erase(RmwCmdResp.begin());
//                } else {
//                    if (DEBUG_BUS) {
//                        PRINTN(setw(10)<<now()<<" -- Rmw Cresp Back Pressure :: task="<<RmwCmdResp[0]<<" RMW MODE="<<RMW_CMD_MODE<<endl);
//                    }
//                }
//            }
//        }
        
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- RMW_ADD :: [W]B["<<trans->burst_length<<"]"<<"QOS["<<trans->qos<<"] addr="<<hex
                    <<trans->address<<dec<<" addr_col="<<trans->addr_col<<" task="<<trans->task<<" type="<<trans->transactionType<<" mask_write="<<trans->mask_wcmd
                    <<" rank="<<trans->rank<<" group="<<trans->group<<" bank="
                    <<trans->bankIndex<<" row="<<trans->row<<" channel="<<trans->channel<<" (rmw_cmd_cnt:"<<rmw_cmd_cnt<<")"<<endl);
        }
        return true;
    }
}

void Rmw::gen_cresp(uint64_t task) {
    RmwCmdResp.push_back(task);
}

bool Rmw::cmd_response(uint64_t task,uint64_t address, uint8_t ch) {
     return ((*(top->channels[ch])->CmdResp)(ch,task,0,0,0));
}

void Rmw::update_cresp() {
    // RMW cresp return to ha 
    if (!RmwCmdResp.empty()) {
        if (pre_cresp_time != now()) {
            if (cmd_response(RmwCmdResp[0], 0, RmwCmdRespCh[0])) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- Rmw Cresp Received :: task="<<RmwCmdResp[0]<<" RMW MODE="<<RMW_CMD_MODE<<endl);
                }
                pre_cresp_time = now();
                RmwCmdResp.erase(RmwCmdResp.begin());
                RmwCmdRespCh.erase(RmwCmdRespCh.begin());
            } else {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- Rmw Cresp Back Pressure :: task="<<RmwCmdResp[0]<<" RMW MODE="<<RMW_CMD_MODE<<endl);
                }
            }
        }
    }
}

    void Rmw::func_check() {
    if (RmwQue.size() != rmw_cmd_cnt) {
        ERROR(setw(10)<<now()<<" -- RmwQue Unmatch, RmwQue="<<RmwQue.size()<<", rmw_cmd_cnt="<<rmw_cmd_cnt);
        assert(0);
    }
}

void Rmw::update() {
    // #region debug-point D:rmw-occupancy
    dbg_cycles++;
    dbg_max_queue = std::max(dbg_max_queue, rmw_cmd_cnt);
    if (full()) dbg_full_cycles++;
    bool dbg_has_ready = false;
    for (size_t i = 0; i < RmwQue.size(); i++) {
        if (RmwQue[i]->transactionType == DATA_READ || RmwCmdState[i]->rmwState == SEND_READY) {
            dbg_has_ready = true;
            break;
        }
    }
    if (!RmwQue.empty() && !dbg_has_ready) dbg_no_ready_cycles++;
    // #endregion
    update_cresp();
    update_write_merge_resp();
#if 0
    func_check();
#endif
    update_state();
//    check_timeout();
    pump_write_merge_buffer();
    sch_que();
    arb_node();
    send_wdata();
}

void Rmw::update_state() {
    if (!DEBUG_RMW_STATE) return;
    unsigned size = RmwQue.size();
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
    PRINTN("Rmw Total Status: R:"<<rcmd_cnt<<" W:"<<wcmd_cnt<<" R+W:"<<rmw_cmd_cnt<<endl);
    for (unsigned i = 0; i < size; i ++) {
        auto t = RmwQue[i];
        auto r = RmwConfCnt[i];
        auto s = RmwCmdState[i];
        PRINTN("Rmw time: "<<now()<<" | type="<<t->transactionType<<" | task="<<t->task<<" | bank="<<t->bankIndex<<" | rank="<<t->rank<<" | row="
                <<t->row<<" | address="<<hex<<t->address<<dec<<" | addr_col="<<t->addr_col<<" | len="<<t->burst_length<<" | channel="<<t->channel<<" | data_size="<<t->data_size
                <<" | data_ready_cnt="<<t->data_ready_cnt<<" | timeout="<<t->timeout<<" | qos="<<t->qos<<" | pri="<<t->pri
                <<" | rd_byp="<<t->has_active<<" | mask_wcmd="<<t->mask_wcmd<<" | addr_conf_cnt="<<r->ad_conf_cnt<<" | cmd_state="
                <<s->rmwState<<" | RMW MODE="<<RMW_CMD_MODE<<endl);
    }
    PRINTN("--------------------------------------------------------------------------------------------------"<<endl)
}


void Rmw::sch_que() {
    if (RmwQue.empty()) return;

    // command number check
    if ((rcmd_cnt+wcmd_cnt)!=rmw_cmd_cnt) {
        ERROR(setw(10)<<now()<<" -- Cmd Number Chk Failed, No.Read="<<rcmd_cnt<<" No.Write="<<wcmd_cnt<<" No.Total="<<rmw_cmd_cnt);
        assert(0);
    }
    
    unsigned size = RmwQue.size();
    for (unsigned i = 0; i < size; i++) {
        //prevent big latency in RMW QUE
        if ((now() - RmwCmdState[i]->rmwTimeAdded) > 1000000) {
            ERROR(setw(10)<<now()<<" -- task="<<RmwQue[i]->task<<" address="<<hex<<RmwQue[i]->address<<dec
                    <<" rank="<<RmwQue[i]->rank<<" bank="<<RmwQue[i]->bankIndex<<" row="<<RmwQue[i]->row<<" type="
                    <<RmwQue[i]->transactionType<<" mask_wcmd="<<RmwQue[i]->mask_wcmd);
            ERROR(setw(10)<<now()<<" -- error, qos="<<RmwQue[i]->qos<<", pri="<<RmwQue[i]->pri);
            ERROR(setw(10)<<now()<<" -- RMW FATAL ERROR == big latency"<<", chnl:"<<RmwQue[i]->channel);
            assert(0);
        }
        //prevent wdata lost in RMW QUE
        if (RmwQue[i]->transactionType == DATA_WRITE) {
            if (now() - RmwCmdState[i]->rmwTimeAdded > 100000 && RmwQue[i]->data_ready_cnt <= RmwQue[i]->burst_length) {
                ERROR(setw(10)<<now()<<" -- RMW_DMC["<<RmwQue[i]->channel<<"] task="<<RmwQue[i]->task<<" Wdata number miss match, EXP="
                        <<RmwQue[i]->burst_length<<", ACT="<<RmwQue[i]->data_ready_cnt);
                assert(0);
            }
        }


        uint8_t ch = (EM_ENABLE && EM_MODE==0) ? 0 : RmwQue[i]->channel;
        //Merge between write data and read data (Mask Write)
        if ((RmwQue[i]->transactionType==DATA_WRITE) && (RmwQue[i]->mask_wcmd==true)) {
            auto it = top->channels[ch]->memoryController->rmw_rd_finish.find(RmwQue[i]->task);
            if (it != top->channels[ch]->memoryController->rmw_rd_finish.end()) {
                if((RmwQue[i]->data_ready_cnt>=(RmwQue[i]->burst_length + 1)) && (it->second==true)){
                    RmwCmdState[i]->rmwState = SEND_READY;
                    top->channels[ch]->memoryController->rmw_rd_finish.erase(it);
                    if (DEBUG_BUS) {
                        PRINTN(setw(10)<<now()<<" -- RMW WDATA RDATA MERGE ::[MaskW]B["<<RmwQue[i]->burst_length<<"]"<<"QOS["<<RmwQue[i]->qos<<"] addr="<<hex
                            <<RmwQue[i]->address<<dec<<" task="<<RmwQue[i]->task<<" type="<<RmwQue[i]->transactionType<<" mask_write="<<RmwQue[i]->mask_wcmd
                            <<" rank="<<RmwQue[i]->rank<<" group="<<RmwQue[i]->group<<" bank="
                            <<RmwQue[i]->bankIndex<<" row="<<RmwQue[i]->row<<" mode="<<RMW_CMD_MODE<<" (rmw_cmd_cnt:"<<rmw_cmd_cnt<<")"<<endl);
                    } 
                }
            }
        }

        //Full Write: waiting for write data
        if ((RmwQue[i]->transactionType==DATA_WRITE) && (RmwQue[i]->mask_wcmd==false)) {
            if(RmwQue[i]->data_ready_cnt >= (RmwQue[i]->burst_length + 1)) {
                bool unpaired_write_merge = is_write_merge_candidate(RmwQue[i]);
                if (unpaired_write_merge && !is_unpaired_write_merge_timeout(RmwQue[i], RmwCmdState[i])) {
                    continue;
                }
                if (unpaired_write_merge && UNPAIRED_TO_RMW_EN) {
                    RmwQue[i]->mask_wcmd = true;
                    RmwQue[i]->burst_length = (RmwQue[i]->burst_length + 1) * 2 - 1;
                    RmwQue[i]->data_size = (RmwQue[i]->burst_length + 1) * DMC_DATA_BUS_BITS / 8;
                    top->channels[ch]->memoryController->rmw_rd_finish[RmwQue[i]->task] = false;
                    totalMaskWrites++;
                    if (totalFullWrites > 0) totalFullWrites--;
                    totalWriteMergeUnpairedToRmw++;
                    continue;
                }
                if (unpaired_write_merge && write_merge_unpaired_direct_accounted.find(RmwQue[i]->task) == write_merge_unpaired_direct_accounted.end()) {
                    write_merge_unpaired_direct_accounted[RmwQue[i]->task] = true;
                    totalWriteMergeUnpairedDirect++;
                }
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- RMW WDATA MATCH ::[FullW]B["<<RmwQue[i]->burst_length<<"]"<<"QOS["<<RmwQue[i]->qos<<"] addr="<<hex
                        <<RmwQue[i]->address<<dec<<" task="<<RmwQue[i]->task<<" type="<<RmwQue[i]->transactionType<<" mask_write="<<RmwQue[i]->mask_wcmd
                        <<" rank="<<RmwQue[i]->rank<<" group="<<RmwQue[i]->group<<" bank="
                        <<RmwQue[i]->bankIndex<<" row="<<RmwQue[i]->row<<" mode="<<RMW_CMD_MODE<<" (rmw_cmd_cnt:"<<rmw_cmd_cnt<<")"<<endl);
                } 
                RmwCmdState[i]->rmwState = SEND_READY;   
            }
        }
    }
}



void Rmw::arb_node() {
    if (RmwQue.empty()) return;

    unsigned size = RmwQue.size();
    for (unsigned i = 0; i < size; i++) {
        if ((RmwQue[i]->transactionType == DATA_READ) && (RmwCmdState[i]->rmwState!=QUE_WAITING)){
            ERROR(setw(10)<<now()<<" -- Read Cmd State Wrong, task="<<RmwQue[i]->task<<" type="<<RmwQue[i]->transactionType<<" channel="<<RmwQue[i]->channel
                   <<" address="<<hex<<RmwQue[i]->address<<" cmd_state="<<RmwCmdState[i]->rmwState<<" mask_write="<<RmwQue[i]->mask_wcmd
                   <<" rmw_mode="<<RMW_CMD_MODE);
            assert(0);
        }

        if (now() < RmwQue[i]->arb_time) { 
            continue;
        }
        if (RmwConfCnt[i]->ad_conf_cnt != 0) {
            continue;
        }
        if ((RmwCmdState[i]-> rmwState == MERGE_READ)&&(RmwQue[i]->mask_wcmd==true)) {
            continue;
        }
        bool bypassed_merged = RMW_CMD_MODE == 2
                && bypassed_merged_writes.find(RmwQue[i]->task) != bypassed_merged_writes.end();
        if ((RmwQue[i]->transactionType == DATA_WRITE)&&(RmwQue[i]->mask_wcmd==false)
                && (RmwCmdState[i]->rmwState!=SEND_READY) && !bypassed_merged) {
            continue;
        }

        uint8_t ch = (EM_ENABLE && EM_MODE==0) ? 0 : RmwQue[i]->channel;
        
        // RMW_CMD_MODE: 0 -> fast command mode; 1 -> non fast command mode
        if ((RmwQue[i]->transactionType == DATA_WRITE)&&(RmwQue[i]->mask_wcmd==true)&&(RmwCmdState[i]->rmwState==QUE_WAITING)) {  
            Transaction *trans = new Transaction(RmwQue[i]);
            trans->transactionType = DATA_READ; 
            if (top->channels[ch]->addTransaction(trans)) {
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- RMW SCH :: [MERGE_READ_CMD] task="<<RmwQue[i]->task<<" type="<<RmwQue[i]->transactionType
                            <<" qos="<<RmwQue[i]->qos<<" burst_length:"<<RmwQue[i]->burst_length<<" channel="<<RmwQue[i]->channel<<" address="<<hex<<RmwQue[i]->address
                            <<dec<<" rank="<<RmwQue[i]->rank<<" bank="<<RmwQue[i]->bankIndex<<" row="<<RmwQue[i]->row<<" mode="<<RMW_CMD_MODE<<" rmw_cmd_cnt="<<rmw_cmd_cnt<<endl);
                }
                RmwCmdState[i]->rmwState = MERGE_READ;
                break;
            } else {
                delete trans;
            }
        } else if ((RmwQue[i]->transactionType == DATA_READ)   // read cmd
                    || ((RmwQue[i]->transactionType == DATA_WRITE)&&(RmwQue[i]->mask_wcmd==false)
                        && ((RmwCmdState[i]->rmwState==SEND_READY) || bypassed_merged))  // full write cmd
                    || ((RmwQue[i]->transactionType == DATA_WRITE)&&(RmwQue[i]->mask_wcmd==true)&&(RmwCmdState[i]->rmwState==SEND_READY))) {     // mask write cmd
            if (top->channels[ch]->addTransaction(RmwQue[i])) {
                if (write_merge_first_resp_task.find(RmwQue[i]->task) != write_merge_first_resp_task.end()) {
                    top->mark_merged_write(RmwQue[i]->task, write_merge_first_resp_task[RmwQue[i]->task]);
                }
                // #region debug-point D:rmw-occupancy
                dbg_dispatches++;
                // #endregion
                if (DEBUG_BUS) {
                    PRINTN(setw(10)<<now()<<" -- RMW SCH :: task="<<RmwQue[i]->task<<" type="<<RmwQue[i]->transactionType<<" mask_write="<<RmwQue[i]->mask_wcmd
                            <<" qos="<<RmwQue[i]->qos<<" burst_length:"<<RmwQue[i]->burst_length<<" channel="<<RmwQue[i]->channel<<" data_ready_cnt="<<RmwQue[i]->data_ready_cnt<<" address="<<hex<<RmwQue[i]->address
                            <<dec<<" rank="<<RmwQue[i]->rank<<" bank="<<RmwQue[i]->bankIndex<<" row="<<RmwQue[i]->row<<" rmw_mode="<<RMW_CMD_MODE<<" rmw_cmd_cnt="<<rmw_cmd_cnt<<endl);
                }

                // collect all wdata for sended write cmd
                if (RmwQue[i]->transactionType==DATA_WRITE) {
                    uint64_t t = RmwQue[i]->task;
                    track_write_command(t, RmwQue[i]->burst_length + 1);
                    auto bypassed = bypassed_merged_writes.find(t);
                    if (bypassed != bypassed_merged_writes.end()) {
                        bypassed->second.dispatched = true;
                        unsigned buffered_beats = (RmwQue[i]->burst_length + 1)
                                - bypassed->second.first_remaining - bypassed->second.second_remaining;
                        for (unsigned j = 0; j < buffered_beats; j++) {
                            WdataToSend.push_back(t);
                            WdataChannel.push_back(ch);
                        }
                    }
                    if (bypassed == bypassed_merged_writes.end()) {
                        for (size_t j = 0; j <= RmwQue[i]->burst_length; j++) {
                            WdataToSend.push_back(t);
                            WdataChannel.push_back(ch);
                        }
                    }
                    auto first_resp = write_merge_first_resp_task.find(RmwQue[i]->task);
                    if (first_resp != write_merge_first_resp_task.end()) {
                        uint64_t second_task = RmwQue[i]->task;
                        top->channels[ch]->write_map[first_resp->second].num_256bit = (RmwQue[i]->burst_length + 1) / 2;
                        write_merge_first_resp_task.erase(first_resp);
                        write_merge_first_resp_channel.erase(second_task);
                    }
                    write_merge_unpaired_direct_accounted.erase(RmwQue[i]->task);
                }

                //update statistic info
                if (RmwQue[i]->transactionType == DATA_READ) {
                    rcmd_cnt --;
                } else {
                    wcmd_cnt --;
                }
                rmw_cmd_cnt --;

                //delete all states related to sended cmd this round
                cmd_release_conflict(RmwQue[i]);
                delete RmwConfCnt[i];
                RmwConfCnt.erase(RmwConfCnt.begin() + i);
                delete RmwCmdState[i];
                RmwCmdState.erase(RmwCmdState.begin() + i);
                RmwQue.erase(RmwQue.begin() + i);
                break;
            } else {
                // #region debug-point D:rmw-occupancy
                dbg_dmc_rejects++;
                unsigned mc_occupancy = 0;
                unsigned mc_wb = 0;
                unsigned mc_mask_read = 0;
                for (auto *mc_trans : top->channels[ch]->memoryController->transactionQueue) {
                    mc_occupancy++;
                    if (mc_trans->wb) mc_wb++;
                    if (mc_trans->transactionType == DATA_READ && mc_trans->mask_wcmd) mc_mask_read++;
                }
                unsigned rmw_ready = 0;
                unsigned rmw_waiting = 0;
                for (unsigned j = 0; j < RmwCmdState.size(); j++) {
                    if (RmwCmdState[j]->rmwState == SEND_READY) rmw_ready++;
                    if (RmwCmdState[j]->rmwState == QUE_WAITING) rmw_waiting++;
                }
                dbg_reject_mc_occupancy_sum += mc_occupancy;
                dbg_reject_mc_wb_sum += mc_wb;
                dbg_reject_mc_mask_read_sum += mc_mask_read;
                dbg_reject_rmw_ready_sum += rmw_ready;
                dbg_reject_rmw_waiting_sum += rmw_waiting;
                dbg_reject_mc_occupancy_max = std::max(dbg_reject_mc_occupancy_max, mc_occupancy);
                dbg_reject_mc_wb_max = std::max(dbg_reject_mc_wb_max, mc_wb);
                // #endregion
            }
        } else {
            ERROR(setw(10)<<now()<<" -- Such Cmd Not Expected, task="<<RmwQue[i]->task<<" type="<<RmwQue[i]->transactionType
                   <<" address="<<hex<<RmwQue[i]->address<<" cmd_state="<<RmwCmdState[i]->rmwState<<" mask_write="<<RmwQue[i]->mask_wcmd
                   <<" rmw_mode="<<RMW_CMD_MODE);
            assert(0);
            
        }
    }
}


void Rmw::send_wdata() {
    if (!WdataToSend.empty() && !WdataChannel.empty()) {
        uint64_t t = WdataToSend[0];
        int ch = WdataChannel[0];
        if (DEBUG_BUS) {
            PRINTN(setw(10)<<now()<<" -- RMW_SEND_WDATA :: task="<<t<<" channel="<<ch<<endl);
        }
        bool ret = top->channels[ch]->addData(0, t, false);
        if (ret) {
            check_write_data(t);
            WdataToSend.erase(WdataToSend.begin());
            WdataChannel.erase(WdataChannel.begin());
        }
    }
}

void Rmw::track_write_command(uint64_t task, unsigned beats) {
    wdata_order_queue.push_back(WdataOrderEntry(task, beats));
}

void Rmw::check_write_data(uint64_t task) {
    if (wdata_order_queue.empty() || wdata_order_queue.front().task != task) {
        ERROR(setw(10)<<now()<<" -- RMW WDATA ORDER ERROR :: actual_task="<<task
                <<" expected_task="<<(wdata_order_queue.empty() ? 0xffffffffffffffffull : wdata_order_queue.front().task)
                <<" remaining="<<(wdata_order_queue.empty() ? 0 : wdata_order_queue.front().remaining_beats));
        assert(0);
    }
    if (--wdata_order_queue.front().remaining_beats == 0) {
        wdata_order_queue.erase(wdata_order_queue.begin());
    }
}


unsigned Rmw:: GetRmwQsize() {
    return (rmw_cmd_cnt);
}

void Rmw::check_cnt() {

    uint32_t size = GetRmwQsize();
    if (size >= rmw_que_cnt.size()) {
        rmw_que_cnt.resize(size + 1, 0);
    }
    rmw_que_cnt.at(size)++;
}

void Rmw::register_write(uint64_t address, uint32_t data) {
    uint32_t offset = ((address != 0) ? 4 : 0);
    switch (offset) {
        case 0x0:{
            start_cycle = top->now();
            break;
        }
        case 0x4:{
            end_cycle = top->now();
            if (start_cycle != end_cycle && STATE_LOG == true) {
                statistics();
            }
            break;
        }
        default: break;
    }
}


void Rmw::statistics() {
    unsigned size = 0;
    STATE_PRINTN(setiosflags(ios::left));
    STATE_PRINTN("======================================== START ========================================\n");
    STATE_PRINTN("-------------------- Base Message -----------------------------------------------------\n");
    STATE_PRINTN(DDR_TYPE<<" "<<DMC_RATE<<"Mbps, x"<<JEDEC_DATA_BUS_BITS<<", DMC Data Width: "
            <<DMC_DATA_BUS_BITS<<", CKR: "<<setprecision(1)<<WCK2DFI_RATIO<<endl);
    STATE_PRINTN("Current time: "<<fixed<<now()<<", tDFI: "<<setprecision(4)<<tDFI<<endl);

    unsigned reads = totalReads;
    unsigned bypass_reads = totalBypassReads;
    unsigned writes = totalWrites;
    unsigned bypass_writes = totalBypassWrites;
    unsigned full_writes = totalFullWrites;
    unsigned mask_writes = totalMaskWrites;
    unsigned totals = totalTransactions;
//    unsigned address_conf_cnt = addrconf_cnt;
//    unsigned read_cnt = read_cnt;
//    unsigned write_cnt = write_cnt;
//    unsigned mwrite_cnt = mwrite_cnt;

    STATE_PRINTN("-------------------- Task Statistics (DMC Command Number) -----------------------------\n");
    
    STATE_PRINTN("Read            : "<<setw(8)<<reads-pre_reads);
    STATE_PRINTN(" | Total reads       : "<<setw(8)<<reads);
    STATE_PRINTN(" | Bypass Read            : "<<setw(8)<<bypass_reads-pre_bypass_reads);
    STATE_PRINTN(" | Total bypass reads       : "<<setw(8)<<bypass_reads<<" | "<<endl);
    STATE_PRINTN("Write           : "<<setw(8)<<writes-pre_writes);
    STATE_PRINTN(" | Total writes      : "<<setw(8)<<writes);
    STATE_PRINTN(" | Full Write             : "<<setw(8)<<full_writes-pre_full_writes);
    STATE_PRINTN(" | Total full writes        : "<<setw(8)<<full_writes<<" | "<<endl);
    STATE_PRINTN("Mask Write      : "<<setw(8)<<mask_writes-pre_mask_writes);
    STATE_PRINTN(" | Total mask writes : "<<setw(8)<<mask_writes);
    STATE_PRINTN(" | Bypass Write           : "<<setw(8)<<bypass_writes-pre_bypass_writes);
    STATE_PRINTN(" | Total bypass writes      : "<<setw(8)<<bypass_writes<<" | "<<endl);
    STATE_PRINTN("Total           : "<<setw(8)<<totals-pre_totals);
    STATE_PRINTN(" | Total commands    : "<<setw(8)<<totals<<" | "<<endl);

    STATE_PRINTN("-------------------- WCMD Merge Statistics -------------------------------------------\n");
    STATE_PRINTN("Input           : "<<setw(8)<<totalWriteMergeInput-preWriteMergeInput);
    STATE_PRINTN(" | Total input       : "<<setw(8)<<totalWriteMergeInput);
    STATE_PRINTN(" | Pair merge             : "<<setw(8)<<totalWriteMergePair-preWriteMergePair);
    STATE_PRINTN(" | Total pair merge       : "<<setw(8)<<totalWriteMergePair<<" | "<<endl);
    STATE_PRINTN("Unpaired RMW    : "<<setw(8)<<totalWriteMergeUnpairedToRmw-preWriteMergeUnpairedToRmw);
    STATE_PRINTN(" | Total unpaired RMW: "<<setw(8)<<totalWriteMergeUnpairedToRmw);
    STATE_PRINTN(" | Unpaired direct        : "<<setw(8)<<totalWriteMergeUnpairedDirect-preWriteMergeUnpairedDirect);
    STATE_PRINTN(" | Total unpaired direct  : "<<setw(8)<<totalWriteMergeUnpairedDirect<<" | "<<endl);
    STATE_PRINTN("Buffer full     : "<<setw(8)<<totalWriteMergeBufferFull-preWriteMergeBufferFull);
    STATE_PRINTN(" | Total buffer full : "<<setw(8)<<totalWriteMergeBufferFull);
    STATE_PRINTN(" | Buffer used            : "<<setw(8)<<write_merge_buffer.size());
    STATE_PRINTN(" | Pending data tasks     : "<<setw(8)<<pending_write_data_cnt.size()<<" | "<<endl);



//    STATE_PRINTN("-------------------- Confilct Statistics (DDR Command Number) -------------------------\n");
//    STATE_PRINTN(setw(36)<<"Address conflict"<<" : "<<setw(12)<<address_conf_cnt - pre_address_conf_cnt);
//    STATE_PRINTN(" | "<<setw(36)<<"Total address conf cnt"<<" : "<<address_conf_cnt<<endl);


//    STATE_PRINTN("-------------------- RMW Pressure Statistics (Percentage/Cycle) -----------------------\n");
//    float ratio = float(task_cnt) * 100 / STATE_TIME;
//    STATE_PRINTN(setw(15)<<"Cmd valid"<<" : "<<setw(10)<<task_cnt<<" | ");
//    STATE_PRINTN(setw(15)<<"Ratio"<<" : "<<setw(10)<<ratio<<" | ");
//    ratio = float(total_task_cnt) * 100 / now();
//    STATE_PRINTN(setw(15)<<"Total cmd valid"<<" : "<<setw(10)<<total_task_cnt<<" | ");
//    STATE_PRINTN(setw(15)<<"Ratio"<<" : "<<setw(10)<<ratio<<" | "<<endl);
//    ratio = float(bp_cnt) * 100 / (bp_cnt + access_cnt);
//    STATE_PRINTN(setw(15)<<"DMC access"<<" : "<<setw(10)<<access_cnt<<" | ");
//    STATE_PRINTN(setw(15)<<"Command bp"<<" : "<<setw(10)<<bp_cnt<<" | ");
//    STATE_PRINTN(setw(15)<<"Bp ratio"<<" : "<<setw(10)<<ratio<<" | "<<endl);
//    ratio = float(total_bp_cnt) * 100 / (total_bp_cnt + total_access_cnt);
//    STATE_PRINTN(setw(15)<<"Total access"<<" : "<<setw(10)<<total_access_cnt<<" | ");
//    STATE_PRINTN(setw(15)<<"Total bp"<<" : "<<setw(10)<<total_bp_cnt<<" | ");
//    STATE_PRINTN(setw(15)<<"Total bp ratio"<<" : "<<setw(10)<<ratio<<" | "<<endl);

    STATE_PRINTN("-------------------- RMW: Queue Statistics (Percentage/Cycle) --------------------------\n");
    uint32_t total = 0;
    size = rmw_que_cnt.size();

    for (uint32_t index = 0; index <= size; index ++) {
        STATE_PRINTN("--------");
    }
    STATE_PRINTN(endl);
    STATE_PRINTN(setw(7)<<"Qnum"<<"|");
    for (uint32_t index = 0; index < size; index++) {
        total += rmw_que_cnt.at(index);
        STATE_PRINTN(setw(7)<<index<<"|");
    }
    STATE_PRINTN(endl);
    STATE_PRINTN(setw(7)<<"Per"<<"|");
    for (uint32_t index = 0; index < size; index ++) {
        float cnt_dist_ratio = (float(rmw_que_cnt.at(index)) * 100) / total;
        STATE_PRINTN(setw(7)<<fixed<<setprecision(3)<<cnt_dist_ratio<<"|");
    }
    STATE_PRINTN(endl);
    STATE_PRINTN(setw(7)<<"Cycle"<<"|");
    for (uint32_t index = 0; index < size; index ++) {
        STATE_PRINTN(setw(7)<<fixed<<setprecision(3)<<rmw_que_cnt.at(index)<<"|");
    }
    STATE_PRINTN(endl);
    for (uint32_t index = 0; index <= size; index ++) {
        STATE_PRINTN("--------");
    }
    STATE_PRINTN(endl);

    for (uint32_t index = 0; index < size; index++) {
        rmw_que_cnt.at(index) = 0;
    }

//    task_cnt = 0;
//    access_cnt = 0;
//    bp_cnt = 0;

    //save the value of last time stataes
    pre_reads = reads;
    pre_bypass_reads = bypass_reads;
    pre_writes = writes;
    pre_bypass_writes = bypass_writes;
    pre_full_writes = full_writes;
    pre_mask_writes = mask_writes;
    pre_totals = totals;
    preWriteMergeInput = totalWriteMergeInput;
    preWriteMergePair = totalWriteMergePair;
    preWriteMergeUnpairedToRmw = totalWriteMergeUnpairedToRmw;
    preWriteMergeUnpairedDirect = totalWriteMergeUnpairedDirect;
    preWriteMergeBufferFull = totalWriteMergeBufferFull;
//    pre_address_conf_cnt = address_conf_cnt;
//    pre_read_cnt = read_cnt;
//    pre_write_cnt = write_cnt;
//    pre_mwrite_cnt = mwrite_cnt;


    //clear statistics
    STATE_PRINTN("========================================= END =========================================\n");
    STATE_PRINTN("\n");
}

}
