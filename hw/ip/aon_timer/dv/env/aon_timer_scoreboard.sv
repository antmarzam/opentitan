// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

class aon_timer_scoreboard extends cip_base_scoreboard #(
    .CFG_T(aon_timer_env_cfg),
    .RAL_T(aon_timer_reg_block),
    .COV_T(aon_timer_env_cov)
  );
  `uvm_component_utils(aon_timer_scoreboard)

  // local variables
  local bit  wkup_en;
  local bit  wkup_num_update_due;
  local bit [63:0] wkup_count;
  local uint prescaler;
  local bit [63:0] wkup_thold;

  local bit  wdog_en;
  local bit  wdog_regwen;
  local bit  wdog_pause_in_sleep;
  local bit  wdog_num_update_due;
  local uint wdog_count;
  local uint bark_thold;
  local uint bite_thold;

  local bit  wkup_cause;
  local bit [63:0] wkup_num;
  local bit [31:0] wdog_bark_num;
  local bit [31:0] wdog_bite_num;

  // expected values
  local bit intr_status_exp [2];
  local bit wdog_rst_req_exp = 0;

  typedef enum logic {
    WKUP = 1'b0,
    WDOG = 1'b1
  } timers_e;

  // TLM agent fifos

  // local queues to hold incoming packets pending comparison

<<<<<<< Updated upstream
=======
  string path_to_rtl = "tb.dut";

  event sys_reset_ev;

  // A write to wkup_cause(0x0) can be taken with some delay due to CDC crossing
  // it can happen the write is absorved at the same time the TB predicts a WDOG_intr.
  // Since it's difficult to predict what will happen in this case the TB expects
  // wkup_req_o will be 0 or the latest prediction
  int unsigned aon_clk_cycle = 0;
  int unsigned last_wkup_cause_write_aon_clk_cycle = 0;
>>>>>>> Stashed changes

  extern function new (string name="", uvm_component parent=null);
  extern task run_phase(uvm_phase phase);
  extern task monitor_interrupts();
  extern virtual task process_tl_access(tl_seq_item item, tl_channels_e channel, string ral_name);
  extern virtual task check_interrupt();
  extern virtual task compute_num_clks();
  extern virtual task wait_for_sleep();
  extern virtual task run_wkup_timer();
  extern virtual task run_wdog_bark_timer();
  extern virtual task run_wdog_bite_timer();

<<<<<<< Updated upstream
=======
  // The three collect_*_coverage below tasks run forever and can be safely killed at any time.
  //
  // collect_wkup_timer_coverage: Whenever the sample_coverage event fires, sample the 64-bit
  // wkup_count counter, threshold value and interrupt for wake_up_timer_thold_hit_cg covergroup.
  extern task collect_wkup_timer_coverage(ref event sample_coverage);
  // collect_wdog_bark_timer_coverage: Whenever the sample_coverage event fires, sample the 32-bit
  // wdog_count counter, threshold value and interrupt for watchdog_timer_bark_thold_hit_cg
  // covergroup.
  extern task collect_wdog_bark_timer_coverage(ref event sample_coverage);
  // collect_wdog_bite_timer_coverage: Whenever the sample_coverage event fires, sample the 32-bit
  // wdog_count counter, threshold value and interrupt for watchdog_timer_bite_thold_hit_cg
  // covergroup.
  extern task collect_wdog_bite_timer_coverage(ref event sample_coverage);

  // Sets both predicted timed register (WKUP/WDOG) values, captured from intr_state_exp.
  extern function void capture_timed_regs(output bfm_timed_regs_t state);
  // The argument passed by reference `state` stores the value predicted for intr_state for each
  // type of timer, `r` and tmr refer to the type of timer to predict for the timed and the expected
  // interrupts respectively
  extern function void capture_timed_regs_independently(ref bfm_timed_regs_t state,
                                                        aon_timer_intr_timed_regs::timed_reg_e r,
                                                        timers_e tmr);
  // Initialise the timed regs (called in the build_phase)
  extern function void init_timed_regs();
  // need separate wdog/wkup update functions in case they are called at the same time
  extern function void update_timed_regs_independently(aon_timer_intr_timed_regs::
                                                       timed_reg_e r, timers_e tmr);
  // Update the model side of the WKUP/WDOG(or both) timed registers
  extern function void update_timed_regs();
  extern function void update_timed_regs_wdog();
  extern function void update_timed_regs_wkup();

  // Convenience wrapper function to avoid calling explictly uvm_hdl_read multiple times
  extern function bit hdl_read_bit(string path);
  // Waits for AON signal to become 'value'
  extern task wait_for_aon_signal(string path, bit value);
  // Waits for the WE to rise and fall after a TL-UL write access
  // Does HDL reads to be in sync with when the values kick-in the RTL
  // It needs to be called the moment the TL-access occurs, otherwise the thread may hang if the WE
  // has risen already
  extern task wait_for_we_pulse(input string path);
  // Predicts register wkup_cause whenever the busy flag is 0 and blocks other parts of the TB to
  // predict it's value until an ongoing prediction finishes
  // In addition, the prediction can be delayed by setting 'wait_for_we' if the TB wants the
  // prediction to proceed after the wkup_cause write crosses the CDC boundary to be in sync with
  // the RTL.
  // The advice is to wrap a fork...join_none around this task to avoid blocking
  extern task predict_wkup_cause(bit wkup_cause, bit wait_for_we);
  // Predicts register intr_state whenever the busy flag is 0. It checks the prediction went through
  // and blocks other parts of the TB to predict its value until an ongoing prediction finishes
  // The advice is to wrap a fork...join_none around this task to avoid blocking
  extern task predict_intr_state(bit [1:0] pred_intr_state, bit field_only = 0, bit is_wkup = 0);

  // Update functions wait for the value to cross CDC boundaries by checking the write enable
  // before the TB starts counting
  extern task update_wdog_or_wkup_reg_timely(string path, timers_e timer_type);
  extern task update_wdog_count_timely();
  extern task update_wdog_bite_thold_timely();
  extern task update_wdog_bark_thold_timely();
  extern task update_wkup_count_lo_timely();
  extern task update_wkup_count_hi_timely();
  extern task update_wkup_thold_lo_timely();
  extern task update_wkup_thold_hi_timely();

  // Pushes predicted values for wdog/wkup_interrupt and monitors the value being updated in the
  // RTL advice is to wrap a fork... join_none around the task
  extern task wkup_intr_predicted_values(bit exp_wkup_intr);
  extern task wdog_intr_predicted_values(bit exp_wdog_intr);
  // Checks the predicted interrupt value is the current RTL value by backdoor reads
  extern task check_intr_value_propagated(timers_e timer_type);

  // Converts from timers_e to 'timed_reg_e'
  extern function aon_timer_intr_timed_regs::timed_reg_e timers_e2time_reg_e(timers_e timer_type);
  // Check the value of `intr_wdog_timer_bark/wkup_timer_expired_o' matched the one in the
  // intr_state register via using the timed registers.
  // intr_status_exp[WDOG] can't be used to compare since its value may have been wiped out by the
  // time the comparison is carried out
  extern function void check_aon_domain_interrupt(timers_e timer_type);
  // Returns either `predicted_wdog_intr_q` or `predicted_wkup_intr_q` by reference depending on the
  // timer type passed
  extern function void return_pred_intr_q(timers_e timer_type, ref bit pred_q[$]);
  // Checks the expected mirrored value against the value returned in the TL-UL read for intr_state
  extern function void check_intr_state_bit(timers_e timer_type, bit actual_value);

  // Convenience function to check if the wkup/wdog_ctrl.enable bits are set or not depending on
  // whether the enable argument is 0 or 1.
  extern task wait_for_wdog_enable_matching(bit enable);
  extern task wait_for_wkup_enable_matching(bit enable);

  extern virtual function void reset(string kind = "HARD");

>>>>>>> Stashed changes
endclass : aon_timer_scoreboard

function aon_timer_scoreboard::new (string name="", uvm_component parent=null);
  super.new(name, parent);
endfunction : new

task aon_timer_scoreboard::run_phase(uvm_phase phase);
  super.run_phase(phase);
  fork
    compute_num_clks();
    check_interrupt();
    monitor_interrupts();
  join_none
endtask : run_phase

task aon_timer_scoreboard::monitor_interrupts();
  forever begin
    @(cfg.aon_intr_vif.pins);
    // Sample interrupt pin coverage for interrupt pins
    if (cfg.en_cov) begin
      foreach (cfg.aon_intr_vif.pins[i]) begin
        cov.intr_pins_cg.sample(i, cfg.aon_intr_vif.sample_pin(.idx(i)));
      end
    end
  end
<<<<<<< Updated upstream
endtask : monitor_interrupts
=======
endtask : collect_fcov_from_rtl_interrupts

task aon_timer_scoreboard::predict_wkup_cause(bit wkup_cause, bit wait_for_we);
  fork
    begin : iso_fork
      fork
        begin : wkup_cause_timely
          // Blocks only if 'predicting_value' = 1
          ral.wkup_cause.wait_for_prediction();
          if (wait_for_we) begin
            wait_for_aon_signal(".u_reg.aon_wkup_cause_we", 1);
            ral.wkup_cause.predicting_value = 1;
            wait_for_aon_signal(".u_reg.aon_wkup_cause_we", 0);
          end // if (wait_for_we)
          else
            ral.wkup_cause.predicting_value = 1;

          // The predict method may fail if the register has just been accessed:
          // block here to avoid error
//          while (ral.wkup_cause.is_busy() == 1)
//            cfg.clk_rst_vif.wait_n_clks(1);

//          if (!ral.wkup_cause.predict(.value(wkup_cause), .kind(UVM_PREDICT_DIRECT)))
//            void'(csr.predict(.value(item.a_data), .kind(UVM_PREDICT_WRITE), .be(item.a_mask)));
//            if (!ral.wkup_cause.predict(.value(wkup_cause), .kind(UVM_PREDICT_WRITE), .be('hFFFF)))
          // using UVM_PREDICT_READ because other kinds don't seem to update mirrored value
            if (!ral.wkup_cause.predict(.value(wkup_cause), .kind(UVM_PREDICT_READ)))
            `uvm_fatal(`gfn, $sformatf("%s prediction failed", `gmv(ral.wkup_cause)))
            `uvm_info(`gfn, $sformatf("Updated predicted WKUP-CAUSE to 0x%0x", wkup_cause), UVM_DEBUG)
            $display("`gmv(ral.wkup_cause) = %0d",`gmv(ral.wkup_cause));
            if (wait_for_we)
            last_wkup_cause_write_aon_clk_cycle = aon_clk_cycle;
          `uvm_info(`gfn, "Triggering UVM event 'ral.wkup_cause.value_predicted_ev'", UVM_DEBUG)
          ral.wkup_cause.value_predicted_ev.trigger();
          ral.wkup_cause.predicting_value = 0;
        end : wkup_cause_timely
        begin : reset_kill
          wait (under_reset);
        end : reset_kill
      join_any
      disable fork;
    end : iso_fork
  join
endtask : predict_wkup_cause

// Convenience function to avoid repeating same boilerplate code
// It takes a path relative to the RTL and appends the RTL instance to it
function bit aon_timer_scoreboard::hdl_read_bit(string path);
  bit hdl_bit;
  if (! uvm_hdl_read({path_to_rtl,path}, hdl_bit))
    `uvm_error (`gfn, $sformatf("HDL Read from %s failed", path))
  return hdl_bit;
endfunction : hdl_read_bit

// Does a uvm_hdl_read until the read value becomes 0/1 in the AON domain
task aon_timer_scoreboard::wait_for_aon_signal(string path, bit value);
  bit actual_value;
  do begin
    actual_value = hdl_read_bit(path);
    if (actual_value !== value)
      cfg.aon_clk_rst_vif.wait_clks(1);
  end while (actual_value !== value);
endtask

task aon_timer_scoreboard::wait_for_we_pulse(input string path);
  wait_for_aon_signal(path, 1);
  wait_for_aon_signal(path, 0);
endtask : wait_for_we_pulse

// Update functions waits for the value to cross CDC boundaries before the TB starts counting
task aon_timer_scoreboard::update_wdog_or_wkup_reg_timely(string path, timers_e timer_type);
  fork
    begin: iso_fork
      fork
        begin : hdl_read_wdog_wkup_reg_we
          wait_for_we_pulse(path);
          // Update once the changes have kicked in.
          case(timer_type)
            WKUP: wkup_num_update_due = 1;
            WDOG: wdog_num_update_due = 1;
            default: `uvm_fatal(`gfn, $sformatf("Incorrect timer type:%0s", timer_type.name()))
          endcase
        end : hdl_read_wdog_wkup_reg_we
        begin : reset_kill
          wait (under_reset);
        end : reset_kill
      join_any
      disable fork;
    end : iso_fork
  join
endtask : update_wdog_or_wkup_reg_timely

// Update functions waits for the value to cross CDC boundaries before the TB starts counting
task aon_timer_scoreboard::update_wdog_count_timely();
  update_wdog_or_wkup_reg_timely(.path(".u_reg.aon_wdog_count_we"), .timer_type(WDOG));
endtask : update_wdog_count_timely

task aon_timer_scoreboard::update_wdog_bite_thold_timely();
  update_wdog_or_wkup_reg_timely(.path(".u_reg.aon_wdog_bite_thold_gated_we"), .timer_type(WDOG));
endtask : update_wdog_bite_thold_timely

task aon_timer_scoreboard::update_wdog_bark_thold_timely();
  update_wdog_or_wkup_reg_timely(.path(".u_reg.aon_wdog_bark_thold_gated_we"), .timer_type(WDOG));
endtask : update_wdog_bark_thold_timely

task aon_timer_scoreboard::update_wkup_count_lo_timely();
  update_wdog_or_wkup_reg_timely(.path(".u_reg.aon_wkup_count_lo_we"), .timer_type(WKUP));
endtask : update_wkup_count_lo_timely

task aon_timer_scoreboard::update_wkup_count_hi_timely();
  update_wdog_or_wkup_reg_timely(.path(".u_reg.aon_wkup_count_hi_we"), .timer_type(WKUP)); 
endtask : update_wkup_count_hi_timely

task aon_timer_scoreboard::update_wkup_thold_lo_timely();
  update_wdog_or_wkup_reg_timely(.path(".u_reg.aon_wkup_thold_lo_we"), .timer_type(WKUP));
endtask : update_wkup_thold_lo_timely

task aon_timer_scoreboard::update_wkup_thold_hi_timely();
  update_wdog_or_wkup_reg_timely(.path(".u_reg.aon_wkup_thold_hi_we"), .timer_type(WKUP));
endtask : update_wkup_thold_hi_timely

task aon_timer_scoreboard::predict_intr_state(bit [1:0] pred_intr_state, bit field_only = 0, bit is_wkup=0);
  `uvm_info(`gfn, $sformatf("%m - pred_intr_state = 0x%0x",pred_intr_state), UVM_DEBUG)
  fork
    begin: iso_fork
      fork
        begin : intr_state_timely
          ral.intr_state.predicting_value = 1;

          do 
          begin
            if (ongoing_intr_state_read)
              cfg.clk_rst_vif.wait_n_clks(1);

          end
          while (ongoing_intr_state_read);
          // if (ongoing_intr_state_read) begin
          //   while (ongoing_intr_state_read) begin
          //     cfg.clk_rst_vif.wait_n_clks(1);
          //   end
          // end 
          // else // Update on the neg-edge in case there's an access finishing now
          //   cfg.clk_rst_vif.wait_n_clks(1);

          //          while (ral.intr_state.is_busy() == 1)
//            cfg.clk_rst_vif.wait_n_clks(1);
//          if (!ral.intr_state.predict(.value(pred_intr_state), .kind(UVM_PREDICT_DIRECT)))
//          if (!ral.intr_state.predict(.value(pred_intr_state), .kind(UVM_PREDICT_WRITE), .be('hff)))
          // switched to uvm_predict_read because otherwise the mirrored value doesn't update
          if (!field_only) begin
            if (!ral.intr_state.predict(.value(pred_intr_state), .kind(UVM_PREDICT_READ) ))
            `uvm_fatal(`gfn, $sformatf("%s prediction failed", ral.intr_state.get_name()))
          end 
          else begin
            if (is_wkup) begin
              if (!ral.intr_state.wkup_timer_expired.predict(.value(pred_intr_state[WKUP]), .kind(UVM_PREDICT_READ) ))
                `uvm_fatal(`gfn, $sformatf("%s prediction failed", ral.intr_state.wkup_timer_expired.get_name()))
            end
            else begin
              if (!ral.intr_state.wdog_timer_bark.predict(.value(pred_intr_state[WDOG]), .kind(UVM_PREDICT_READ) ))
                `uvm_fatal(`gfn, $sformatf("%s prediction failed", ral.intr_state.wdog_timer_bark.get_name()))
            end
          end
          $display("intr_state post-predict: pred_intr_state = 0x%0x",pred_intr_state);
          $display("`gmv(ral.intr_state) = 0x%0x",`gmv(ral.intr_state));
          `uvm_info(`gfn, "Triggering UVM event 'ral.intr_state.value_predicted_ev'", UVM_DEBUG)
          ral.intr_state.value_predicted_ev.trigger();
          ral.intr_state.predicting_value  = 0;
        end : intr_state_timely
        begin : reset_kill
          wait (under_reset);
          ral.intr_state.predicting_value  = 0;
        end : reset_kill
      join_any
      disable fork;
    end : iso_fork
  join
endtask : predict_intr_state

task aon_timer_scoreboard::wkup_intr_predicted_values(bit exp_wkup_intr);
  static int unsigned last_cycle_count = 0;

  if (last_cycle_count != timed_regs.time_now) begin
    `uvm_info(`gfn, $sformatf("%m - Predicted wkup_intr = 0x%0x", exp_wkup_intr), UVM_DEBUG)
    predicted_wkup_intr_q.push_back(exp_wkup_intr);
    last_cycle_count = timed_regs.time_now;
    fork
      begin: iso_fork
        fork
          begin : wait_values_to_propagate
            // do backdoor read and delete values no longer valid.
            check_intr_value_propagated(WKUP);
          end : wait_values_to_propagate
          begin : reset_kill
            wait (under_reset);
          end : reset_kill
        join_any
        disable fork;
      end : iso_fork
    join
  end
endtask : wkup_intr_predicted_values

task aon_timer_scoreboard::wdog_intr_predicted_values(bit exp_wdog_intr);
  static int unsigned last_cycle_count = 0;

  if (last_cycle_count != timed_regs.time_now) begin
    `uvm_info(`gfn, $sformatf("%m - Predicted wdog_intr = 0x%0x", exp_wdog_intr), UVM_DEBUG)
    predicted_wdog_intr_q.push_back(exp_wdog_intr);
    last_cycle_count = timed_regs.time_now;
    fork
      begin: iso_fork
        fork
          begin : wait_values_to_propagate
            // do backdoor read and delete values no longer valid.
            check_intr_value_propagated(WDOG);
          end : wait_values_to_propagate
          begin : reset_kill
            wait (under_reset);
          end : reset_kill
        join_any
        disable fork;
      end : iso_fork
    join
  end
endtask : wdog_intr_predicted_values

task aon_timer_scoreboard::check_intr_value_propagated(timers_e timer_type);
  int unsigned idx; // Idx to the latest prediction
  bit          exp_value;
  bit          act_data;
  bit          value_matched;

  case(timer_type)
    WKUP: begin
      if (predicted_wkup_intr_q.size == 0)
        `uvm_fatal(`gfn, "'predicted_wkup_intr_q' Queue is empty")
      idx = predicted_wkup_intr_q.size -1;
      // Synchronise with the next neg-edge
      cfg.clk_rst_vif.wait_n_clks(1);
      do begin
        csr_rd(.ptr(ral.intr_state.wkup_timer_expired),   .value(act_data), .backdoor(1));
        if (predicted_wkup_intr_q[idx] == act_data && ongoing_intr_state_read == 0) begin
          value_matched = 1;
          // Remove all the other predictions no longer valid:
          for (int i = 0; i < idx; i++)
            void'(predicted_wkup_intr_q.pop_front());
        end
        else //negedge to avoid a race condition
          cfg.clk_rst_vif.wait_n_clks(1);
      end
      while (value_matched == 0);
    end
    WDOG: begin
      if (predicted_wdog_intr_q.size == 0)
        `uvm_fatal(`gfn, "'predicted_wdog_intr_q' Queue is empty")
      idx = predicted_wdog_intr_q.size -1;
      // Synchronise with the next neg-edge
      cfg.clk_rst_vif.wait_n_clks(1);
      do begin
        csr_rd(.ptr(ral.intr_state.wdog_timer_bark),   .value(act_data), .backdoor(1));
        if (predicted_wdog_intr_q[idx] == act_data && ongoing_intr_state_read == 0) begin
          value_matched = 1;
          // Remove all the other predictions no longer valid:
          for (int i = 0; i < idx; i++)
            void'(predicted_wdog_intr_q.pop_front());
        end
        else
          cfg.clk_rst_vif.wait_n_clks(1);
      end
      while (value_matched == 0);

    end
  endcase
endtask : check_intr_value_propagated
>>>>>>> Stashed changes

task aon_timer_scoreboard::process_tl_access(tl_seq_item item, tl_channels_e channel,
                                             string ral_name);
  uvm_reg csr;
  bit     do_read_check   = 1'b1;
  bit     write           = item.is_write();
  uvm_reg_addr_t csr_addr = cfg.ral_models[ral_name].get_word_aligned_addr(item.a_addr);

  bit     addr_phase_read   = (!write && channel == AddrChannel);
  bit     addr_phase_write  = (write && channel == AddrChannel);
  bit     data_phase_read   = (!write && channel == DataChannel);
  bit     data_phase_write  = (write && channel == DataChannel);

  // if access was to a valid csr, get the csr handle
  if (csr_addr inside {cfg.ral_models[ral_name].csr_addrs}) begin
    csr = cfg.ral_models[ral_name].default_map.get_reg_by_offset(csr_addr);
    `DV_CHECK_NE_FATAL(csr, null)
  end
  else begin
    `uvm_fatal(`gfn, $sformatf("Access unexpected addr 0x%0h", csr_addr))
  end

  // if incoming access is a write to a valid csr, then make updates right away
  if (addr_phase_write) begin
    void'(csr.predict(.value(item.a_data), .kind(UVM_PREDICT_WRITE), .be(item.a_mask)));
    if (cfg.en_cov) begin
      //Sample configuration coverage
      cov.timer_cfg_cg.sample(prescaler, bark_thold, bite_thold,
                              wkup_thold, wdog_regwen, wdog_pause_in_sleep, wkup_cause);
    end
  end

  // process the csr req
  // for write, update local variable and fifo at address phase
  // for read, update predication at address phase and compare at data phase
  case (csr.get_name())
    // add individual case item for each csr
    "intr_state": begin
      do_read_check = 1'b0;
<<<<<<< Updated upstream
      if (data_phase_write) begin
        uint intr_state_val = item.a_data;
        if (intr_state_val[WKUP]) intr_status_exp[WKUP] = 1'b0;
        if (intr_state_val[WDOG]) intr_status_exp[WDOG] = 1'b0;
=======

      if (addr_phase_write) begin
        uint intr_state_val = item.a_mask[0] ? item.a_data : 0;
        bit update_prediction;
        if (intr_state_val[WKUP]) begin
          intr_status_exp[WKUP] = 1'b0;
          update_prediction = 1;
          fork wkup_intr_predicted_values(intr_status_exp[WKUP]); join_none
        end
        if (intr_state_val[WDOG]) begin
          intr_status_exp[WDOG] = 1'b0;
          update_prediction = 1;
          fork wdog_intr_predicted_values(intr_status_exp[WDOG]); join_none
        end

        if (update_prediction) begin
          uvm_reg_data_t predicted_intr_status;
          predicted_intr_status[WDOG] = intr_status_exp[WDOG];
          predicted_intr_status[WKUP] = intr_status_exp[WKUP];
          `uvm_info(`gfn, $sformatf("Updated intr_status_exp = 0x%0x",intr_status_exp),
                    UVM_DEBUG)
          `uvm_info("RM_ME", $sformatf("[INTR_STATE WR] - Predicting intr_state - 0x%0x",predicted_intr_status), UVM_DEBUG)
          fork predict_intr_state(predicted_intr_status); join_none

          // The timed registers predictions need to be independent of each other
          if (intr_state_val[WDOG])
            update_timed_regs_wdog();
          if (intr_state_val[WKUP])
            update_timed_regs_wkup();
        end
      end
      else if (data_phase_read) begin
        check_intr_state_bit(.timer_type(WKUP), .actual_value(item.d_data[WKUP]));
        check_intr_state_bit(.timer_type(WDOG), .actual_value(item.d_data[WDOG]));
>>>>>>> Stashed changes
      end
      // INTR_EN register does not exists in AON timer because the interrupts are
      // enabled as long as timers are enabled.
      if (cfg.en_cov && data_phase_read) begin
        cov.intr_cg.sample(WKUP, wkup_en, item.d_data[WKUP]);
        cov.intr_cg.sample(WDOG, wdog_en, item.d_data[WDOG]);
      end
    end
    "wkup_ctrl": begin
      prescaler = get_reg_fld_mirror_value(ral, csr.get_name(), "prescaler");
      wkup_en   = get_reg_fld_mirror_value(ral, csr.get_name(), "enable");
      if (data_phase_write) wkup_num_update_due = 1;
    end
    "wkup_cause": begin
      wkup_cause = csr.get_mirrored_value();
      intr_status_exp[WKUP] = item.a_data;
    end
    "wkup_count_lo": begin
      wkup_count[31:0] =  csr.get_mirrored_value();
      if (data_phase_write) wkup_num_update_due = 1;
    end
    "wkup_count_hi": begin
      wkup_count[63:32] =  csr.get_mirrored_value();
      if (data_phase_write) wkup_num_update_due = 1;
    end
    "wkup_thold_lo": begin
      wkup_thold[31:0] =  csr.get_mirrored_value();
      if (data_phase_write) wkup_num_update_due = 1;
    end
    "wkup_thold_hi": begin
      wkup_thold[63:32] =  csr.get_mirrored_value();
      if (data_phase_write) wkup_num_update_due = 1;
    end
    "wdog_ctrl": begin
      wdog_en = get_reg_fld_mirror_value(ral, csr.get_name(), "enable");
      wdog_pause_in_sleep = get_reg_fld_mirror_value(ral, csr.get_name(), "pause_in_sleep");
    end
    "wdog_count": begin
      wdog_count =  csr.get_mirrored_value();
      if (data_phase_write) wdog_num_update_due = 1;
    end
    "wdog_regwen": begin
      wdog_regwen =  csr.get_mirrored_value();
    end
    "wdog_bark_thold": begin
      bark_thold =  csr.get_mirrored_value();
      if (data_phase_write) wdog_num_update_due = 1;
    end
    "wdog_bite_thold": begin
      bite_thold =  csr.get_mirrored_value();
      if (data_phase_write) wdog_num_update_due = 1;
    end
    "intr_test": begin
      uint intr_test_val = item.a_data;
<<<<<<< Updated upstream
      if (intr_test_val[WKUP]) intr_status_exp[WKUP] = 1'b1;
      if (intr_test_val[WDOG]) intr_status_exp[WDOG] = 1'b1;
      if (cfg.en_cov) begin
        cov.intr_test_cg.sample(WKUP, intr_test_val[WKUP],
                                wkup_en, intr_status_exp[WKUP]);
        cov.intr_test_cg.sample(WDOG, intr_test_val[WDOG],
                                wdog_en, intr_status_exp[WDOG]);
      end
    end
=======
      if (addr_phase_write) begin
        // The timed registers predictions need to be independent of each other
        if (intr_test_val[WKUP]) begin
          intr_status_exp[WKUP] = 1'b1;
          `uvm_info(`gfn, "Setting intr_status_exp[WKUP]", UVM_DEBUG)
          update_timed_regs_wkup();
          fork wkup_intr_predicted_values(intr_status_exp[WKUP]); join_none
        end
        if (intr_test_val[WDOG]) begin
          intr_status_exp[WDOG] = 1'b1;
          `uvm_info(`gfn, "Setting intr_status_exp[WDOG]", UVM_DEBUG)
          update_timed_regs_wdog();
          fork wdog_intr_predicted_values(intr_status_exp[WDOG]); join_none
        end

        if (intr_test_val[WDOG] | intr_test_val[WKUP]) begin
          `uvm_info(`gfn, "Updating intr_state due to intr_test write", UVM_DEBUG)
          // The call to intr_state.busy() may block and hence we update when possible
          `uvm_info("RM_ME", $sformatf("[INTR_TEST] - Predicting intr_state - 0x%0x",intr_status_exp), UVM_DEBUG)
          fork predict_intr_state(intr_status_exp); join_none
        end
        if (cfg.en_cov) begin
          cov.intr_test_cg.sample(WKUP, intr_test_val[WKUP],
                                  wkup_en, intr_status_exp[WKUP]);
          cov.intr_test_cg.sample(WDOG, intr_test_val[WDOG],
                                  wdog_en, intr_status_exp[WDOG]);
        end
      end // if (addr_phase_write)
    end // case: "intr_test"
>>>>>>> Stashed changes
    default: begin
      // No other special behaviour for writes
    end
  endcase

  // On reads, if do_read_check, is set, then check mirrored_value against item.d_data
  if (data_phase_read) begin
    if (do_read_check) begin
      `DV_CHECK_EQ(csr.get_mirrored_value(), item.d_data,
                   $sformatf("reg name: %0s", csr.get_full_name()))
    end
    void'(csr.predict(.value(item.d_data), .kind(UVM_PREDICT_READ)));
  end
endtask : process_tl_access

// Task : check_interrupt
// wait for expected # of clocks and check for interrupt state reg and pin
task aon_timer_scoreboard::check_interrupt();
  forever begin
    wait (!under_reset);

    fork : isolation_fork
      fork
        wait (under_reset);
        run_wkup_timer();
        run_wdog_bark_timer();
        run_wdog_bite_timer();
      join_any

      // run_wkup_timer and run_wdog_timer never return so if we've got here then we've gone into
      // reset. Kill the two timer processes then go around and wait until we come out of reset
      // again.
      disable fork;
    join
  end
endtask : check_interrupt

task aon_timer_scoreboard::compute_num_clks();
  forever begin : compute_num_clks
    // calculate number of clocks required to have interrupt from wkup
    @(wkup_num_update_due or wdog_num_update_due);
    wait(!under_reset);
    if (wkup_num_update_due) begin
      if (wkup_count <= wkup_thold) begin
        wkup_num = ((wkup_thold - wkup_count + 1) * (prescaler + 1));
      end
      else begin
        wkup_num = 0;
      end
      `uvm_info(`gfn, $sformatf("Calculated WKUP_NUM: %d", wkup_num), UVM_HIGH)
    end
    if (wdog_num_update_due) begin
      // calculate wdog bark
      if (wdog_count < bark_thold) begin
        wdog_bark_num = bark_thold - wdog_count;
      end
      else begin
        wdog_bark_num = 0;
      end
      `uvm_info(`gfn, $sformatf("Calculated wdog_bark_num: %d", wdog_bark_num), UVM_HIGH)
      if (wdog_count < bite_thold) begin
        wdog_bite_num = bite_thold - wdog_count;
      end
      else begin
        wdog_bite_num = 0;
      end
      `uvm_info(`gfn, $sformatf("Calculated wdog_bite_num: %d", wdog_bite_num), UVM_HIGH)
    end
    wkup_num_update_due = 0;
    wdog_num_update_due = 0;
  end // compute_num_clks
endtask : compute_num_clks

task aon_timer_scoreboard::wait_for_sleep();
  wait ( !(wdog_pause_in_sleep & cfg.sleep_vif.sample_pin()));
endtask : wait_for_sleep

task aon_timer_scoreboard::run_wkup_timer();
  event sample_coverage;
  forever begin
<<<<<<< Updated upstream
    wait (wkup_en);
=======
    wait (cfg.clk_rst_vif.rst_n === 1);
    wait_for_wkup_enable_matching(.enable(1));
    `uvm_info(`gfn, "WKUP ctrl.enable signal is set", UVM_DEBUG)

>>>>>>> Stashed changes
    fork
      begin
        forever begin
          @(sample_coverage.triggered);
          if (cfg.en_cov) begin
            bit [63:0] rtl_count;
            //reading RTL value since TB doesn't keep track of count
            csr_rd(.ptr(ral.wkup_count_lo), .value(rtl_count[31:0]), .backdoor(1));
            csr_rd(.ptr(ral.wkup_count_lo), .value(rtl_count[63:32]), .backdoor(1));
            cov.wake_up_timer_thold_hit_cg.sample(intr_status_exp[WKUP],
                                                  wkup_thold,
                                                  rtl_count);
          end
        end
      end
      begin
        // trying to count how many cycles we need to count
        uint count = 0;
        // It takes 4 aon clks from the write enabling the watchdog to take effect due to the CDC
        // logic.
        cfg.aon_clk_rst_vif.wait_clks(4);
        while (count < wkup_num) begin
          cfg.aon_clk_rst_vif.wait_clks(1);
          // reset the cycle counter when we update the cycle count needed
          count = wkup_num_update_due ? 0 : (count + 1);
          `uvm_info(`gfn, $sformatf("WKUP Timer count: %d", count), UVM_HIGH)
          -> sample_coverage;
        end
        `uvm_info(`gfn, $sformatf("WKUP Timer expired check for interrupts"), UVM_HIGH)
<<<<<<< Updated upstream
        intr_status_exp[WKUP] = 1'b1;
        -> sample_coverage;

        // Interrupt should happen N+1 clock ticks after count == wkup_num.
        cfg.aon_clk_rst_vif.wait_clks(prescaler+1);
        // Wait for 2 extra cycles in AON clock domain to account for CDC randomization delay
        // since wakeup interrupt is synchronized to AON clock domain
        if (cfg.en_dv_cdc) begin
          cfg.aon_clk_rst_vif.wait_clks(2);
        end
        // Wait a further 5 clocks for the interrupt to propagate through logic in the clk domain
        // to become visible on the top-level pins.
        cfg.clk_rst_vif.wait_clks(5);
        // Check interrupt pin
        `DV_CHECK_CASE_EQ(intr_status_exp[WKUP],
                          cfg.intr_vif.sample_pin(.idx(WKUP)))
=======

        // Using a local interrupt flag in case 'intr_status_exp[WKUP]' changes
        // due to TL-UL accesses
/*         local_interrupt = 1;
        intr_status_exp[WKUP] = local_interrupt;

        wkup_cause = `gmv(ral.wkup_cause);
        wkup_cause |= intr_status_exp[WKUP];
        `uvm_info(`gfn, $sformatf("Predicting wkup_cause = 0x%0x",wkup_cause), UVM_DEBUG)
        fork predict_wkup_cause(.wkup_cause(wkup_cause), .wait_for_we(0)); join_none


        -> sample_coverage;
 */
        // TODO: fix after RTL fix (RTL currently doesn't reset the prescale_count_q when
        // wkup_ctrl is written
        if (! uvm_hdl_read({path_to_rtl,".u_core.prescale_count_q"}, preloaded_prescaler_value))
          `uvm_error (`gfn, "HDL Read from tb.dut.u_core.prescale_count_q")

        // Interrupt should happen N+1 clock ticks after count == wkup_num.
        // Oriignal delay:
        //          cfg.aon_clk_rst_vif.wait_clks(prescaler+1);
        $display("prescaler = %0d",prescaler);
        if (prescaler==0) begin
          cfg.aon_clk_rst_vif.wait_clks(1);
        end
        // TODO: applies to else_if/ese fix after RTL fix (RTL currently doesn't reset the
        // prescale_count_q when wkup_ctrl is written
        else if ((prescaler + 1) > preloaded_prescaler_value) begin
          cfg.aon_clk_rst_vif.wait_clks((prescaler + 1 - preloaded_prescaler_value));
        end
        else begin
          bit wkup_incr;
          bit [63:0] wkup_count, wkup_thold;
          // Note: This is a bug/feature in the RTL. If the RTL was counting with
          // wkup_ctrl.enable=1, prescaler = X, and by the time SW sends a wup_ctrl.enable=0
          // the internal prescaler counter `prescale_count_q` has a value of Y, if once we enable
          // the wkup_ctrl.enable=1, the value written to wkup_ctrl.prescaler is lower than the
          // value already in `prescaler_count_q`, the prescaler will have to overflow before the
          // new prescaler value becomes effective

          // RTL's `prescale_count_q` may have a "pre-counted" value in it, and unless
          // there's a reset, the interrupt won't propagate to the output until:
          // prescale_count_q == reg2hw_i.wkup_ctrl.prescaler.q (In other words until
          // wkup_incr is true

          // Read threshold, and count and see if we've incremented
          do begin
            csr_rd(.ptr(ral.wkup_count_lo), .value(wkup_count[31:0]), .backdoor(1));
            csr_rd(.ptr(ral.wkup_count_hi), .value(wkup_count[63:32]), .backdoor(1));

            csr_rd(.ptr(ral.wkup_thold_lo), .value(wkup_thold[31:0]), .backdoor(1));
            csr_rd(.ptr(ral.wkup_thold_hi), .value(wkup_thold[63:32]), .backdoor(1));

            if (wkup_count < wkup_thold)
              cfg.aon_clk_rst_vif.wait_clks(1);
          end while (wkup_count < wkup_thold);

          do begin
            if (! uvm_hdl_read({path_to_rtl,".u_core.wkup_incr"},wkup_incr))
              `uvm_error (`gfn, "HDL Read from tb.dut.u_core.wkup_incr failed")
            if (!wkup_incr)
              cfg.aon_clk_rst_vif.wait_clks(1);

          end while(!wkup_incr);
          // Extra delay to allow the interrupt to propagate
          cfg.aon_clk_rst_vif.wait_clks(1);
        end // else: !if((prescaler + 1) > preloaded_prescaler_value)
        `uvm_info("RM_ME_ANTONIO", "AFTER AON DELAY", UVM_DEBUG)

        // TODO: do not set interrupts after the pres-caler delay (code from after count moved here):
        // Using a local interrupt flag in case 'intr_status_exp[WKUP]' changes
        // due to TL-UL accesses
        local_interrupt = 1;
        intr_status_exp[WKUP] = local_interrupt;

        wkup_cause = `gmv(ral.wkup_cause);
        wkup_cause |= intr_status_exp[WKUP];
        `uvm_info(`gfn, $sformatf("Predicting wkup_cause = 0x%0x",wkup_cause), UVM_DEBUG)
        fork predict_wkup_cause(.wkup_cause(wkup_cause), .wait_for_we(0)); join_none


        -> sample_coverage;

        // WKUP interrupt ('intr_status_exp[WKUP]')  may have been cleared by the time we reach
        // the AON delay, so we overwrite it and revert it to its original value after
        // updating timed regs
        local_interrupt = intr_status_exp[WKUP];
        intr_status_exp[WKUP] = 1;

        // TODO: now predictions are inmediate, so we need to call predict at the right time
        // Push the local_interrupt
        fork wkup_intr_predicted_values(intr_status_exp[WKUP]); join_none

        local_intr_exp = intr_status_exp;
//        fork predict_intr_state(local_intr_exp); join_none

//        update_timed_regs_wkup();
        // Restoring the value for 'intr_status_exp[WKUP]'
        intr_status_exp[WKUP] = local_interrupt;

        // Push the local_interrupt
//        fork wkup_intr_predicted_values(intr_status_exp[WKUP]); join_none

        $display("before CLK delay: gmv: ral.intr_state = 0x%0x",`gmv(ral.intr_state));
        `uvm_info("RM_ME", "ANTONIP: BEFORE DELAY", UVM_DEBUG)
        // The register 'intr_state' will be updated 4 clock cycles after the counter > threshold
        // Otherwise, the TB may be predicting the value too early and mess up comparisons

        // Synchronising to the pos-edge before counting edges in case the current cycle is already ongoing
        // to avoid updating the mirrored value too early
        @(cfg.clk_rst_vif.cb);
        cfg.clk_rst_vif.wait_n_clks(3);
//        cfg.clk_rst_vif.wait_clks(3);
        if (hdl_read_bit(".hw2reg.intr_state.wkup_timer_expired.de") == 1)
          cfg.clk_rst_vif.wait_n_clks(1); // extra cycle to update intr_state in time

        local_interrupt = intr_status_exp[WKUP];
        intr_status_exp[WKUP] = 1;
        local_intr_exp = intr_status_exp;
        update_timed_regs_wkup();
  
        $display("ral.intr_state = 0x%0x",`gmv(ral.intr_state));
        `uvm_info("RM_ME", $sformatf("[WKUP] - Predicting intr_state - 0x%0x",local_intr_exp), UVM_DEBUG)
//        fork predict_intr_state(local_intr_exp); join_none
        fork predict_intr_state(.pred_intr_state(local_intr_exp), .field_only(1), .is_wkup(1)); join_none
        // Restoring the value for 'intr_status_exp[WKUP]'
        intr_status_exp[WKUP] = local_interrupt;


        // Wait a further 5 clocks for the interrupt to propagate through logic in the clk domain
        // to become visible on the top-level pins.
// TODO: 4 out of the 5 clk cycles below had been consumed above
//        cfg.clk_rst_vif.wait_clks(5);


        cfg.clk_rst_vif.wait_clks(2);
        // attempted to fix with below
        //        cfg.clk_rst_vif.wait_clks(1);
        check_aon_domain_interrupt(.timer_type(WKUP));
        $display("cfg.under_reset = %0d",cfg.under_reset);
        $display("cfg.aon_clk_rst_vif.rst_n = %0d",cfg.aon_clk_rst_vif.rst_n);
>>>>>>> Stashed changes
        // Check wakeup pin
        `DV_CHECK_CASE_EQ(1,
                          cfg.aon_intr_vif.sample_pin(.idx(1)))
        `uvm_info(`gfn, $sformatf("WKUP Timer check passed."), UVM_HIGH)
      end
      begin
        wait (!wkup_en || !cfg.aon_clk_rst_vif.rst_n);
<<<<<<< Updated upstream
=======
        wkup_en = 0;
        $display("WKUP thread: after rst");
        $display("cfg.aon_clk_rst_vif.rst_n = %0d",cfg.aon_clk_rst_vif.rst_n);
        wait_for_wkup_enable_matching(.enable(0));
>>>>>>> Stashed changes
        `uvm_info(`gfn, $sformatf("WKUP Timer disabled, quit scoring"), UVM_HIGH)
        wkup_en = 0;
      end
    join_any
    disable fork;
  end
endtask : run_wkup_timer

<<<<<<< Updated upstream
=======
function aon_timer_intr_timed_regs::timed_reg_e aon_timer_scoreboard::timers_e2time_reg_e(timers_e
                                                                                          timer_type
                                                                                          );
  aon_timer_intr_timed_regs::timed_reg_e return_value;

  if (timer_type == WDOG)
    return_value = aon_timer_intr_timed_regs::TimedIntrStateWdogBark;
  else if (timer_type == WKUP)
    return_value = aon_timer_intr_timed_regs::TimedIntrStateWkupExpired;
  else
    `uvm_fatal(`gfn, $sformatf("Wrong timer index passed (%s)",timer_type.name))

  return return_value;
endfunction

function void aon_timer_scoreboard::check_aon_domain_interrupt(timers_e timer_type);
  bit cdc_reg_compared = 0;
  aon_timer_intr_timed_regs::timed_reg_e reg_idx = timers_e2time_reg_e(timer_type);

  for(int i=0; i < timed_regs.timed[reg_idx].fields.size(); i++) begin
//    if (timed_regs.timed[reg_idx].fields[i].pred_valid) begin
      for(int i=0; i < timed_regs.timed[reg_idx].fields.size(); i++) begin
        $display("timed_regs.timed[reg_idx].fields[i].pred_latest.val_new = %0d",timed_regs.timed[reg_idx].fields[i].pred_latest.val_new);
        $display("timed_regs.timed[reg_idx].fields[i].pred_valid = %0d",timed_regs.timed[reg_idx].fields[i].pred_valid);
    end 
//  end
  end 
  for(int i=0; i < timed_regs.timed[reg_idx].fields.size(); i++) begin
    if (timed_regs.timed[reg_idx].fields[i].pred_valid) begin
      `DV_CHECK_CASE_EQ(timed_regs.timed[reg_idx].fields[i].pred_latest.val_new,
                       cfg.intr_vif.sample_pin(.idx(timer_type)))
      cdc_reg_compared = 1;
    end
  end
          $display("cfg.under_reset = %0d",cfg.under_reset);
          $display("cfg.clk_rst_vif.rst_n = %0d",cfg.clk_rst_vif.rst_n);
  if (!cfg.under_reset) begin
    if (!cdc_reg_compared) begin
      `uvm_fatal(`gfn, $sformatf({"TB failed to compare sys-clock %s  interrupt pin - ",
                                  "likely due to the CDC timed register not having a prediction"},
                                  timer_type.name))
    end
    `uvm_info(`gfn, $sformatf("'intr_%0s=0x%0x' comparison matched",
                              timer_type == WKUP ? "wkup_timer_expired_o" : "wdog_timer_bark_o",
                              cfg.intr_vif.sample_pin(.idx(timer_type))), UVM_DEBUG)
  end
endfunction : check_aon_domain_interrupt

function void aon_timer_scoreboard::return_pred_intr_q(timers_e timer_type, ref bit pred_q[$]);
  if (timer_type == WDOG)
    pred_q = predicted_wdog_intr_q;
  else if (timer_type == WKUP)
    pred_q = predicted_wkup_intr_q;
  else
    `uvm_fatal(`gfn, $sformatf("Wrong timer index passed (%s)",timer_type.name))

  if (pred_q.size == 0)
    `uvm_fatal(`gfn, {"'predicted_", timer_type == WKUP ? "wkup" : "wdog", "_intr_q.size = 0'"})
endfunction

function void aon_timer_scoreboard::check_intr_state_bit(timers_e timer_type, bit actual_value);
  bit cdc_reg_compared = 0;
  bit pred_q[$];

  return_pred_intr_q(timer_type, pred_q);
  `DV_CHECK_CASE_EQ(pred_q[0], // Comparing against the oldes predicted value
                    actual_value)

  `uvm_info(`gfn, $sformatf("'intr_state.%0s=0x%0x' comparison matched",
                            timer_type.name, actual_value), UVM_DEBUG)
endfunction : check_intr_state_bit

task aon_timer_scoreboard::collect_wdog_bark_timer_coverage(ref event sample_coverage);
  forever begin
    @ (sample_coverage);
    if (cfg.en_cov) begin
      bit [31:0] rtl_count;
      //reading RTL value since TB doesn't keep track of count
      csr_rd(.ptr(ral.wdog_count), .value(rtl_count), .backdoor(1));
      cov.watchdog_timer_bark_thold_hit_cg.sample(intr_status_exp[WDOG], bark_thold, rtl_count);
    end
  end
endtask : collect_wdog_bark_timer_coverage
>>>>>>> Stashed changes

task aon_timer_scoreboard::run_wdog_bark_timer();
  event sample_coverage;
  forever begin
    wait (wdog_en);
    fork
      begin
        forever begin
          @(sample_coverage.triggered);
          if (cfg.en_cov) begin
            bit [31:0] rtl_count;
            //reading RTL value since TB doesn't keep track of count
            csr_rd(.ptr(ral.wdog_count), .value(rtl_count), .backdoor(1));
            cov.watchdog_timer_bark_thold_hit_cg.sample(intr_status_exp[WDOG],
                                                        bark_thold,
                                                        rtl_count);
          end
        end
      end
      begin
        // trying to count how many cycles we need to count
        uint count = 0;
        // It takes 4 aon clks from the write enabling the watchdog to take effect due to the CDC
        // logic.
        cfg.aon_clk_rst_vif.wait_clks(4);
        while (count < wdog_bark_num) begin
          wait_for_sleep();
          cfg.aon_clk_rst_vif.wait_clks(1);
          // reset the cycle counter when we update the cycle count needed
          count = wdog_num_update_due ? 0 : (count + 1);
          `uvm_info(`gfn, $sformatf("WDOG Bark Timer count: %d", count), UVM_HIGH)
          -> sample_coverage;
        end
        `uvm_info(`gfn, $sformatf("WDOG Bark Timer expired check for interrupts"), UVM_HIGH)
        intr_status_exp[WDOG] = 1'b1;
        -> sample_coverage;

        // Propagation delay of one cycle from aon_core to interrupt pins.
        cfg.aon_clk_rst_vif.wait_clks(1);
<<<<<<< Updated upstream
        // Wait a further 5 clocks for the interrupt to propagate through logic in the clk domain
        // to become visible on the top-level pins.
        cfg.clk_rst_vif.wait_clks(5);
        // Check interrupt and reset_req pins
        `DV_CHECK_CASE_EQ(intr_status_exp[WDOG],
                          cfg.intr_vif.sample_pin(.idx(WDOG)))

        // Check wakeup pin
        `DV_CHECK_CASE_EQ(intr_status_exp[WDOG],
                          cfg.aon_intr_vif.sample_pin(.idx(1)))
        `uvm_info(`gfn,
                  $sformatf("WDOG INTR Bark: %d",
                            intr_status_exp[WDOG]),
                  UVM_HIGH)
=======
        // If the enable becomes unset at this time the prediction needs to finish
        predicting_interrupt = 1;

        // Using flag to predict the interrupt
        local_interrupt = intr_status_exp[WDOG];
        intr_status_exp[WDOG] = 1;
        // If not using aux variable 'intr_status_exp' may update prior to the call to
        // predict_intr_state and end up predicting wrong value
        local_intr_exp = intr_status_exp;
        fork wdog_intr_predicted_values(local_intr_exp[WDOG]); join_none
        $display("Predicting intr_state WDOG");
//        fork predict_intr_state(local_intr_exp); join_none

 //       update_timed_regs_wdog();
        intr_status_exp[WDOG] = local_interrupt;

        // The register 'intr_state' will be updated 4 clock cycles after the counter > threshold
        // Otherwise, the TB may be predicting the value too early and mess up comparisons
//        cfg.clk_rst_vif.wait_clks(4);


        // Synchronising to the pos-edge before counting edges in case the current cycle is already ongoing
        // to avoid updating the mirrored value too early
//        @(cfg.clk_rst_vif.cb);
//        cfg.clk_rst_vif.wait_clks(3);

//        @(cfg.clk_rst_vif.cbn);
        @(cfg.clk_rst_vif.cb);
        cfg.clk_rst_vif.wait_n_clks(3);
        // Wait another cycle if "DE" is set, we don't want to update mirrored value early
        $display("hdl_read_bit(hw2reg.intr_state.wdog_timer_bark.de) = %0d",hdl_read_bit(".hw2reg.intr_state.wdog_timer_bark.de"));
        if (hdl_read_bit(".hw2reg.intr_state.wdog_timer_bark.de") == 1) begin
          cfg.clk_rst_vif.wait_n_clks(1); // extra cycle to update intr_state in time
          $display("DE is set");
        end
        // Using flag to predict the interrupt
        local_interrupt = intr_status_exp[WDOG];
        intr_status_exp[WDOG] = 1;
        update_timed_regs_wdog();

        // If not using aux variable 'intr_status_exp' may update prior to the call to
        // predict_intr_state and end up predicting wrong value
        local_intr_exp = intr_status_exp;
        fork wdog_intr_predicted_values(local_intr_exp[WDOG]); join_none
//        $display("Predicting intr_state WDOG");
        `uvm_info("RM_ME", $sformatf("[WDOG] - Predicting intr_state - 0x%0x",local_intr_exp), UVM_DEBUG)
//        fork predict_intr_state(local_intr_exp); join_none
        fork predict_intr_state(.pred_intr_state(local_intr_exp), .field_only(1), .is_wkup(0)); join_none

        intr_status_exp[WDOG] = local_interrupt;


        // Wait a further 5 clocks for the interrupt to propagate through logic in the clk domain
        // to become visible on the top-level pins.
//        cfg.clk_rst_vif.wait_clks(5);
//        cfg.clk_rst_vif.wait_clks(1);
        cfg.clk_rst_vif.wait_clks(2);
        // Check `intr_wdog_timer_bark_o`
        check_aon_domain_interrupt(.timer_type(WDOG));

        // It could happen intr_state reg was written betwen the time intr_status_exp[WDOG] was
        // set until the output 'intr_wdog_timer_bark' was compared and set, if that's the case
        // we set the variable to the value it should be
        csr_rd(.ptr(ral.intr_state), .value(backdoor_intr_state), .backdoor(1));

        if (backdoor_intr_state[WDOG] == 1 && intr_status_exp[WDOG]==0) begin
          `uvm_info(`gfn, {"Tweaking 'intr_status_exp[WDOG]=1' due to a write",
                           "since the value was predicted"}, UVM_DEBUG)
          intr_status_exp[WDOG] = 1; // Set again to ensure TB is in sync
        end
        // Reading actual enable to see if the output will be set
        csr_rd(.ptr(ral.wdog_ctrl.enable), .value(is_enabled), .backdoor(1));
        // Check reset_req pins:
        if (is_enabled) begin
          bit predicted_wkup_req = (ral.wkup_cause.predicting_value==0) ?
              `gmv(ral.wkup_cause) : wkup_cause;
          $display("ral.wkup_cause.predicting_value = %0d",ral.wkup_cause.predicting_value);
          $display("`gmv(ral.wkup_cause) = %0d",`gmv(ral.wkup_cause));
          $display("wkup_cause = %0d",wkup_cause);
          // If the write to wkup_cause wasn't absorved in this same cycle, we compare against the
          // prediction
          if (last_wkup_cause_write_aon_clk_cycle != aon_clk_cycle) begin
            `DV_CHECK_CASE_EQ(predicted_wkup_req, cfg.aon_intr_vif.sample_pin(.idx(1)))
          end
          else if (!(cfg.aon_intr_vif.sample_pin(.idx(1)) inside {0, predicted_wkup_req})) begin
            // Otherwise, check whether the write to wkup_cause(0x0) unset the interrupt or
            // whether it matches the predicted value
            `uvm_fatal(`gfn, $sformatf("aon_wkup_req_o comparison failed (not matching 0/%0d)",
                       predicted_wkup_req))
          end
        end // if (is_enabled)
        else begin
          // If disabled, wkup_output will be 0/1 depending on the value the flop ended up latching.
          // Hence we don't compare
        end
        `uvm_info(`gfn,$sformatf("WDOG INTR Bark: %d", intr_status_exp[WDOG]), UVM_HIGH)
        predicting_interrupt = 0;
>>>>>>> Stashed changes
      end
      begin
        wait (!wdog_en || !cfg.aon_clk_rst_vif.rst_n);
        `uvm_info(`gfn, $sformatf("WDOG Timer disabled, quit scoring"), UVM_HIGH)
        wdog_en = 0;
      end
    join_any
    disable fork;
  end
endtask : run_wdog_bark_timer

<<<<<<< Updated upstream
=======
task aon_timer_scoreboard::wait_for_wdog_enable_matching(bit enable);
  bit local_enable;
  do begin
    csr_rd(.ptr(ral.wdog_ctrl.enable), .value(local_enable), .backdoor(1));
    `uvm_info(`gfn, $sformatf("[backdoor read] : WDOG_CTRL.enable = 0x%0x",local_enable),
              UVM_DEBUG)
    if (local_enable != enable)
      cfg.aon_clk_rst_vif.wait_clks(1);
  end
  while(local_enable != enable);
endtask : wait_for_wdog_enable_matching

task aon_timer_scoreboard::wait_for_wkup_enable_matching(bit enable);
  fork
    begin: iso_fork
      fork 
      begin
      bit local_enable;
      do begin
        csr_rd(.ptr(ral.wkup_ctrl.enable), .value(local_enable), .backdoor(1));
        `uvm_info(`gfn, $sformatf("[backdoor read] : WKUP_CTRL.enable = 0x%0x",local_enable),
                  UVM_DEBUG)
      if (local_enable != enable)
        cfg.aon_clk_rst_vif.wait_clks(1);
      end
    while(local_enable != enable);
    end
    begin
       wait(cfg.under_reset);
    end
  join_any 
  disable fork;
  end
  join
endtask : wait_for_wkup_enable_matching

task aon_timer_scoreboard::collect_wdog_bite_timer_coverage(ref event sample_coverage);
  forever begin
    @ (sample_coverage);
    if (cfg.en_cov) begin
      bit [31:0] rtl_count;
      //reading RTL value since TB doesn't keep track of count
      csr_rd(.ptr(ral.wdog_count), .value(rtl_count), .backdoor(1));
      cov.watchdog_timer_bite_thold_hit_cg.sample(wdog_rst_req_exp, bite_thold, rtl_count);
    end
  end
endtask : collect_wdog_bite_timer_coverage

>>>>>>> Stashed changes
task aon_timer_scoreboard::run_wdog_bite_timer();
  event sample_coverage;
  forever begin
    wait (wdog_en);
    fork
      begin
        forever begin
          @(sample_coverage.triggered);
          if (cfg.en_cov) begin
            bit [31:0] rtl_count;
            //reading RTL value since TB doesn't keep track of count
            csr_rd(.ptr(ral.wdog_count), .value(rtl_count), .backdoor(1));
            cov.watchdog_timer_bite_thold_hit_cg.sample(wdog_rst_req_exp,
                                                        bite_thold,
                                                        rtl_count);
          end
        end
      end
      begin
        // trying to count how many cycles we need to count
        uint count = 0;
        // It takes 4 aon clks from the write enabling the watchdog to take effect due to the CDC
        // logic.
        cfg.aon_clk_rst_vif.wait_clks(4);
        while (count < wdog_bite_num) begin
          wait_for_sleep();
          cfg.aon_clk_rst_vif.wait_clks(1);
          // reset the cycle counter when we update the cycle count needed
          count = wdog_num_update_due ? 0 : (count + 1);
          `uvm_info(`gfn, $sformatf("WDOG Bite Timer count: %d", count), UVM_HIGH)
          -> sample_coverage;
        end
        `uvm_info(`gfn, $sformatf("WDOG Bite Timer expired check for interrupts"), UVM_HIGH)
        wdog_rst_req_exp = 1'b1;
        -> sample_coverage;

        // Propagation delay of one cycle from aon_core to interrupt pins.
        cfg.aon_clk_rst_vif.wait_clks(1);
        // Wait a further 5 clocks for the interrupt to propagate through logic in the clk domain
        // to become visible on the top-level pins.
        cfg.clk_rst_vif.wait_clks(5);
        // Check reset_req pin
        `DV_CHECK_CASE_EQ(wdog_rst_req_exp,
                          cfg.aon_intr_vif.sample_pin(.idx(0)))

        `uvm_info(`gfn,
                  $sformatf("WDOG INTR Bite: %d",
                            wdog_rst_req_exp),
                  UVM_HIGH)
      end
      begin
        wait (!wdog_en || !cfg.aon_clk_rst_vif.rst_n);
        `uvm_info(`gfn, $sformatf("WDOG Timer disabled, quit scoring"), UVM_HIGH)
        wdog_en = 0;
      end
    join_any
    disable fork;
  end
endtask : run_wdog_bite_timer
<<<<<<< Updated upstream
=======

function void aon_timer_scoreboard::reset(string kind = "HARD");
  super.reset(kind);
  // reset local fifos queues and variables
  `uvm_info(`gfn, "Resetting scoreboard", UVM_DEBUG)
  intr_status_exp = 0;
  wkup_cause = 0;

  ongoing_intr_state_read = 0;
  predicted_wkup_intr_q = {};
  predicted_wdog_intr_q = {};
  // the register is initialised to 0x0
  predicted_wdog_intr_q.push_back(0);
  predicted_wkup_intr_q.push_back(0);
//  extern virtual function void reset(string kind = "HARD");
  `uvm_info(`gfn, "Triggering 'sys_reset_ev'", UVM_DEBUG)
  -> sys_reset_ev;
endfunction : reset
>>>>>>> Stashed changes
