CAPI=2:
# Copyright lowRISC contributors (OpenTitan project).
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
name: ${instance_vlnv("lowrisc:dv:rstmgr_env:0.1")}
description: "RSTMGR DV UVM environment"
filesets:
  files_rtl:
    depend:
      - ${instance_vlnv("lowrisc:ip:rstmgr")}

  files_dv:
    depend:
      - lowrisc:dv:ralgen
      - lowrisc:dv:cip_lib
      - ${instance_vlnv("lowrisc:ip:rstmgr_pkg")}
      - ${instance_vlnv("lowrisc:constants:top_pkg")}

    files:
      - rstmgr_env_pkg.sv
      - rstmgr_env_cfg.sv: {is_include_file: true}
      - rstmgr_env_cov.sv: {is_include_file: true}
      - rstmgr_virtual_sequencer.sv: {is_include_file: true}
      - rstmgr_scoreboard.sv: {is_include_file: true}
      - rstmgr_env.sv: {is_include_file: true}
      - seq_lib/rstmgr_vseq_list.sv: {is_include_file: true}
      - seq_lib/rstmgr_base_vseq.sv: {is_include_file: true}
      - seq_lib/rstmgr_common_vseq.sv: {is_include_file: true}
      - seq_lib/rstmgr_por_stretcher_vseq.sv: {is_include_file: true}
      - seq_lib/rstmgr_reset_vseq.sv: {is_include_file: true}
      - seq_lib/rstmgr_smoke_vseq.sv: {is_include_file: true}
      - seq_lib/rstmgr_stress_all_vseq.sv: {is_include_file: true}
      - seq_lib/rstmgr_sw_rst_reset_race_vseq.sv: {is_include_file: true}
      - seq_lib/rstmgr_sw_rst_vseq.sv: {is_include_file: true}
      - seq_lib/rstmgr_sec_cm_scan_intersig_mubi_vseq.sv: {is_include_file: true}
      - seq_lib/rstmgr_leaf_rst_cnsty_vseq.sv: {is_include_file: true}
      - seq_lib/rstmgr_leaf_rst_shadow_attack_vseq.sv: {is_include_file: true}
      - rstmgr_if.sv
    file_type: systemVerilogSource

generate:
  ral:
    generator: ralgen
    parameters:
      name: rstmgr
      ip_hjson: ../../data/rstmgr.hjson

targets:
  default:
    filesets:
      - files_dv
      - files_rtl
    generate:
      - ral
